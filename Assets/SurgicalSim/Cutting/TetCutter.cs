// TetCutter.cs
// 核心切割执行器
// 功能：接收切割请求 → 查找相交 tet → 标记 inactive → 更新 GPU → 重建表面

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Cutting
{
    /// <summary>
    /// 切割请求数据
    /// </summary>
    public struct CutRequest
    {
        public Vector3 position;     // 切割点世界坐标
        public Vector3 prevPosition; // 上一帧切割点（用于连续切割线段）
        public float   radius;       // 切割影响半径
        public bool    isActive;     // 是否正在切割
        public bool    hasPrev;      // 是否有上一帧位置
    }

    /// <summary>
    /// 切割结果
    /// </summary>
    public struct CutResult
    {
        public int removedCount;          // 本次删除的 tet 数
        public int totalRemovedCount;     // 累计删除的 tet 数
        public bool surfaceUpdated;       // 表面是否需要更新
        public List<int> affectedVerts;   // 受影响的顶点（用于施加扰动）
    }

    public class TetCutter
    {
        TetMeshData     _data;
        XPBDSolverGPU   _solver;

        int _totalRemoved = 0;
        bool _surfaceDirty = false;
        bool _bufferDirty = false;  // GPU buffer 是否需要上传

        // 当前表面三角形（动态更新）
        int[] _currentSurfaceTris;

        public int TotalRemovedTets => _totalRemoved;
        public bool IsSurfaceDirty => _surfaceDirty;
        public bool IsBufferDirty => _bufferDirty;

        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data   = data;
            _solver = solver;
            _totalRemoved = 0;
            _surfaceDirty = false;
            _bufferDirty = false;
            _currentSurfaceTris = data.SurfaceTriIds;
        }

        /// <summary>
        /// 执行切割（只标记 tet，不立即上传 GPU）
        /// </summary>
        public CutResult ExecuteCut(CutRequest request)
        {
            var result = new CutResult();
            result.affectedVerts = new List<int>();
            if (!request.isActive || _data == null) return result;

            // 1. 查找被切割的 tet
            List<int> hitTets;

            if (request.hasPrev)
            {
                hitTets = TetIntersection.FindTetsAlongSegment(
                    _data, request.prevPosition, request.position, request.radius);
            }
            else
            {
                hitTets = TetIntersection.FindTetsInSphere(
                    _data, request.position, request.radius);
            }

            if (hitTets.Count == 0) return result;

            // 2. 标记为 inactive，收集受影响顶点
            var affectedSet = new HashSet<int>();
            foreach (int t in hitTets)
            {
                if (_data.TetActive[t])
                {
                    _data.TetActive[t] = false;
                    result.removedCount++;
                    _totalRemoved++;

                    // 收集这个 tet 的 4 个顶点
                    affectedSet.Add(_data.TetIds[t*4+0]);
                    affectedSet.Add(_data.TetIds[t*4+1]);
                    affectedSet.Add(_data.TetIds[t*4+2]);
                    affectedSet.Add(_data.TetIds[t*4+3]);
                }
            }

            if (result.removedCount > 0)
            {
                _bufferDirty = true;
                _surfaceDirty = true;
                result.surfaceUpdated = true;
                result.affectedVerts = new List<int>(affectedSet);
            }

            result.totalRemovedCount = _totalRemoved;
            return result;
        }

        // 待扰动的顶点列表（累积，在 FlushToGPU 中一次性处理）
        List<int> _pendingPerturbVerts = new List<int>();
        Vector3 _lastCutDirection = Vector3.down;

        /// <summary>
        /// 记录切割方向（CuttingTool 调用）
        /// </summary>
        public void SetCutDirection(Vector3 dir) => _lastCutDirection = dir;

        /// <summary>
        /// 上传脏 buffer 到 GPU + 对切面粒子施加扰动
        /// </summary>
        public void FlushToGPU()
        {
            if (!_bufferDirty || _solver == null) return;

            // 1. 上传 TetActive + EdgeActive
            _solver.UploadTetActiveBuffer(_data.TetActive);

            // 2. 对受影响粒子施加速度扰动（让切面能展开）
            if (_pendingPerturbVerts.Count > 0)
            {
                // 先从 GPU 读回最新位置
                _solver.ReadbackPositions(_data);

                foreach (int vi in _pendingPerturbVerts)
                {
                    if (_data.InvMass[vi] == 0f) continue; // 固定点跳过

                    // 计算该粒子附近的"切面法线"：从切割中心指向粒子的方向
                    // 这会让切面两侧的粒子向相反方向弹开
                    Vector3 perturbDir = _lastCutDirection.normalized;
                    if (perturbDir.sqrMagnitude < 0.01f)
                        perturbDir = Vector3.down;

                    // 施加一个小速度扰动
                    _data.Velocities[vi] += perturbDir * 0.3f;
                }

                // 上传修改后的速度到 GPU
                _solver.UploadPositionsAndVelocities(_data);
                _pendingPerturbVerts.Clear();
            }

            _bufferDirty = false;
        }

        /// <summary>
        /// 添加待扰动顶点
        /// </summary>
        public void AddPerturbVerts(List<int> verts)
        {
            _pendingPerturbVerts.AddRange(verts);
        }

        /// <summary>
        /// 重建表面（在切割后调用）
        /// </summary>
        public int[] RebuildSurface()
        {
            if (!_surfaceDirty) return _currentSurfaceTris;

            _currentSurfaceTris = SurfaceReconstructor.RebuildSurface(_data);
            _surfaceDirty = false;

            return _currentSurfaceTris;
        }

        /// <summary>
        /// 获取当前表面三角形
        /// </summary>
        public int[] GetCurrentSurfaceTris() => _currentSurfaceTris;
    }
}
