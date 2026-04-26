// EdgeSnapper.cs
// 基于边约束禁用的切割系统
// 核心思想：不重建 solver，只禁用被切割平面穿过的边约束
// 配合 CutSurfaceMesher 生成光滑的切面几何
//
// 工作流：
// 1. SurgicalTool 提供工具扫掠体（上一帧→当前帧的尖端轨迹）
// 2. EdgeSnapper 找到被扫掠体穿过的所有 tet 边
// 3. 禁用这些边的距离约束（调用 solver.DisableEdges）
// 4. 生成切面几何（使用精确的交叉点坐标实现光滑切面）

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Cutting
{
    public class EdgeSnapper
    {
        TetMeshData   _data;
        XPBDSolverGPU _solver;

        // 已禁用的边集合（防止重复处理）
        HashSet<int> _disabledEdges = new HashSet<int>();

        // 顶点→边 的映射（加速查询）
        List<int>[] _vertToEdges;

        // 边的 key → 边索引的映射
        Dictionary<long, int> _edgeKeyToIdx;

        // 切面几何（累积）
        List<Vector3> _cutVerts   = new List<Vector3>();
        List<int>     _cutTris    = new List<int>();
        List<Vector3> _cutNormals = new List<Vector3>();

        public struct SnapResult
        {
            public int  newDisabledEdges;
            public int  newInactiveTets;
            public int  cutTriangles;
            public float elapsedMs;
        }

        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data   = data;
            _solver = solver;
            _disabledEdges.Clear();
            _cutVerts.Clear();
            _cutTris.Clear();
            _cutNormals.Clear();

            BuildLookups();
        }

        void BuildLookups()
        {
            int numE = _solver.NumEdges;
            int[] edgeIds = _solver.EdgeIds;
            int numP = _solver.NumParticles;

            // 顶点 → 边列表
            _vertToEdges = new List<int>[numP];
            for (int i = 0; i < numP; i++) _vertToEdges[i] = new List<int>(8);

            _edgeKeyToIdx = new Dictionary<long, int>(numE);

            for (int e = 0; e < numE; e++)
            {
                int a = edgeIds[e * 2];
                int b = edgeIds[e * 2 + 1];
                _vertToEdges[a].Add(e);
                _vertToEdges[b].Add(e);

                int lo = a < b ? a : b;
                int hi = a < b ? b : a;
                long key = (long)lo * 200000 + hi;
                _edgeKeyToIdx[key] = e;
            }

            Debug.Log($"[EdgeSnapper] 初始化 | 边: {numE} | 顶点: {numP}");
        }

        /// <summary>
        /// 执行切割：用一个扫掠平面切割
        /// tipPos: 工具尖端当前位置
        /// prevTipPos: 工具尖端上一帧位置
        /// toolDir: 工具方向
        /// toolRadius: 工具影响半径
        /// </summary>
        public SnapResult Cut(Vector3 tipPos, Vector3 prevTipPos,
                              Vector3 toolDir, float toolRadius)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var result = new SnapResult();

            // 计算切割平面
            // 平面由工具移动方向和工具方向共同定义
            Vector3 moveDir = (tipPos - prevTipPos);
            float moveDist = moveDir.magnitude;
            if (moveDist < 0.00001f) return result;
            moveDir /= moveDist;

            // 切割平面法线 = 工具方向 × 移动方向
            Vector3 planeNormal = Vector3.Cross(toolDir, moveDir).normalized;
            if (planeNormal.sqrMagnitude < 0.01f)
                planeNormal = Vector3.Cross(toolDir, Vector3.up).normalized;

            Vector3 planePoint = (tipPos + prevTipPos) * 0.5f;

            // ── Step 1: 找被切割区域的边 ────────────────────

            // 扫掠体范围内的所有边
            int numE = _solver.NumEdges;
            int[] edgeIds = _solver.EdgeIds;
            var edgesToDisable = new HashSet<int>();
            var crossPoints = new List<CutPoint>(); // 交叉点

            for (int e = 0; e < numE; e++)
            {
                if (_disabledEdges.Contains(e)) continue;
                if (!_solver.EdgeActive[e]) continue;

                int vi0 = edgeIds[e * 2];
                int vi1 = edgeIds[e * 2 + 1];

                Vector3 p0 = _data.Positions[vi0];
                Vector3 p1 = _data.Positions[vi1];

                // 检查边是否在工具影响范围内
                Vector3 edgeMid = (p0 + p1) * 0.5f;
                float distToTool = PointSegmentDist(edgeMid, prevTipPos, tipPos);
                if (distToTool > toolRadius * 5f) continue;

                // 检查边是否被切割平面穿过
                float d0 = Vector3.Dot(p0 - planePoint, planeNormal);
                float d1 = Vector3.Dot(p1 - planePoint, planeNormal);

                if (d0 * d1 < 0) // 两端在不同侧
                {
                    // 计算精确交叉点
                    float t = Mathf.Abs(d0) / (Mathf.Abs(d0) + Mathf.Abs(d1));
                    Vector3 cutPos = Vector3.Lerp(p0, p1, t);

                    // 检查交叉点是否在工具扫掠范围内
                    float distCut = PointSegmentDist(cutPos, prevTipPos, tipPos);
                    if (distCut <= toolRadius * 3f)
                    {
                        edgesToDisable.Add(e);
                        crossPoints.Add(new CutPoint
                        {
                            position = cutPos,
                            edgeIdx  = e,
                            vi0 = vi0, vi1 = vi1,
                            t = t,
                            normal = planeNormal
                        });
                    }
                }
            }

            if (edgesToDisable.Count == 0)
            {
                return result;
            }

            // ── Step 2: 只禁用边约束（不删除 tet！）──────────
            // 关键：不传 tetActiveOverride，不让 solver 禁用任何 tet
            // 所有 tet 保持 active → 表面完整 → 零宽度切割
            _solver.DisableEdgesOnly(edgesToDisable);

            foreach (int e in edgesToDisable)
                _disabledEdges.Add(e);

            result.newDisabledEdges = edgesToDisable.Count;

            // ── Step 3: 生成光滑切面几何 ────────────────────
            GenerateSmoothCutSurface(crossPoints, planeNormal, ref result);

            // 注意：不更新 TetActive！所有 tet 保持 active
            result.newInactiveTets = 0;

            sw.Stop();
            result.elapsedMs = (float)sw.Elapsed.TotalMilliseconds;

            if (result.newDisabledEdges > 0)
            {
                Debug.Log($"[EdgeSnapper] 切割 | 禁用边: +{result.newDisabledEdges} (总: {_disabledEdges.Count}) | " +
                          $"耗时: {result.elapsedMs:F1}ms");
            }

            return result;
        }

        /// <summary>
        /// 使用交叉点生成光滑切面
        /// 关键：不使用 tet 顶点，而是使用精确的边-平面交叉点
        /// </summary>
        void GenerateSmoothCutSurface(List<CutPoint> points, Vector3 normal, ref SnapResult result)
        {
            if (points.Count < 3) return;

            // 将交叉点投影到切割平面上，用 fan triangulation 生成三角形
            Vector3 center = Vector3.zero;
            foreach (var p in points) center += p.position;
            center /= points.Count;

            // 按角度排序
            Vector3 u = (points[0].position - center).normalized;
            if (u.sqrMagnitude < 0.001f) u = Vector3.right;
            Vector3 v = Vector3.Cross(normal, u).normalized;

            points.Sort((a, b) =>
            {
                Vector3 da = a.position - center;
                Vector3 db = b.position - center;
                float angleA = Mathf.Atan2(Vector3.Dot(da, v), Vector3.Dot(da, u));
                float angleB = Mathf.Atan2(Vector3.Dot(db, v), Vector3.Dot(db, u));
                return angleA.CompareTo(angleB);
            });

            // Fan triangulation from center
            int baseIdx = _cutVerts.Count;
            _cutVerts.Add(center);
            _cutNormals.Add(normal);

            for (int i = 0; i < points.Count; i++)
            {
                _cutVerts.Add(points[i].position);
                _cutNormals.Add(normal);
            }

            for (int i = 0; i < points.Count; i++)
            {
                int next = (i + 1) % points.Count;
                // 正面
                _cutTris.Add(baseIdx);           // center
                _cutTris.Add(baseIdx + 1 + i);   // current
                _cutTris.Add(baseIdx + 1 + next); // next
                // 反面
                _cutTris.Add(baseIdx);
                _cutTris.Add(baseIdx + 1 + next);
                _cutTris.Add(baseIdx + 1 + i);

                result.cutTriangles += 2;
            }
        }

        // ── 工具方法 ─────────────────────────────────────────

        struct CutPoint
        {
            public Vector3 position;
            public int     edgeIdx;
            public int     vi0, vi1;
            public float   t;
            public Vector3 normal;
        }

        /// <summary>点到线段的最短距离</summary>
        static float PointSegmentDist(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Vector3.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-10f);
            t = Mathf.Clamp01(t);
            Vector3 closest = a + ab * t;
            return (p - closest).magnitude;
        }

        // ── 公开属性 ─────────────────────────────────────────
        public List<Vector3> CutVerts   => _cutVerts;
        public List<int>     CutTris    => _cutTris;
        public List<Vector3> CutNormals => _cutNormals;
        public int TotalDisabledEdges   => _disabledEdges.Count;
    }
}
