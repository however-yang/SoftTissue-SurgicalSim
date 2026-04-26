// VertexSplitter.cs
// 真正的零宽度切割：顶点分裂
// 核心思想：切割平面穿过 tet 的边时，不删除 tet，
// 而是将切面边界上的共享顶点复制一份，两侧 tet 引用不同副本
// 这样切口自然闭合（零宽度），只有在物理力作用下才会张开
//
// 参考: CudaMeshCutting merge() 的 UnionFind 顶点合并逻辑

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Cutting
{
    public class VertexSplitter
    {
        TetMeshData   _data;
        XPBDSolverGPU _solver;

        // 累积切面几何
        List<Vector3> _allCutVerts   = new List<Vector3>();
        List<int>     _allCutTris    = new List<int>();
        List<Vector3> _allCutNormals = new List<Vector3>();

        // 已经分裂过的边（防止重复分裂）
        HashSet<long> _splitEdges = new HashSet<long>();

        // 分裂结果
        public struct SplitResult
        {
            public int  splitVertexCount;   // 新增的顶点数
            public int  cutTriangleCount;   // 新增的切面三角形数
            public bool topologyChanged;    // 拓扑是否变化
            public float elapsedMs;
        }

        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data   = data;
            _solver = solver;
            _splitEdges.Clear();
            _allCutVerts.Clear();
            _allCutTris.Clear();
            _allCutNormals.Clear();
        }

        /// <summary>
        /// 执行零宽度切割：vertex splitting
        /// planePoint: 切割平面上的一点
        /// planeNormal: 切割平面法线
        /// affectedTets: 预筛选的受影响 tet 列表
        /// </summary>
        public SplitResult ExecuteSplit(Vector3 planePoint, Vector3 planeNormal,
                                       List<int> affectedTets)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var result = new SplitResult();

            if (affectedTets == null || affectedTets.Count == 0)
                return result;

            planeNormal = planeNormal.normalized;

            // ── Step 1: 分类顶点 ───────────────────────────────
            // 计算每个受影响顶点到切割平面的距离
            var vertexSide = new Dictionary<int, int>(); // vertexIdx → +1/-1
            var affectedVerts = new HashSet<int>();

            foreach (int t in affectedTets)
            {
                if (!_data.TetActive[t]) continue;
                int b = t * 4;
                for (int i = 0; i < 4; i++)
                {
                    int vi = _data.TetIds[b + i];
                    if (!vertexSide.ContainsKey(vi))
                    {
                        float d = Vector3.Dot(_data.Positions[vi] - planePoint, planeNormal);
                        vertexSide[vi] = d >= 0 ? 1 : -1;
                    }
                    affectedVerts.Add(vi);
                }
            }

            // ── Step 2: 找到需要分裂的顶点 ────────────────────
            // 如果一个顶点被 *两侧* 的 tet 共享，它就需要分裂
            // 具体做法：对每个顶点，检查它所属的所有 *受影响* tet 是否在同一侧

            // 先建立受影响顶点 → 受影响 tet 的映射
            var vert2Tets = new Dictionary<int, List<int>>();
            foreach (int t in affectedTets)
            {
                if (!_data.TetActive[t]) continue;
                int b = t * 4;
                for (int i = 0; i < 4; i++)
                {
                    int vi = _data.TetIds[b + i];
                    if (!vert2Tets.ContainsKey(vi))
                        vert2Tets[vi] = new List<int>();
                    vert2Tets[vi].Add(t);
                }
            }

            // 确定每个 tet 的"侧面"：用多数顶点的侧面决定
            var tetSide = new Dictionary<int, int>();
            foreach (int t in affectedTets)
            {
                if (!_data.TetActive[t]) continue;
                int b = t * 4;
                int sideSum = 0;
                for (int i = 0; i < 4; i++)
                    sideSum += vertexSide[_data.TetIds[b + i]];
                tetSide[t] = sideSum >= 0 ? 1 : -1;
            }

            // 找需要分裂的顶点：它在两侧的 tet 中都出现了
            var vertsToSplit = new HashSet<int>();
            foreach (var kv in vert2Tets)
            {
                int vi = kv.Key;
                bool hasPos = false, hasNeg = false;
                foreach (int t in kv.Value)
                {
                    if (!tetSide.ContainsKey(t)) continue;
                    if (tetSide[t] > 0) hasPos = true;
                    else hasNeg = true;
                }
                if (hasPos && hasNeg)
                    vertsToSplit.Add(vi);
            }

            if (vertsToSplit.Count == 0)
                return result;

            // ── Step 3: 执行顶点分裂 ──────────────────────────
            // 对每个需要分裂的顶点：
            //   - 保留原始顶点给 positive 侧的 tet
            //   - 创建一个新顶点给 negative 侧的 tet
            //   - 新顶点位置 = 原始位置（零宽度！）

            int origNumParticles = _data.NumParticles;
            var newPositions  = new List<Vector3>(_data.Positions);
            var newRestPos    = new List<Vector3>(_data.RestPositions);
            var newPrevPos    = new List<Vector3>(_data.PrevPositions);
            var newVelocities = new List<Vector3>(_data.Velocities);
            var newInvMass    = new List<float>(_data.InvMass);
            var newTetActive  = new List<bool>(_data.TetActive);

            // 复制 TetIds 为可修改列表
            int[] tetIds = (int[])_data.TetIds.Clone();

            // oldVertex → newVertex 的映射（只对 negative 侧）
            var splitMap = new Dictionary<int, int>(); // 原始顶点 → 新顶点索引

            foreach (int vi in vertsToSplit)
            {
                int newIdx = newPositions.Count;
                splitMap[vi] = newIdx;

                // 复制顶点属性（位置完全相同 → 零宽度！）
                newPositions.Add(_data.Positions[vi]);
                newRestPos.Add(_data.RestPositions[vi]);
                newPrevPos.Add(_data.PrevPositions[vi]);
                newVelocities.Add(_data.Velocities[vi]);
                newInvMass.Add(_data.InvMass[vi]);

                result.splitVertexCount++;
            }

            // ── Step 4: 更新 tet 索引 ─────────────────────────
            // negative 侧的 tet：将分裂顶点的引用改为新顶点
            foreach (int t in affectedTets)
            {
                if (!_data.TetActive[t]) continue;
                if (!tetSide.ContainsKey(t)) continue;
                if (tetSide[t] >= 0) continue; // positive 侧保持不变

                int b = t * 4;
                for (int i = 0; i < 4; i++)
                {
                    int vi = tetIds[b + i];
                    if (splitMap.TryGetValue(vi, out int newIdx))
                    {
                        tetIds[b + i] = newIdx;
                    }
                }
            }

            // ── Step 5: 应用到 TetMeshData ───────────────────
            _data.Positions     = newPositions.ToArray();
            _data.SetRestPositions(newRestPos.ToArray());
            _data.PrevPositions = newPrevPos.ToArray();
            _data.Velocities    = newVelocities.ToArray();
            _data.InvMass       = newInvMass.ToArray();
            _data.TetActive     = newTetActive.ToArray();
            _data.SetTetIds(tetIds);
            _data.SetNumParticles(newPositions.Count);

            // ── Step 6: 生成切面三角形 ────────────────────────
            GenerateCutSurface(affectedTets, tetSide, planePoint, planeNormal, result);

            result.topologyChanged = result.splitVertexCount > 0;

            sw.Stop();
            result.elapsedMs = (float)sw.Elapsed.TotalMilliseconds;

            if (result.splitVertexCount > 0)
            {
                Debug.Log($"[VertexSplitter] 分裂完成 | 新顶点: {result.splitVertexCount} | " +
                          $"总顶点: {_data.NumParticles} | 切面三角形: {result.cutTriangleCount} | " +
                          $"耗时: {result.elapsedMs:F1}ms");
            }

            return result;
        }

        /// <summary>
        /// 生成切面三角形：在切割平面与 tet 边的交叉点处
        /// </summary>
        void GenerateCutSurface(List<int> affectedTets, Dictionary<int, int> tetSide,
                                Vector3 planePoint, Vector3 planeNormal, SplitResult result)
        {
            foreach (int t in affectedTets)
            {
                if (!_data.TetActive[t]) continue;

                int b = t * 4;
                // 找到这个 tet 中被切割平面穿过的边的交叉点
                var cutPoints = new List<Vector3>();

                for (int e = 0; e < 6; e++)
                {
                    int li0 = TetTopology.EdgePairs[e, 0];
                    int li1 = TetTopology.EdgePairs[e, 1];
                    int vi0 = _data.TetIds[b + li0];
                    int vi1 = _data.TetIds[b + li1];

                    float d0 = Vector3.Dot(_data.Positions[vi0] - planePoint, planeNormal);
                    float d1 = Vector3.Dot(_data.Positions[vi1] - planePoint, planeNormal);

                    if (d0 * d1 < 0) // 不同侧
                    {
                        float absD0 = Mathf.Abs(d0);
                        float absD1 = Mathf.Abs(d1);
                        float t_param = absD0 / (absD0 + absD1);
                        Vector3 cutPos = Vector3.Lerp(
                            _data.Positions[vi0], _data.Positions[vi1], t_param);
                        cutPoints.Add(cutPos);
                    }
                }

                if (cutPoints.Count >= 3)
                {
                    int baseIdx = _allCutVerts.Count;

                    if (cutPoints.Count == 3)
                    {
                        _allCutVerts.AddRange(cutPoints);
                        // 双面：正面 + 反面
                        _allCutTris.Add(baseIdx); _allCutTris.Add(baseIdx+1); _allCutTris.Add(baseIdx+2);
                        _allCutTris.Add(baseIdx); _allCutTris.Add(baseIdx+2); _allCutTris.Add(baseIdx+1);
                        _allCutNormals.Add(planeNormal); _allCutNormals.Add(planeNormal); _allCutNormals.Add(planeNormal);
                        result.cutTriangleCount += 2;
                    }
                    else if (cutPoints.Count == 4)
                    {
                        // 排序四边形顶点
                        OrderQuadVerts(cutPoints, planeNormal);
                        _allCutVerts.AddRange(cutPoints);
                        // 正面
                        _allCutTris.Add(baseIdx); _allCutTris.Add(baseIdx+1); _allCutTris.Add(baseIdx+2);
                        _allCutTris.Add(baseIdx); _allCutTris.Add(baseIdx+2); _allCutTris.Add(baseIdx+3);
                        // 反面
                        _allCutTris.Add(baseIdx); _allCutTris.Add(baseIdx+2); _allCutTris.Add(baseIdx+1);
                        _allCutTris.Add(baseIdx); _allCutTris.Add(baseIdx+3); _allCutTris.Add(baseIdx+2);
                        for (int i = 0; i < 4; i++) _allCutNormals.Add(planeNormal);
                        result.cutTriangleCount += 4;
                    }
                }
            }
        }

        void OrderQuadVerts(List<Vector3> verts, Vector3 normal)
        {
            if (verts.Count != 4) return;
            Vector3 center = (verts[0] + verts[1] + verts[2] + verts[3]) * 0.25f;
            Vector3 u = (verts[0] - center).normalized;
            Vector3 v = Vector3.Cross(normal, u).normalized;
            float[] angles = new float[4];
            for (int i = 0; i < 4; i++)
            {
                Vector3 d = verts[i] - center;
                angles[i] = Mathf.Atan2(Vector3.Dot(d, v), Vector3.Dot(d, u));
            }
            for (int i = 0; i < 3; i++)
                for (int j = i + 1; j < 4; j++)
                    if (angles[j] < angles[i])
                    {
                        float ta = angles[i]; angles[i] = angles[j]; angles[j] = ta;
                        Vector3 tv = verts[i]; verts[i] = verts[j]; verts[j] = tv;
                    }
        }

        // ── 公开属性 ─────────────────────────────────────────
        public List<Vector3> CutSurfaceVerts   => _allCutVerts;
        public List<int>     CutSurfaceTris    => _allCutTris;
        public List<Vector3> CutSurfaceNormals => _allCutNormals;
    }
}
