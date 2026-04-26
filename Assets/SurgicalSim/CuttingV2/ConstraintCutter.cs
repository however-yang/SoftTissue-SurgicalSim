// ConstraintCutter.cs — 方案 C: Constraint-Based Vertex Splitting
//
// 核心思想:
//   不修改 tet 拓扑 (不删除/不细分)
//   只在切割面处复制顶点 + 断开跨切面的 edge 约束
//   两侧自然分离 → 零宽度切口 → 平滑无锯齿
//
// 算法:
//   1. 构建切割面 (锁定法线)
//   2. 找到工具路径附近被切面穿过的 tet
//   3. 找到需要分裂的顶点 (同时属于切面两侧 tet 的顶点)
//   4. 复制顶点, 负侧 tet 使用副本
//   5. 更新 edge 约束索引, 断开跨切面的边
//   6. 增量上传 GPU (不做 full reinit)
//
// 优势:
//   - Tet 数量不变 → Graph Coloring 不变 → 不需要 reinit
//   - 复制顶点初始位置重合 → 零宽度切口
//   - 不产生退化元素 → 天然平滑

using System;
using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.CuttingV2
{
    /// <summary>
    /// 约束切割器: 通过顶点复制 + 约束断裂实现零宽度平滑切割
    /// </summary>
    public class ConstraintCutter
    {
        // ── 外部引用 ─────────────────────────────────────────
        TetMeshData   _data;
        XPBDSolverGPU _solver;

        // ── 切割状态 ─────────────────────────────────────────
        Vector3 _lockedNormal;       // 锁定法线 (一次切割保持一致)
        int     _totalSplitVerts;    // 累计分裂顶点数
        int     _totalCutEdges;      // 累计断裂边数
        bool    _dirty;              // 有脏数据需要上传 GPU

        // ── 邻接关系 (CPU 端缓存) ────────────────────────────
        List<int>[] _vertToTets;     // vertexIndex → list of tet indices
        List<int>[] _vertToEdges;    // vertexIndex → list of edge indices

        // ── 持久边缓存 ──────────────────────────────────────
        // edgeKey → 已在该边上创建的分裂点索引
        Dictionary<long, int> _edgeSplitCache = new Dictionary<long, int>();

        // ── 参数 ────────────────────────────────────────────
        const float MIN_MOVE_DIST = 0.0002f;
        const int   MAX_AFFECTED_TETS = 60;

        // ── 公开属性 ─────────────────────────────────────────
        public int TotalSplitVerts => _totalSplitVerts;
        public int TotalCutEdges  => _totalCutEdges;
        public bool IsDirty       => _dirty;

        // ══════════════════════════════════════════════════════
        // 初始化
        // ══════════════════════════════════════════════════════

        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data   = data;
            _solver = solver;
            _totalSplitVerts = 0;
            _totalCutEdges   = 0;
            _dirty  = false;
            _lockedNormal = Vector3.zero;
            _edgeSplitCache.Clear();

            BuildAdjacency();

            Debug.Log($"[ConstraintCutter] 初始化 | V:{data.NumParticles} T:{data.NumTets} " +
                      $"E:{solver.NumEdges}");
        }

        /// <summary>构建 vertex→tet 和 vertex→edge 邻接表</summary>
        void BuildAdjacency()
        {
            int numV = _data.NumParticles;
            int numT = _data.NumTets;
            int numE = _solver.NumEdges;

            // vertex → tets
            _vertToTets = new List<int>[numV * 3]; // 预留3倍 (切割会新增顶点)
            for (int i = 0; i < _vertToTets.Length; i++)
                _vertToTets[i] = new List<int>(8);

            for (int t = 0; t < numT; t++)
            {
                if (!_data.TetActive[t]) continue;
                for (int k = 0; k < 4; k++)
                {
                    int v = _data.TetIds[t * 4 + k];
                    if (v < _vertToTets.Length)
                        _vertToTets[v].Add(t);
                }
            }

            // vertex → edges
            _vertToEdges = new List<int>[numV * 3];
            for (int i = 0; i < _vertToEdges.Length; i++)
                _vertToEdges[i] = new List<int>(8);

            int[] eids = _solver.EdgeIds;
            for (int e = 0; e < numE; e++)
            {
                int a = eids[e * 2], b = eids[e * 2 + 1];
                if (a < _vertToEdges.Length) _vertToEdges[a].Add(e);
                if (b < _vertToEdges.Length) _vertToEdges[b].Add(e);
            }
        }

        /// <summary>确保邻接表能容纳新增顶点</summary>
        void EnsureAdjacencyCapacity(int requiredSize)
        {
            if (_vertToTets.Length >= requiredSize) return;

            int newSize = Math.Max(requiredSize, _vertToTets.Length * 2);
            var newVT = new List<int>[newSize];
            Array.Copy(_vertToTets, newVT, _vertToTets.Length);
            for (int i = _vertToTets.Length; i < newSize; i++)
                newVT[i] = new List<int>(8);
            _vertToTets = newVT;

            var newVE = new List<int>[newSize];
            Array.Copy(_vertToEdges, newVE, _vertToEdges.Length);
            for (int i = _vertToEdges.Length; i < newSize; i++)
                newVE[i] = new List<int>(8);
            _vertToEdges = newVE;
        }

        // ══════════════════════════════════════════════════════
        // 主入口
        // ══════════════════════════════════════════════════════

        public struct CutResult
        {
            public int splitVerts;
            public int cutEdges;
            public int affectedTets;
            public float elapsedMs;
        }

        public CutResult Cut(Vector3 tipPos, Vector3 prevTipPos,
                             Vector3 toolDir, float toolRadius)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var result = new CutResult();

            Vector3 moveDir = tipPos - prevTipPos;
            float moveDist = moveDir.magnitude;
            if (moveDist < MIN_MOVE_DIST) return result;
            moveDir /= moveDist;

            // ── Step 1: 切割面 (锁定法线) ────────────────────
            Vector3 rawNormal = Vector3.Cross(toolDir, moveDir).normalized;
            if (rawNormal.sqrMagnitude < 0.01f)
                rawNormal = Vector3.Cross(toolDir, Vector3.up).normalized;
            if (_lockedNormal.sqrMagnitude < 0.01f)
                _lockedNormal = rawNormal;

            Vector3 planeN = _lockedNormal;
            Vector3 planeP = (tipPos + prevTipPos) * 0.5f;

            // ── Step 2: 找到受影响的 tet ─────────────────────
            var affectedTets = FindAffectedTets(tipPos, prevTipPos, toolRadius, planeN, planeP);
            if (affectedTets.Count == 0)
            {
                sw.Stop();
                return result;
            }
            result.affectedTets = affectedTets.Count;

            // ── Step 3: 找到需要分裂的顶点 ───────────────────
            // 条件: 该顶点被切面两侧的 affected tet 同时使用
            var splitMap = FindVerticesNeedingSplit(affectedTets, planeN, planeP);
            if (splitMap.Count == 0)
            {
                sw.Stop();
                return result;
            }

            // ── Step 4: 创建副本顶点 ────────────────────────
            CreateVertexCopies(splitMap);

            // ── Step 5: 更新 tet 索引 (负侧 tet 用副本) ──────
            UpdateTetIndices(affectedTets, splitMap, planeN, planeP);

            // ── Step 6: 更新 edge 约束 ──────────────────────
            int cutEdgeCount = UpdateEdgeConstraints(splitMap, planeN, planeP);

            _totalSplitVerts += splitMap.Count;
            _totalCutEdges   += cutEdgeCount;
            _dirty = true;
            result.splitVerts = splitMap.Count;
            result.cutEdges   = cutEdgeCount;

            sw.Stop();
            result.elapsedMs = (float)sw.Elapsed.TotalMilliseconds;

            Debug.Log($"[ConstraintCut] split:{splitMap.Count}v cut:{cutEdgeCount}e " +
                      $"tet:{affectedTets.Count} {result.elapsedMs:F1}ms " +
                      $"total V:{_data.NumParticles}");

            return result;
        }

        /// <summary>停止切割时重置法线</summary>
        public void ResetNormal() => _lockedNormal = Vector3.zero;

        // ══════════════════════════════════════════════════════
        // Step 2: 找受影响的 tet
        // ══════════════════════════════════════════════════════

        List<int> FindAffectedTets(Vector3 tipPos, Vector3 prevTipPos,
            float radius, Vector3 planeN, Vector3 planeP)
        {
            var result = new List<int>();
            float radiusSq = (radius * 3f) * (radius * 3f);
            int numT = _data.NumTets;

            for (int t = 0; t < numT; t++)
            {
                if (!_data.TetActive[t]) continue;
                if (result.Count >= MAX_AFFECTED_TETS) break;

                // 粗筛: 重心距离工具路径
                Vector3 center = _data.TetCenter(t);
                if (PointSegDistSq(center, prevTipPos, tipPos) > radiusSq)
                    continue;

                // 精筛: 切面穿过该 tet (有正有负)
                int b = t * 4;
                float d0 = SignedDist(_data.Positions[_data.TetIds[b  ]], planeP, planeN);
                float d1 = SignedDist(_data.Positions[_data.TetIds[b+1]], planeP, planeN);
                float d2 = SignedDist(_data.Positions[_data.TetIds[b+2]], planeP, planeN);
                float d3 = SignedDist(_data.Positions[_data.TetIds[b+3]], planeP, planeN);

                bool hasPos = d0 > 0 || d1 > 0 || d2 > 0 || d3 > 0;
                bool hasNeg = d0 < 0 || d1 < 0 || d2 < 0 || d3 < 0;
                if (hasPos && hasNeg)
                    result.Add(t);
            }
            return result;
        }

        // ══════════════════════════════════════════════════════
        // Step 3: 找需要分裂的顶点
        // ══════════════════════════════════════════════════════

        /// <summary>
        /// 返回 splitMap: originalVertexIndex → -1 (待创建副本)
        /// 条件: 顶点同时被切面正侧和负侧的 affected tet 引用
        /// </summary>
        Dictionary<int, int> FindVerticesNeedingSplit(
            List<int> affectedTets, Vector3 planeN, Vector3 planeP)
        {
            // 收集所有 affected tet 的顶点
            var vertPosTets = new Dictionary<int, int>(); // vertex → 正侧 tet 数
            var vertNegTets = new Dictionary<int, int>(); // vertex → 负侧 tet 数

            foreach (int t in affectedTets)
            {
                // 用 tet 的多数顶点判断侧面
                int tetSide = TetSide(t, planeP, planeN);

                for (int k = 0; k < 4; k++)
                {
                    int v = _data.TetIds[t * 4 + k];
                    if (tetSide > 0)
                    {
                        if (!vertPosTets.ContainsKey(v)) vertPosTets[v] = 0;
                        vertPosTets[v]++;
                    }
                    else
                    {
                        if (!vertNegTets.ContainsKey(v)) vertNegTets[v] = 0;
                        vertNegTets[v]++;
                    }
                }
            }

            // 需要分裂: 同时出现在正侧和负侧
            var splitMap = new Dictionary<int, int>();
            foreach (var kv in vertPosTets)
            {
                int v = kv.Key;
                if (vertNegTets.ContainsKey(v))
                {
                    // 该顶点被两侧的 affected tet 共享 → 需要分裂
                    splitMap[v] = -1; // -1 表示副本索引待创建
                }
            }

            return splitMap;
        }

        // ══════════════════════════════════════════════════════
        // Step 4: 创建副本顶点
        // ══════════════════════════════════════════════════════

        void CreateVertexCopies(Dictionary<int, int> splitMap)
        {
            // 确保 TetMeshData 有足够容量
            int needed = _data.NumParticles + splitMap.Count;
            _data.EnsureCapacity(needed + 100);
            EnsureAdjacencyCapacity(needed + 100);

            // 分离偏移量: 模拟刀片推开组织的物理效果
            // 太大 → 不自然; 太小 → 看不到切口
            // 使用平均边长的 5% 作为偏移
            float separationDist = EstimateMeanEdgeLength() * 0.05f;
            if (separationDist < 0.0005f) separationDist = 0.0005f;

            // 创建副本顶点
            var keys = new List<int>(splitMap.Keys);
            foreach (int origV in keys)
            {
                Vector3 pos = _data.Positions[origV];
                Vector3 vel = _data.Velocities[origV];
                float invMass = _data.InvMass[origV];

                // V' (副本) 向 -N 方向偏移
                Vector3 posNeg = pos - _lockedNormal * separationDist;
                int newV = _data.AddParticle(posNeg, vel, invMass);

                // 设置 rest position (与原始相同, 不偏移)
                if (newV < _data.RestPositions.Length)
                    _data.RestPositions[newV] = _data.RestPositions[origV];

                // 设置 prev position
                if (newV < _data.PrevPositions.Length)
                    _data.PrevPositions[newV] = posNeg;

                // 原始顶点 V 向 +N 方向偏移 (对称分离)
                if (_data.InvMass[origV] > 0f) // 不移动固定点
                {
                    _data.Positions[origV] = pos + _lockedNormal * separationDist;
                    _data.PrevPositions[origV] = _data.Positions[origV];
                }

                splitMap[origV] = newV;
            }
        }

        /// <summary>估算平均边长 (用于计算分离偏移)</summary>
        float EstimateMeanEdgeLength()
        {
            int[] eids = _solver.EdgeIds;
            int numE = Mathf.Min(_solver.NumEdges, 100); // 采样前100条边
            float totalLen = 0;
            int count = 0;
            for (int e = 0; e < numE; e++)
            {
                int a = eids[e * 2], b = eids[e * 2 + 1];
                if (a < _data.NumParticles && b < _data.NumParticles)
                {
                    totalLen += (_data.Positions[a] - _data.Positions[b]).magnitude;
                    count++;
                }
            }
            return count > 0 ? totalLen / count : 0.01f;
        }

        // ══════════════════════════════════════════════════════
        // Step 5: 更新 tet 索引
        // ══════════════════════════════════════════════════════

        void UpdateTetIndices(List<int> affectedTets,
            Dictionary<int, int> splitMap, Vector3 planeN, Vector3 planeP)
        {
            foreach (int t in affectedTets)
            {
                int tetSide = TetSide(t, planeP, planeN);
                if (tetSide >= 0) continue; // 正侧保持原始顶点

                // 负侧 tet: 将分裂顶点替换为副本
                for (int k = 0; k < 4; k++)
                {
                    int idx = t * 4 + k;
                    int v = _data.TetIds[idx];
                    if (splitMap.TryGetValue(v, out int newV))
                    {
                        _data.TetIds[idx] = newV;

                        // 更新邻接关系
                        _vertToTets[v].Remove(t);
                        if (newV < _vertToTets.Length)
                            _vertToTets[newV].Add(t);
                    }
                }
            }
        }

        // ══════════════════════════════════════════════════════
        // Step 6: 更新 edge 约束
        // ══════════════════════════════════════════════════════

        int UpdateEdgeConstraints(Dictionary<int, int> splitMap,
            Vector3 planeN, Vector3 planeP)
        {
            int[] eids = _solver.EdgeIds;
            bool[] eact = _solver.EdgeActive;
            int numE = _solver.NumEdges;
            var edgesToDisable = new HashSet<int>();

            // 收集所有涉及分裂顶点的边
            var affectedEdges = new HashSet<int>();
            foreach (var kv in splitMap)
            {
                int origV = kv.Key;
                if (origV < _vertToEdges.Length)
                {
                    foreach (int e in _vertToEdges[origV])
                        affectedEdges.Add(e);
                }
            }

            foreach (int e in affectedEdges)
            {
                if (!eact[e]) continue;
                int a = eids[e * 2], b = eids[e * 2 + 1];

                bool aSplit = splitMap.ContainsKey(a);
                bool bSplit = splitMap.ContainsKey(b);

                if (!aSplit && !bSplit) continue;

                // 判断两端各在哪侧
                float da = SignedDist(_data.Positions[a], planeP, planeN);
                float db = SignedDist(_data.Positions[b], planeP, planeN);
                int sideA = da >= 0 ? 1 : -1;
                int sideB = db >= 0 ? 1 : -1;

                if (sideA != sideB)
                {
                    // 边跨越切面 → 断开
                    edgesToDisable.Add(e);
                }
                else if (sideA < 0 && sideB < 0)
                {
                    // 两端都在负侧 → 更新索引为副本
                    if (aSplit) eids[e * 2]     = splitMap[a];
                    if (bSplit) eids[e * 2 + 1] = splitMap[b];
                }
                // 两端都在正侧 → 保持原始索引, 不做修改
            }

            // 断开跨切面的边
            if (edgesToDisable.Count > 0)
                _solver.DisableEdgesOnly(edgesToDisable);

            return edgesToDisable.Count;
        }

        // ══════════════════════════════════════════════════════
        // GPU 增量更新
        // ══════════════════════════════════════════════════════

        /// <summary>
        /// 将修改增量上传到 GPU
        /// 需要 full reinit (因为顶点数变了, buffer 大小变了)
        /// 但保留物理状态 (位置/速度不重置)
        /// </summary>
        public void FlushToGPU()
        {
            if (!_dirty) return;

            // ★ 关键: 在 Dispose 前回读全部物理状态 (位置+速度+prevPos)
            // 否则 Init 后 GPU 速度会被清零, 导致仿真冻结
            _solver.ReadbackAll(_data);

            // 由于顶点数增加, 需要 solver reinit
            _solver.Dispose();
            _solver.Init(_data);

            _dirty = false;

            // 重建邻接关系
            BuildAdjacency();
        }

        // ══════════════════════════════════════════════════════
        // 工具方法
        // ══════════════════════════════════════════════════════

        static float SignedDist(Vector3 p, Vector3 planeP, Vector3 planeN)
        {
            return Vector3.Dot(p - planeP, planeN);
        }

        /// <summary>用正侧顶点数量判断 tet 在切面哪一侧</summary>
        int TetSide(int t, Vector3 planeP, Vector3 planeN)
        {
            int posCount = 0;
            for (int k = 0; k < 4; k++)
            {
                int v = _data.TetIds[t * 4 + k];
                if (SignedDist(_data.Positions[v], planeP, planeN) >= 0)
                    posCount++;
            }
            return posCount >= 2 ? 1 : -1; // 多数决定
        }

        static float PointSegDistSq(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Vector3.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-10f);
            t = Mathf.Clamp01(t);
            return (p - (a + ab * t)).sqrMagnitude;
        }
    }
}
