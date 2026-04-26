// VertexSplitCutter.cs — 四面体细分切割
// 论文: "Parallel Constraint Graph Partitioning and Coloring
//        for Realtime Soft-Body Cutting"  Peng Yu et al., PG 2025
//
// 算法:
//  1. 锁定切面法线 (第一帧确定, 后续帧保持 → 平滑切口)
//  2. 1-3 / 2-2 细分 (论文 Fig.5)
//  3. 正侧/负侧分离: 用切面签名距离分 2 组 (不用 Union-Find)

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Cutting
{
    public class VertexSplitCutter
    {
        TetMeshData   _data;
        XPBDSolverGPU _solver;
        ComputeShader _computeShader;

        int _totalSplitVerts, _totalCutEdges;
        bool _needReinit;

        // 持久边缓存: 连续切割共享切面顶点
        Dictionary<long, int> _edgeCutVertCache = new Dictionary<long, int>();

        // 节流
        int _framesSinceLastReinit = 999;
        const int REINIT_COOLDOWN_FRAMES = 2;

        // 参数
        const float SNAP_EPSILON = 0.1f;
        const int MAX_TETS_PER_CUT = 25;
        const int MAX_TOTAL_TETS = 30000;

        // 锁定法线: 一次切割动作内保持同一法线 → 平滑切口
        Vector3 _lockedNormal = Vector3.zero;
        bool _wascutting = false;

        public struct CutResult
        {
            public int newSplitVerts, newCutEdges;
            public float elapsedMs;
            public bool needsReinit;
        }

        public int TotalSplitVerts => _totalSplitVerts;
        public int TotalCutEdges  => _totalCutEdges;
        public bool NeedsReinit   => _needReinit;

        public void Init(TetMeshData data, XPBDSolverGPU solver, ComputeShader cs)
        {
            _data = data; _solver = solver; _computeShader = cs;
            _edgeCutVertCache.Clear();
            _totalSplitVerts = _totalCutEdges = 0;
            _needReinit = false;
            _framesSinceLastReinit = 999;
            _lockedNormal = Vector3.zero;
            _wascutting = false;
            _data.EnsureTetCapacity(_data.NumTets * 3);
        }

        // ══════════════════════════════════════════════════════
        // 主入口
        // ══════════════════════════════════════════════════════
        public CutResult Cut(Vector3 tipPos, Vector3 prevTipPos,
                             Vector3 toolDir, float toolRadius)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var result = new CutResult();
            _framesSinceLastReinit++;

            Vector3 moveDir = tipPos - prevTipPos;
            float moveDist = moveDir.magnitude;
            if (moveDist < 0.0001f) return result;
            moveDir /= moveDist;

            // ── 切面法线: 锁定在切割开始时 ───────────────────
            Vector3 rawNormal = Vector3.Cross(toolDir, moveDir).normalized;
            if (rawNormal.sqrMagnitude < 0.01f)
                rawNormal = Vector3.Cross(toolDir, Vector3.up).normalized;

            if (_lockedNormal.sqrMagnitude < 0.01f)
                _lockedNormal = rawNormal;
            // 切割过程中保持锁定; 停止切割后重置 (由外部调 ResetNormal)

            Vector3 planeNormal = _lockedNormal;
            Vector3 planePoint  = (tipPos + prevTipPos) * 0.5f;

            if (_framesSinceLastReinit < REINIT_COOLDOWN_FRAMES) return result;
            if (_data.NumTets > MAX_TOTAL_TETS) return result;

            // ── 遍历 tet 细分 ────────────────────────────────
            int numTetsBefore = _data.NumTets;
            int numVertsBefore = _data.NumParticles;
            var cutVerts   = new HashSet<int>();
            var newTetList = new List<int>();
            var removedTets = new List<int>();

            for (int t = 0; t < numTetsBefore; t++)
            {
                if (!_data.TetActive[t]) continue;
                if (removedTets.Count >= MAX_TETS_PER_CUT) break;

                Vector3 center = _data.TetCenter(t);
                if (PointSegDist(center, prevTipPos, tipPos) > toolRadius * 3f)
                    continue;

                int b4 = t * 4;
                int[] vi = { _data.TetIds[b4], _data.TetIds[b4+1],
                             _data.TetIds[b4+2], _data.TetIds[b4+3] };
                float[] sd = new float[4];
                for (int j = 0; j < 4; j++)
                    sd[j] = Vector3.Dot(_data.Positions[vi[j]] - planePoint, planeNormal);

                var pos = new List<int>(); var neg = new List<int>();
                for (int j = 0; j < 4; j++)
                {
                    if (sd[j] >= 0) pos.Add(j); else neg.Add(j);
                }
                if (pos.Count == 0 || neg.Count == 0) continue;

                var newT = new List<int>();
                var cv = new HashSet<int>();

                if (pos.Count == 1 && neg.Count == 3)
                    Sub13(vi, sd, pos[0], neg, newT, cv);
                else if (pos.Count == 3 && neg.Count == 1)
                    Sub13(vi, sd, neg[0], pos, newT, cv);
                else if (pos.Count == 2 && neg.Count == 2)
                    Sub22(vi, sd, pos, neg, newT, cv);

                if (newT.Count > 0)
                {
                    _data.DeactivateTet(t);
                    removedTets.Add(t);
                    newTetList.AddRange(newT);
                    foreach (int c in cv) cutVerts.Add(c);
                }
            }

            if (removedTets.Count == 0) return result;

            // ── 分离: 正/负侧 2 组 (简单签名距离) ────────────
            int splitCount = SeparateBySide(
                cutVerts, newTetList, planePoint, planeNormal);

            result.newSplitVerts = _data.NumParticles - numVertsBefore;
            result.newCutEdges = removedTets.Count;
            _totalSplitVerts += result.newSplitVerts;
            _totalCutEdges += result.newCutEdges;
            _needReinit = true;
            result.needsReinit = true;
            _framesSinceLastReinit = 0;

            sw.Stop();
            result.elapsedMs = (float)sw.Elapsed.TotalMilliseconds;
            Debug.Log($"[Cut] {removedTets.Count}→{newTetList.Count}tet " +
                      $"+{result.newSplitVerts}v {splitCount}sep " +
                      $"{result.elapsedMs:F1}ms T:{_data.NumTets}");
            return result;
        }

        /// <summary>停止切割时调用, 重置锁定法线</summary>
        public void ResetNormal() { _lockedNormal = Vector3.zero; }

        public void SafeReinit()
        {
            if (!_needReinit) return;
            int n = _data.NumParticles;
            var sP = new Vector3[n]; var sV = new Vector3[n]; var sPr = new Vector3[n];
            System.Array.Copy(_data.Positions, sP, n);
            System.Array.Copy(_data.Velocities, sV, n);
            System.Array.Copy(_data.PrevPositions, sPr, n);
            _solver.Dispose();
            _solver.Init(_data);
            int c = Mathf.Min(n, _data.NumParticles);
            System.Array.Copy(sP, _data.Positions, c);
            System.Array.Copy(sV, _data.Velocities, c);
            System.Array.Copy(sPr, _data.PrevPositions, c);
            _solver.UploadPositionsAndVelocities(_data);
            _needReinit = false;
        }

        // ══════════════════════════════════════════════════════
        // 1-3 细分 (论文 Fig.5a)
        //  A 孤立, B/C/D 另侧
        //  Cap:   (A, P0, P1, P2)
        //  Prism: (P0, B, C, D), (P0, P1, C, D), (P0, P1, P2, D)
        //    — 经典 fan-from-P0-through-D 分解三棱柱
        // ══════════════════════════════════════════════════════
        void Sub13(int[] vi, float[] sd, int aIdx, List<int> bIdx,
            List<int> outT, HashSet<int> outCv)
        {
            int A = vi[aIdx];
            int B = vi[bIdx[0]], C = vi[bIdx[1]], D = vi[bIdx[2]];
            float dA = sd[aIdx];

            int P0 = CutVert(A, B, dA, sd[bIdx[0]]);
            int P1 = CutVert(A, C, dA, sd[bIdx[1]]);
            int P2 = CutVert(A, D, dA, sd[bIdx[2]]);
            outCv.Add(P0); outCv.Add(P1); outCv.Add(P2);

            // cap (A 侧)
            TryAdd(outT, A, P0, P1, P2);
            // prism (B 侧) — fan from P0 through D
            TryAdd(outT, P0, B,  C,  D);
            TryAdd(outT, P0, P1, C,  D);
            TryAdd(outT, P0, P1, P2, D);
        }

        // ══════════════════════════════════════════════════════
        // 2-2 细分 (论文 Fig.5b)
        //  {A,D} 正, {B,C} 负
        //  E1=A-B, F1=A-C, G1=D-B, H1=D-C, 对角线 F1-H1
        //  Side1: (A,E1,F1,H1), (A,D,F1,H1), (D,F1,G1,H1)
        //  Side2: (B,E1,F1,H1), (B,C,F1,H1), (C,F1,G1,H1)
        // ══════════════════════════════════════════════════════
        void Sub22(int[] vi, float[] sd, List<int> pIdx, List<int> nIdx,
            List<int> outT, HashSet<int> outCv)
        {
            int A = vi[pIdx[0]], D = vi[pIdx[1]];
            int B = vi[nIdx[0]], C = vi[nIdx[1]];
            float dA = sd[pIdx[0]], dD = sd[pIdx[1]];
            float dB = sd[nIdx[0]], dC = sd[nIdx[1]];

            int E1 = CutVert(A, B, dA, dB);
            int F1 = CutVert(A, C, dA, dC);
            int G1 = CutVert(D, B, dD, dB);
            int H1 = CutVert(D, C, dD, dC);
            outCv.Add(E1); outCv.Add(F1); outCv.Add(G1); outCv.Add(H1);

            // Side 1 (A,D)
            TryAdd(outT, A, E1, F1, H1);
            TryAdd(outT, A, D,  F1, H1);
            TryAdd(outT, D, F1, G1, H1);
            // Side 2 (B,C)
            TryAdd(outT, B, E1, F1, H1);
            TryAdd(outT, B, C,  F1, H1);
            TryAdd(outT, C, F1, G1, H1);
        }

        // ══════════════════════════════════════════════════════
        // 分离: 正侧/负侧 各一份切面顶点
        //
        // 对每个 cutVert:
        //  1. 找所有引用它的新 tet
        //  2. 用每个 tet 中原始顶点的平均签名距离判断正/负侧
        //  3. 负侧统一用 cutVert 的副本
        // ══════════════════════════════════════════════════════
        int SeparateBySide(HashSet<int> cutVerts, List<int> newTets,
            Vector3 planePoint, Vector3 planeNormal)
        {
            int splits = 0;
            foreach (int cv in cutVerts)
            {
                // 收集引用此顶点的 tet, 分正/负
                var posTets = new List<int>();
                var negTets = new List<int>();

                foreach (int t in newTets)
                {
                    if (!_data.TetActive[t]) continue;
                    int b4 = t * 4;
                    bool refs = false;
                    for (int j = 0; j < 4; j++)
                        if (_data.TetIds[b4+j] == cv) { refs = true; break; }
                    if (!refs) continue;

                    // 用该 tet 中非切面顶点的签名距离来判断侧
                    float sum = 0; int cnt = 0;
                    for (int j = 0; j < 4; j++)
                    {
                        int vid = _data.TetIds[b4+j];
                        if (!cutVerts.Contains(vid))
                        {
                            sum += Vector3.Dot(
                                _data.Positions[vid] - planePoint, planeNormal);
                            cnt++;
                        }
                    }
                    if (cnt == 0) continue;
                    if (sum >= 0) posTets.Add(t);
                    else          negTets.Add(t);
                }

                // 只有两侧都有 tet 才分离
                if (posTets.Count == 0 || negTets.Count == 0) continue;

                // 创建副本给负侧
                int newV = _data.AddParticle(
                    _data.Positions[cv], _data.Velocities[cv], _data.InvMass[cv]);
                if (newV < 0) break;
                _data.RestPositions[newV] = _data.RestPositions[cv];
                _data.PrevPositions[newV] = _data.PrevPositions[cv];

                foreach (int t in negTets)
                {
                    int b4 = t * 4;
                    for (int j = 0; j < 4; j++)
                        if (_data.TetIds[b4+j] == cv)
                        { _data.TetIds[b4+j] = newV; break; }
                }
                splits++;
            }
            return splits;
        }

        // ══════════════════════════════════════════════════════
        // 工具方法
        // ══════════════════════════════════════════════════════
        int CutVert(int va, int vb, float da, float db)
        {
            long key = EdgeKey(va, vb);
            if (_edgeCutVertCache.TryGetValue(key, out int c)) return c;

            float absA = Mathf.Abs(da), absB = Mathf.Abs(db);
            float t = absA / (absA + absB);
            int res;
            if (t < SNAP_EPSILON) res = va;
            else if (t > 1f - SNAP_EPSILON) res = vb;
            else
            {
                Vector3 p = Vector3.Lerp(_data.Positions[va], _data.Positions[vb], t);
                Vector3 r = Vector3.Lerp(_data.RestPositions[va], _data.RestPositions[vb], t);
                Vector3 pr = Vector3.Lerp(_data.PrevPositions[va], _data.PrevPositions[vb], t);
                Vector3 v = Vector3.Lerp(_data.Velocities[va], _data.Velocities[vb], t);
                float im = Mathf.Lerp(_data.InvMass[va], _data.InvMass[vb], t);
                res = _data.AddParticle(p, v, im);
                _data.RestPositions[res] = r;
                _data.PrevPositions[res] = pr;
            }
            _edgeCutVertCache[key] = res;
            return res;
        }

        int AddSafe(int a, int b, int c, int d)
        {
            Vector3 pa = _data.Positions[a], pb = _data.Positions[b];
            Vector3 pc = _data.Positions[c], pd = _data.Positions[d];
            float vol = Vector3.Dot(Vector3.Cross(pb-pa, pc-pa), pd-pa);
            if (Mathf.Abs(vol) < 1e-15f) return -1;
            return vol < 0 ? _data.AddTet(a,b,d,c) : _data.AddTet(a,b,c,d);
        }

        void TryAdd(List<int> list, int a, int b, int c, int d)
        {
            int idx = AddSafe(a,b,c,d);
            if (idx >= 0) list.Add(idx);
        }

        static long EdgeKey(int a, int b)
        {
            if (a > b) { int tmp = a; a = b; b = tmp; }
            return (long)a * 200000L + b;
        }

        static float PointSegDist(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Vector3.Dot(p-a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-10f);
            t = Mathf.Clamp01(t);
            return (p - (a + ab*t)).magnitude;
        }
    }
}
