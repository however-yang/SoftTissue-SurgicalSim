// TetRemovalCutter.cs — 删除四面体 + 双面Cap切缘重建
//
// 算法:
//   1. 找到被切割面穿过的 tet → 标记 TetActive = false
//   2. 计算切面与 tet 边的交点 → 生成 Cap 三角形
//   3. 正/负两侧各生成一份 Cap (初始重合 → 物理拉开 → 近零宽度切口)
//   4. 只更新 GPU TetActive buffer — 不做 Dispose+Init
//
// 优势:
//   - tet 只减不增, buffer 大小不变
//   - 不需要 solver reinit
//   - Cap 面在同一切割面上 → 自动共面 → 平滑无锯齿

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Cutting
{
    public class TetRemovalCutter
    {
        TetMeshData   _data;
        XPBDSolverGPU _solver;

        int _totalRemovedTets;
        bool _dirty; // 有新的 tet 被删除, 需要更新 GPU

        // 锁定法线: 一次切割动作内保持同一法线
        Vector3 _lockedNormal = Vector3.zero;

        // Cap 面数据 (纯渲染用)
        List<Vector3> _capVertices = new List<Vector3>();
        List<int> _capTriangles = new List<int>();
        List<Vector3> _capNormals = new List<Vector3>();

        // 持久边缓存: 相邻 tet 的切面交点共享
        Dictionary<long, Vector3> _edgeCutPointCache = new Dictionary<long, Vector3>();

        // 参数
        const int MAX_TETS_PER_CUT = 30;
        const float MIN_MOVE_DIST = 0.0001f;

        // ── 公开属性 ─────────────────────────────────────────

        public int TotalRemovedTets => _totalRemovedTets;
        public bool IsDirty => _dirty;
        public List<Vector3> CapVertices => _capVertices;
        public List<int> CapTriangles => _capTriangles;
        public List<Vector3> CapNormals => _capNormals;

        // ── 初始化 ──────────────────────────────────────────

        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data = data;
            _solver = solver;
            _totalRemovedTets = 0;
            _dirty = false;
            _lockedNormal = Vector3.zero;
            _capVertices.Clear();
            _capTriangles.Clear();
            _capNormals.Clear();
            _edgeCutPointCache.Clear();
        }

        // ── 主入口 ──────────────────────────────────────────

        public struct CutResult
        {
            public int removedTets;
            public int capTriangles;
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

            // ── 切割面 (锁定法线) ────────────────────────────
            Vector3 rawNormal = Vector3.Cross(toolDir, moveDir).normalized;
            if (rawNormal.sqrMagnitude < 0.01f)
                rawNormal = Vector3.Cross(toolDir, Vector3.up).normalized;

            if (_lockedNormal.sqrMagnitude < 0.01f)
                _lockedNormal = rawNormal;

            Vector3 planeN = _lockedNormal;
            Vector3 planeP = (tipPos + prevTipPos) * 0.5f;

            // ── 遍历 tet, 删除被穿过的 ───────────────────────
            int numT = _data.NumTets;
            int removed = 0;

            for (int t = 0; t < numT; t++)
            {
                if (!_data.TetActive[t]) continue;
                if (removed >= MAX_TETS_PER_CUT) break;

                // 粗筛: 重心距离
                Vector3 center = _data.TetCenter(t);
                if (PointSegDist(center, prevTipPos, tipPos) > toolRadius * 3f)
                    continue;

                // 精筛: 签名距离分类
                int b4 = t * 4;
                int v0 = _data.TetIds[b4], v1 = _data.TetIds[b4+1];
                int v2 = _data.TetIds[b4+2], v3 = _data.TetIds[b4+3];

                float d0 = Vector3.Dot(_data.Positions[v0] - planeP, planeN);
                float d1 = Vector3.Dot(_data.Positions[v1] - planeP, planeN);
                float d2 = Vector3.Dot(_data.Positions[v2] - planeP, planeN);
                float d3 = Vector3.Dot(_data.Positions[v3] - planeP, planeN);

                bool hasPos = (d0 > 0) || (d1 > 0) || (d2 > 0) || (d3 > 0);
                bool hasNeg = (d0 < 0) || (d1 < 0) || (d2 < 0) || (d3 < 0);
                if (!hasPos || !hasNeg) continue;

                // ── 删除这个 tet ─────────────────────────────
                _data.DeactivateTet(t);
                removed++;

                // ── 生成 Cap (双面) ──────────────────────────
                int[] vi = { v0, v1, v2, v3 };
                float[] sd = { d0, d1, d2, d3 };
                GenerateCapForTet(vi, sd, planeP, planeN);
            }

            if (removed > 0)
            {
                _totalRemovedTets += removed;
                _dirty = true;
                result.removedTets = removed;
                result.capTriangles = _capTriangles.Count / 3;
            }

            sw.Stop();
            result.elapsedMs = (float)sw.Elapsed.TotalMilliseconds;

            if (removed > 0)
                Debug.Log($"[TetRemovalCut] -{removed}tet " +
                          $"cap:{result.capTriangles}tri " +
                          $"{result.elapsedMs:F1}ms T:{_data.NumTets}");

            return result;
        }

        /// <summary>停止切割时重置法线</summary>
        public void ResetNormal()
        {
            _lockedNormal = Vector3.zero;
        }

        /// <summary>增量更新 GPU TetActive buffer (不做 full reinit)</summary>
        public void FlushToGPU()
        {
            if (!_dirty || _solver == null) return;
            _solver.UpdateTetActive(_data);
            _dirty = false;
        }

        // ══════════════════════════════════════════════════════
        // Cap 生成
        // ══════════════════════════════════════════════════════

        void GenerateCapForTet(int[] vi, float[] sd,
            Vector3 planeP, Vector3 planeN)
        {
            // 找到被切面穿过的边, 计算交点
            var cutPoints = new List<Vector3>();
            int[][] edges = {
                new[]{0,1}, new[]{0,2}, new[]{0,3},
                new[]{1,2}, new[]{1,3}, new[]{2,3}
            };

            foreach (var e in edges)
            {
                int a = e[0], b = e[1];
                float da = sd[a], db = sd[b];

                // 只处理穿过切面的边
                if ((da > 0 && db > 0) || (da < 0 && db < 0) ||
                    (da == 0 && db == 0)) continue;

                Vector3 pa = _data.Positions[vi[a]];
                Vector3 pb = _data.Positions[vi[b]];

                // 缓存: 同一条边只算一次
                long key = EdgeKey(vi[a], vi[b]);
                if (!_edgeCutPointCache.TryGetValue(key, out Vector3 cp))
                {
                    float absA = Mathf.Abs(da), absB = Mathf.Abs(db);
                    float t = absA / (absA + absB + 1e-10f);
                    cp = Vector3.Lerp(pa, pb, t);
                    _edgeCutPointCache[key] = cp;
                }
                cutPoints.Add(cp);
            }

            if (cutPoints.Count < 3) return;

            // 投影到切面 (确保完全共面 → 平滑)
            for (int i = 0; i < cutPoints.Count; i++)
            {
                float dist = Vector3.Dot(cutPoints[i] - planeP, planeN);
                cutPoints[i] -= planeN * dist;
            }

            // 排序: 按极角排列 (确保正确的多边形顺序)
            Vector3 centroid = Vector3.zero;
            foreach (var p in cutPoints) centroid += p;
            centroid /= cutPoints.Count;

            Vector3 refDir = (cutPoints[0] - centroid).normalized;
            Vector3 biDir = Vector3.Cross(planeN, refDir).normalized;

            cutPoints.Sort((a, b) =>
            {
                float angleA = Mathf.Atan2(
                    Vector3.Dot(a - centroid, biDir),
                    Vector3.Dot(a - centroid, refDir));
                float angleB = Mathf.Atan2(
                    Vector3.Dot(b - centroid, biDir),
                    Vector3.Dot(b - centroid, refDir));
                return angleA.CompareTo(angleB);
            });

            // Fan 三角化: centroid → 多边形扇形分解
            // 正面 Cap (法线 = +planeN)
            int baseIdx = _capVertices.Count;
            _capVertices.Add(centroid);
            _capNormals.Add(planeN);
            for (int i = 0; i < cutPoints.Count; i++)
            {
                _capVertices.Add(cutPoints[i]);
                _capNormals.Add(planeN);
            }
            for (int i = 0; i < cutPoints.Count; i++)
            {
                int next = (i + 1) % cutPoints.Count;
                _capTriangles.Add(baseIdx);          // centroid
                _capTriangles.Add(baseIdx + 1 + i);  // current
                _capTriangles.Add(baseIdx + 1 + next); // next
            }

            // 背面 Cap (法线 = -planeN, 反转缠绕)
            baseIdx = _capVertices.Count;
            _capVertices.Add(centroid);
            _capNormals.Add(-planeN);
            for (int i = 0; i < cutPoints.Count; i++)
            {
                _capVertices.Add(cutPoints[i]);
                _capNormals.Add(-planeN);
            }
            for (int i = 0; i < cutPoints.Count; i++)
            {
                int next = (i + 1) % cutPoints.Count;
                _capTriangles.Add(baseIdx);          // centroid
                _capTriangles.Add(baseIdx + 1 + next); // next (反转)
                _capTriangles.Add(baseIdx + 1 + i);  // current (反转)
            }
        }

        // ══════════════════════════════════════════════════════
        // 工具
        // ══════════════════════════════════════════════════════

        static long EdgeKey(int a, int b)
        {
            if (a > b) { int t = a; a = b; b = t; }
            return (long)a * 200000L + b;
        }

        static float PointSegDist(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float t = Vector3.Dot(p - a, ab) / Mathf.Max(ab.sqrMagnitude, 1e-10f);
            t = Mathf.Clamp01(t);
            return (p - (a + ab * t)).magnitude;
        }
    }
}
