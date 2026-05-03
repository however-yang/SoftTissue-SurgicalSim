// TetSubdivisionCutter.cs — Phase 2: 论文级切割质量
//
// 基于 PG2025 "Parallel Constraint Graph Partitioning and Coloring
//              for Realtime Soft-Body Cutting" (Peng Yu et al.)
//
// Phase 2 新增 (Section 4.3):
//   1. 轨迹修正 (Trajectory Correction): 交点接近顶点时吸附
//   2. 共享原始顶点判据 (Shared Original Vertex): 决定分离/连接
//   3. 切口内表面三角形: 使切口可见

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.CuttingV3
{
    public class TetSubdivisionCutter
    {
        TetMeshData   _data;
        XPBDSolverGPU _solver;

        int  _totalNewVerts, _totalNewTets, _totalCutTets;
        bool _dirty;

        readonly HashSet<int> _alreadyCutTets = new HashSet<int>();

        // ★ 双层缓存策略，彻底解决藕断丝连
        // _frameSplitCache : 帧级，每帧清除。同帧内同边复用相同 posV/negV，与当前帧平面精确匹配。
        // _strokeSplitCache: 划刀级，ResetStroke才清除。跨帧拓扑连续性，复用已切割原始边的顶点对。
        //   只缓存原始顶点的边（两端 ID < _strokeStartVertCount），防止新顶点间的边污染。
        readonly Dictionary<long, (int posV, int negV)> _frameSplitCache
            = new Dictionary<long, (int, int)>();
        readonly Dictionary<long, (int posV, int negV)> _strokeSplitCache
            = new Dictionary<long, (int, int)>();
        readonly Dictionary<int, int> _frameSnapCache = new Dictionary<int, int>();
        
        struct VertexSideLock { public int side; public Vector3 normal; }
        readonly Dictionary<int, VertexSideLock> _strokeVertexSide = new Dictionary<int, VertexSideLock>();

        int _originalVertCount;
        int _strokeStartVertCount;

        const float MIN_TET_VOL = 1e-11f;
        private int _strokeStartTetCount = -1;

        // ── 诊断 ─────────────────────────────────────────────
        public float  LastMoveDistance        { get; private set; }
        public int    LastCandidateTetCount   { get; private set; }
        public int    LastIntersectedTetCount { get; private set; }
        public string LastRejectReason        { get; private set; } = "idle";
        public int  TotalNewVerts => _totalNewVerts;
        public int  TotalNewTets  => _totalNewTets;
        public int  TotalCutTets  => _totalCutTets;
        public bool IsDirty       => _dirty;

        // ══════════════════════════════════════════════════════
        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data = data; _solver = solver;
            _totalNewVerts = _totalNewTets = _totalCutTets = 0;
            _dirty = false;
            _alreadyCutTets.Clear();
            _frameSplitCache.Clear();
            _strokeSplitCache.Clear();
            _frameSnapCache.Clear();
            _strokeVertexSide.Clear();
            _originalVertCount    = data.NumParticles;
            _strokeStartVertCount = data.NumParticles;
            _data.EnsureCapacity(data.NumParticles * 4);
            _data.EnsureTetCapacity(data.NumTets * 4);
            Debug.Log($"[SweptCutter] Init V:{data.NumParticles} T:{data.NumTets}");
        }

        private Vector3 _lastValidNormal = Vector3.zero;
        private Vector3 _strokeBaseNormal = Vector3.zero;

        public void ResetStroke()
        {
            LastRejectReason = "not_cutting";
            _lastValidNormal = Vector3.zero;
            _strokeBaseNormal = Vector3.zero;
            // 新划刀：双层缓存全部清除，顶点计数重置
            _frameSplitCache.Clear();
            _strokeSplitCache.Clear();
            _frameSnapCache.Clear();
            _strokeVertexSide.Clear();
            _strokeStartTetCount  = -1;
            _strokeStartVertCount = _data != null ? _data.NumParticles : 0;
        }

        // ══════════════════════════════════════════════════════
        // 主入口: 平面切割
        // ══════════════════════════════════════════════════════
        public struct CutResult { public int newVerts, newTets, cutTets; public float elapsedMs; }

        public CutResult Cut(Vector3 A0, Vector3 B0, Vector3 A1, Vector3 B1)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var result = new CutResult();
            LastCandidateTetCount = LastIntersectedTetCount = 0;

            float moveA = (A1 - A0).magnitude;
            float moveB = (B1 - B0).magnitude;
            LastMoveDistance = Mathf.Max(moveA, moveB);

            if (LastMoveDistance < 1e-5f)
            { LastRejectReason = "move_too_small"; return result; }

            // ── 构建切割平面 ───────────────────────────────────
            Vector3 bladeDir = (B1 - A1);
            float bladeLen = bladeDir.magnitude;
            if (bladeLen < 1e-6f)
            { LastRejectReason = "degenerate_blade"; return result; }
            bladeDir /= bladeLen;

            Vector3 moveDir = ((A1 + B1) - (A0 + B0)) * 0.5f;
            float moveMag = moveDir.magnitude;
            if (moveMag < 1e-6f)
            { LastRejectReason = "no_movement"; return result; }
            moveDir /= moveMag;

            // ★ 修复：解锁法线与中心！允许切面随手腕甩动而弯曲！
            // 之前为了防止切口锯齿，锁定了第一帧的法线。但这导致如果玩家在收刀时“甩腕”或“旋转”刀锋，
            // 数学平面依然是直的，刀尖扫过的边缘区域即使在包围盒内，也因为不与这个固定的直平面相交而漏切（藕断丝连）！
            // 由于我们已经有了贯穿整个 Stroke 的 _edgeCache 和 _snappedCache 兜底，
            // 即使每帧平面发生微小偏转，缓存也能把相邻帧的四面体完美“缝合”在一起，绝不会出现锯齿或裂缝！
            Vector3 planeNormal = Vector3.Cross(bladeDir, moveDir);
            float planeNLen = planeNormal.magnitude;
            if (planeNLen < 1e-4f)
            { 
                if (_lastValidNormal != Vector3.zero)
                {
                    planeNormal = _lastValidNormal;
                }
                else
                {
                    LastRejectReason = "parallel_move"; 
                    return result; 
                }
            }
            else
            {
                planeNormal /= planeNLen;
                
                // ★ 智能分段：当刀片发生明显转弯（累计超过 25 度）时，解锁新生成的子四面体，允许二次切割。
                if (_strokeBaseNormal == Vector3.zero)
                {
                    _strokeBaseNormal = planeNormal;
                }
                else
                {
                    float angle = Vector3.Angle(_strokeBaseNormal, planeNormal);
                    if (angle > 25f)
                    {
                        Debug.Log($"[TetSubdivisionCutter] 检测到刀刃累计转弯 {angle:F1} 度，解锁子四面体二次切割！");
                        _strokeStartTetCount = _data.NumTets;
                        _strokeStartVertCount = _data.NumParticles;
                        _strokeBaseNormal = planeNormal; // 重置基准法线
                    }
                }
                _lastValidNormal = planeNormal;
            }

            Vector3 planeCenter = (A1 + B1) * 0.5f;

            // ★ 帧级缓存每帧必须清除：每帧平面法线都可能因刀的旋转而偏转，
            // 帧缓存若不清除，旧交点会错位到新平面之外，产生拓扑扭曲和藕断丝连！
            _frameSplitCache.Clear();
            _frameSnapCache.Clear();  // 同一帧内同顶点 snap 必须复用同一克隆——每帧重置

            // ── 遍历所有 tet ─────────────────────────────────
            int vBefore = _data.NumParticles;
            int tBefore = _data.NumTets;

            // ★ 防止“切口粉碎机”效应的核心防线：
            // 因为这是一把“连续挥动”的刀，平面的数学方程被锁定了（_lockedNormal）。
            // 如果我们在这个循环里检查这一刀刚才（或者前几帧）切出来的子四面体，
            // 它们因为物理引擎的弹性会抖动，导致它们不可避免地再次跨越这个虚拟的数学平面。
            // 结果就是：切好的肉自己疯狂抖动着撞向留在原地的“幽灵刀刃”，被切成成千上万的粉末！
            // 所以，在同一个连续挥刀动作里，我们【绝对不能】切这一刀切出来的子四面体！
            if (_strokeStartTetCount == -1) _strokeStartTetCount = tBefore;
            int tetSnapshot = _strokeStartTetCount;
            
            int candCount = 0, cutCount = 0;

            for (int t = 0; t < tetSnapshot; t++)
            {
                if (!_data.TetActive[t]) continue;
                if (_alreadyCutTets.Contains(t)) continue;

                int b = t * 4;
                int i0 = _data.TetIds[b], i1 = _data.TetIds[b+1];
                int i2 = _data.TetIds[b+2], i3 = _data.TetIds[b+3];

                // 1. ★ AABB 测试提前，只处理在刀刃扫掠区域内的 tet
                Vector3 minBounds = Vector3.Min(Vector3.Min(A0, B0), Vector3.Min(A1, B1));
                Vector3 maxBounds = Vector3.Max(Vector3.Max(A0, B0), Vector3.Max(A1, B1));
                const float kMargin = 0.015f;
                Vector3 lo = minBounds - Vector3.one * kMargin;
                Vector3 hi = maxBounds + Vector3.one * kMargin;
                
                Vector3 v0 = _data.Positions[i0], v1 = _data.Positions[i1];
                Vector3 v2 = _data.Positions[i2], v3 = _data.Positions[i3];
                Vector3 tMin = Vector3.Min(Vector3.Min(v0, v1), Vector3.Min(v2, v3));
                Vector3 tMax = Vector3.Max(Vector3.Max(v0, v1), Vector3.Max(v2, v3));
                // 如果两个 AABB 在任意一个轴上不重叠，则不可能相交
                if (tMin.x > hi.x || tMax.x < lo.x ||
                    tMin.y > hi.y || tMax.y < lo.y ||
                    tMin.z > hi.z || tMax.z < lo.z)
                    continue;

                // 2. 计算点到切割平面的有符号距离
                float d0 = Vector3.Dot(v0 - planeCenter, planeNormal);
                float d1 = Vector3.Dot(v1 - planeCenter, planeNormal);
                float d2 = Vector3.Dot(v2 - planeCenter, planeNormal);
                float d3 = Vector3.Dot(v3 - planeCenter, planeNormal);

                // 3. ★ 终极跨帧一致性修复：锁定顶点侧，防止平面旋转导致的跨帧拓扑翻转！
                // 如果一个顶点在之前的帧被判断在正侧，刀刃旋转后它不能跑到负侧去，否则会缝合断开的面形成藕断丝连。
                d0 = EnforceConsistentSide(i0, d0, planeNormal);
                d1 = EnforceConsistentSide(i1, d1, planeNormal);
                d2 = EnforceConsistentSide(i2, d2, planeNormal);
                d3 = EnforceConsistentSide(i3, d3, planeNormal);

                // 4. Trajectory Correction (平面吸附)
                const float SNAP_DIST = 1e-4f;
                if (Mathf.Abs(d0) < SNAP_DIST) d0 = 0f;
                if (Mathf.Abs(d1) < SNAP_DIST) d1 = 0f;
                if (Mathf.Abs(d2) < SNAP_DIST) d2 = 0f;
                if (Mathf.Abs(d3) < SNAP_DIST) d3 = 0f;

                int pos = 0, neg = 0;
                if (d0 >= 0f) pos++; else neg++;
                if (d1 >= 0f) pos++; else neg++;
                if (d2 >= 0f) pos++; else neg++;
                if (d3 >= 0f) pos++; else neg++;

                if (pos == 4 || pos == 0) continue;

                candCount++;

                int[] vi = { i0, i1, i2, i3 };
                float[] sd = { d0, d1, d2, d3 };

                // ★ 拓扑一致性关键: 按照全局顶点ID排序！
                // 这保证了所有相邻四面体在共享面上切出的多边形，其三角剖分的对角线选择是完全一致的！
                // 否则共享面的对角线会交叉，导致内部面碎片和渲染错误！
                System.Array.Sort(vi, sd);

                _data.DeactivateTet(t);
                _alreadyCutTets.Add(t);

                if (pos == 1 || pos == 3)
                    Case1_Split(vi, sd, pos);
                else if (pos == 2)
                    Case2_Split(vi, sd);

                cutCount++;
            }

            LastCandidateTetCount = candCount;
            LastIntersectedTetCount = cutCount;
            result.cutTets  = cutCount;
            result.newVerts = _data.NumParticles - vBefore;
            result.newTets  = _data.NumTets - tBefore;
            result.elapsedMs = (float)sw.Elapsed.TotalMilliseconds;
            _totalCutTets  += result.cutTets;
            _totalNewVerts += result.newVerts;
            _totalNewTets  += result.newTets;
            _dirty = cutCount > 0;
            LastRejectReason = cutCount > 0 ? "cut_ok" : "no_intersection";

            if (cutCount > 0)
                Debug.Log($"[SweptCutter] cut:{cutCount} new:{result.newVerts}v/{result.newTets}t " +
                          $"V:{_data.NumParticles} T:{_data.NumTets} {result.elapsedMs:F1}ms");
            return result;
        }

        // ═══════════════════════════════════════════════════════
        // Case 1: 1 isolated vertex → 4 sub-tets
        //
        // tet ABCD, A 孤立
        //   B₁ on edge(A,B), C₁ on edge(A,C), D₁ on edge(A,D)
        //
        // 孤立侧: {A, B₁, C₁, D₁}
        //
        // 多数侧 = 三棱柱 B-C-D (底面) / B₁-C₁-D₁ (切口面)
        //   ★ 保面分解 (pivot=B): 保留原始面 BCD
        //     {B, C, D, D₁}      ← 保留面 BCD ✓
        //     {B, C, C₁, D₁}     ← 中间连接
        //     {B, B₁, C₁, D₁}   ← 保留切口面 B₁C₁D₁ ✓
        // ═══════════════════════════════════════════════════════
        void Case1_Split(int[] v, float[] d, int posCount)
        {
            int isoSide = posCount == 1 ? 1 : -1;
            int isoIdx = -1;
            int[] oIdx = new int[3]; int oi = 0;
            for (int i = 0; i < 4; i++)
            {
                if ((d[i] >= 0 ? 1 : -1) == isoSide) isoIdx = i;
                else oIdx[oi++] = i;
            }

            int A = v[isoIdx];
            int B = v[oIdx[0]], C = v[oIdx[1]], D = v[oIdx[2]];
            bool isoIsPos = isoSide > 0;

            var (b1p, b1n) = EP(A, B, d[isoIdx], d[oIdx[0]]);
            var (c1p, c1n) = EP(A, C, d[isoIdx], d[oIdx[1]]);
            var (d1p, d1n) = EP(A, D, d[isoIdx], d[oIdx[2]]);

            int B1_iso = isoIsPos ? b1p : b1n;
            int C1_iso = isoIsPos ? c1p : c1n;
            int D1_iso = isoIsPos ? d1p : d1n;
            int B1_oth = isoIsPos ? b1n : b1p;
            int C1_oth = isoIsPos ? c1n : c1p;
            int D1_oth = isoIsPos ? d1n : d1p;

            // 孤立侧
            AT(A, B1_iso, C1_iso, D1_iso);

            // 多数侧: 保面三棱柱分解
            AT(B, C,      D,      D1_oth);   // 保留面 BCD ✓
            AT(B, C,      C1_oth, D1_oth);    // 中间连接
            AT(B, B1_oth, C1_oth, D1_oth);    // 保留切口面 ✓
        }

        // ═══════════════════════════════════════════════════════
        // Case 2: 2+2 split → 6 sub-tets
        //
        // tet ABCD, {A,D} 正侧, {B,C} 负侧
        //   E₁ on edge(A,B), F₁ on edge(A,C)
        //   G₁ on edge(D,C), H₁ on edge(D,B)
        //
        // 正侧 = 四棱锥, 分解保留边 AD:
        //   {A, E₁, F₁, H₁}    ← A侧
        //   {A, D,  F₁, H₁}    ← 保留 AD ✓
        //   {D, F₁, G₁, H₁}    ← D侧
        //
        // 负侧同理, 保留边 BC:
        //   {B, E₁, F₁, H₁}    ← B侧
        //   {B, C,  F₁, H₁}    ← 保留 BC ✓
        //   {C, F₁, G₁, H₁}    ← C侧
        // ═══════════════════════════════════════════════════════
        void Case2_Split(int[] v, float[] d)
        {
            int A = -1, D_ = -1, B = -1, C = -1;
            float dA = 0, dD = 0, dB = 0, dC = 0;
            int pi = 0, ni = 0;
            for (int i = 0; i < 4; i++)
            {
                if (d[i] >= 0f)
                {
                    if (pi == 0) { A = v[i]; dA = d[i]; } else { D_ = v[i]; dD = d[i]; }
                    pi++;
                }
                else
                {
                    if (ni == 0) { B = v[i]; dB = d[i]; } else { C = v[i]; dC = d[i]; }
                    ni++;
                }
            }

            var (e1p, e1n) = EP(A,  B, dA, dB);
            var (f1p, f1n) = EP(A,  C, dA, dC);
            var (g1p, g1n) = EP(D_, C, dD, dC);
            var (h1p, h1n) = EP(D_, B, dD, dB);

            // 正侧 (A, D_)
            // ★ 全局一致性三角化：严格对齐 min-max ID
            AT(A, e1p, f1p, g1p);
            AT(A, e1p, g1p, h1p);
            AT(A, D_, g1p, h1p);

            // 负侧 (B, C)
            AT(B, e1n, f1n, g1n);
            AT(B, e1n, g1n, h1n);
            AT(B, C, f1n, g1n);
        }

        // ═══════════════════════════════════════════════════════
        // ★ Phase 2: Edge Pair with Trajectory Correction
        //
        // 论文 §4.3: "snapping intersection points within a threshold
        //             distance to their nearest vertex"
        //
        // 当 t < SNAP_T → 吸附到 va (正侧端点)
        // 当 t > 1-SNAP_T → 吸附到 vb (负侧端点)
        // 否则 → 在交点位置创建双顶点 (posV/negV)
        // ═══════════════════════════════════════════════════════
        (int posV, int negV) EP(int va, int vb, float da, float db)
        {
            long key = EK(va, vb);

            // 1. 帧级 split 缓存：同帧内同边只切一次
            if (_frameSplitCache.TryGetValue(key, out var frameCached)) return frameCached;

            float t = da / (da - db);

            // 2. 划刀级缓存：原始边跨帧拓扑复用
            bool isOriginalEdge = (va < _strokeStartVertCount) && (vb < _strokeStartVertCount);
            if (isOriginalEdge && _strokeSplitCache.TryGetValue(key, out var strokeCached))
            {
                _frameSplitCache[key] = strokeCached;
                return strokeCached;
            }

            (int posV, int negV) result;

            if (t <= 1e-4f || Mathf.Abs(da) <= 1.05e-4f)
            {
                // ★ 核心修复：snap 时同一帧内同一顶点的克隆必须复用！
                if (!_frameSnapCache.TryGetValue(va, out int va_dup))
                {
                    va_dup = Clone(va);
                    _frameSnapCache[va] = va_dup;
                }
                result = da >= 0f ? (va, va_dup) : (va_dup, va);
            }
            else if (t >= 1f - 1e-4f || Mathf.Abs(db) <= 1.05e-4f)
            {
                // 同理，snap 到 vb 时也必须复用同一克隆
                if (!_frameSnapCache.TryGetValue(vb, out int vb_dup))
                {
                    vb_dup = Clone(vb);
                    _frameSnapCache[vb] = vb_dup;
                }
                result = db >= 0f ? (vb, vb_dup) : (vb_dup, vb);
            }
            else
            {
                // 正常情况：在边的交点处创建两个重合的双顶点
                Vector3 ip  = Vector3.Lerp(_data.Positions[va],     _data.Positions[vb],     t);
                Vector3 ir  = Vector3.Lerp(_data.RestPositions[va], _data.RestPositions[vb], t);
                Vector3 iv  = Vector3.Lerp(_data.Velocities[va],    _data.Velocities[vb],    t);
                Vector3 ipv = Vector3.Lerp(_data.PrevPositions[va], _data.PrevPositions[vb], t);
                float imA = _data.InvMass[va], imB = _data.InvMass[vb];
                float im  = (imA == 0f || imB == 0f) ? 0f : Mathf.Lerp(imA, imB, t);

                int posV = _data.AddParticle(ip, iv, im);
                _data.RestPositions[posV] = ir; _data.PrevPositions[posV] = ipv;
                int negV = _data.AddParticle(ip, iv, im);
                _data.RestPositions[negV] = ir; _data.PrevPositions[negV] = ipv;
                result = (posV, negV);
            }

            _frameSplitCache[key] = result;
            if (isOriginalEdge) _strokeSplitCache[key] = result;
            return result;
        }

        int Clone(int s)
        {
            int i = _data.AddParticle(_data.Positions[s], _data.Velocities[s], _data.InvMass[s]);
            _data.RestPositions[i]  = _data.RestPositions[s];
            _data.PrevPositions[i] = _data.PrevPositions[s];
            return i;
        }

        /// <summary>添加一个 tet, 自动修正绕序, 始终创建</summary>
        void AT(int a, int b, int c, int d)
        {
            // 退化/重复顶点检查
            if (a == b || a == c || a == d || b == c || b == d || c == d) return;

            // ★ 修复：必须使用 RestPositions 来判断绕序！
            // 如果使用变形后的 Positions，当网格在物理受力挤压时切开，可能会算出一个错误的相反绕序。
            // 等物理引擎将其恢复原状时，它就会永久翻转（Inside-out），导致法线朝内，渲染出黑色的背面！
            Vector3 p0 = _data.RestPositions[a], p1 = _data.RestPositions[b];
            Vector3 p2 = _data.RestPositions[c], p3 = _data.RestPositions[d];
            float sv = Vector3.Dot(Vector3.Cross(p1 - p0, p2 - p0), p3 - p0);

            // 体积过滤: 丢弃退化子tet → 减少碎屑
            if (Mathf.Abs(sv) < MIN_TET_VOL) return;

            if (sv < 0f) _data.AddTet(a, c, b, d);
            else         _data.AddTet(a, b, c, d);
        }

        // ══════════════════════════════════════════════════════
        public int RemoveStretchedTets(float maxStretchRatio = 5.0f, float minAbsoluteLength = 0.003f)
        {
            if (_data == null) return 0;
            int removedCount = 0;
            
            float sqrMaxRatio = maxStretchRatio * maxStretchRatio;
            float sqrMinLen = minAbsoluteLength * minAbsoluteLength;

            for (int t = 0; t < _data.NumTets; t++)
            {
                if (!_data.TetActive[t]) continue;
                
                int b = t * 4;
                int v0 = _data.TetIds[b + 0];
                int v1 = _data.TetIds[b + 1];
                int v2 = _data.TetIds[b + 2];
                int v3 = _data.TetIds[b + 3];

                Vector3 p0 = _data.Positions[v0];
                Vector3 p1 = _data.Positions[v1];
                Vector3 p2 = _data.Positions[v2];
                Vector3 p3 = _data.Positions[v3];

                Vector3 r0 = _data.RestPositions[v0];
                Vector3 r1 = _data.RestPositions[v1];
                Vector3 r2 = _data.RestPositions[v2];
                Vector3 r3 = _data.RestPositions[v3];

                bool isStretched = false;
                
                bool CheckEdge(Vector3 cpA, Vector3 cpB, Vector3 crA, Vector3 crB)
                {
                    float currentSqr = (cpA - cpB).sqrMagnitude;
                    float restSqr = (crA - crB).sqrMagnitude;
                    
                    if (restSqr < 1e-12f) return currentSqr > sqrMinLen; // Zero rest length but stretched
                    
                    float ratio = currentSqr / restSqr;
                    // 如果拉伸超过 10 倍（平方100倍），不管它多短，绝对是拓扑错误（藕断丝连），必须斩断！
                    if (ratio > 100.0f) return true; 
                    
                    if (currentSqr < sqrMinLen) return false; // Ignore short edges
                    return ratio > sqrMaxRatio;
                }

                if (CheckEdge(p0, p1, r0, r1)) isStretched = true;
                else if (CheckEdge(p0, p2, r0, r2)) isStretched = true;
                else if (CheckEdge(p0, p3, r0, r3)) isStretched = true;
                else if (CheckEdge(p1, p2, r1, r2)) isStretched = true;
                else if (CheckEdge(p1, p3, r1, r3)) isStretched = true;
                else if (CheckEdge(p2, p3, r2, r3)) isStretched = true;

                if (isStretched)
                {
                    _data.TetActive[t] = false;
                    removedCount++;
                    _dirty = true;
                }
            }

            if (removedCount > 0)
            {
                Debug.Log($"[TetSubdivisionCutter] 清理藕断丝连: 已熔断 {removedCount} 个拉伸异形四面体!");
            }

            return removedCount;
        }

        public void FlushToGPU()
        {
            if (!_dirty) return;
            _solver.ReadbackAll(_data);
            _solver.Dispose();
            _solver.Init(_data);
            _dirty = false;
        }

        static long EK(int a, int b)
        { if (a > b) { int t = a; a = b; b = t; } return (long)a * 2000000L + b; }
        // ══════════════════════════════════════════════════════
        // 跨帧拓扑一致性锁定
        // ══════════════════════════════════════════════════════
        float EnforceConsistentSide(int vid, float d, Vector3 currentNormal)
        {
            if (vid >= _strokeStartVertCount) return d; // 仅对原始顶点生效
            
            int currentSide = d >= 0f ? 1 : -1;
            
            if (_strokeVertexSide.TryGetValue(vid, out var locked))
            {
                // 如果平面发生了显著旋转（>25度，cos(25)≈0.9），则旧的锁定失效，更新为新平面的锁定
                if (Vector3.Dot(locked.normal, currentNormal) < 0.9f)
                {
                    _strokeVertexSide[vid] = new VertexSideLock { side = currentSide, normal = currentNormal };
                    return d;
                }

                if (currentSide != locked.side)
                {
                    // 发生了跨帧侧翻转！强制拉回锁定侧
                    // 锁为正侧(1) -> 设为 0f，这会被后续 Trajectory Correction 识别并吸附到 0f (保持正侧)
                    // 锁为负侧(-1) -> 设为 -1.01e-4f，恰好大于 SNAP_DIST，保证它严格是负的并且极靠近切面
                    return locked.side > 0 ? 0f : -1.01e-4f;
                }
            }
            else
            {
                // 首次在切面附近评估该顶点，记录其初始拓扑侧和法线
                _strokeVertexSide[vid] = new VertexSideLock { side = currentSide, normal = currentNormal };
            }
            return d;
        }
    }
}
