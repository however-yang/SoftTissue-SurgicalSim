// CuttingToolV3.cs — Phase 1: 扫掠面连续切割控制器
//
// 工作流:
//   1. SurgicalTool 提供刀刃线段端点 (BladeA, BladeB)
//   2. 碰触检测: 刀刃附近是否有活跃 tet
//   3. 将刀刃端点转换到 mesh local space
//   4. 传递扫掠四边形 (A0,B0,A1,B1) 给 TetSubdivisionCutter
//   5. FixedUpdate 中 flush 到 GPU

using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Cutting;
using SurgicalSim.Physics;

namespace SurgicalSim.CuttingV3
{
    public class CuttingToolV3 : MonoBehaviour
    {
        [Header("切割参数")]
        [Tooltip("扫掠面多大范围内的tet会被检测")]
        public float cutRadius = 0.05f;

        [Header("碰触检测")]
        [Tooltip("刀刃附近多大范围算碰到肝脏")]
        public float contactRadius = 0.03f;

        [Header("输入")]
        public bool useSurgicalTool = true;

        [Header("性能")]
        [Range(1, 10)]
        public int flushInterval = 1;

        [Header("调试")]
        public bool showDebugRay = true;

        // ── 私有 ─────────────────────────────────────────────
        TetSubdivisionCutter _cutter;
        TetMeshData          _data;
        TetMeshVisualizer    _visualizer;
        XPBDSolverGPU        _solver;
        SurgicalTool         _surgicalTool;
        Camera               _cam;

        bool    _isCutting;
        bool    _wasInsideMesh;
        int     _framesSinceFlush;

        // 上帧刀刃端点 (local space)
        Vector3 _prevBladeA_local;
        Vector3 _prevBladeB_local;
        bool    _hasPrevBlade;

        // 鼠标模式
        Vector3 _prevHitPoint;
        bool    _hasHit;

        LineRenderer _debugLine;

        // ══════════════════════════════════════════════════════
        public void Init(TetMeshData data, XPBDSolverGPU solver,
                         TetMeshVisualizer visualizer)
        {
            _data       = data;
            _solver     = solver;
            _visualizer = visualizer;
            _cam        = Camera.main;

            _cutter = new TetSubdivisionCutter();
            _cutter.Init(data, solver);
            _framesSinceFlush = 0;
            _wasInsideMesh    = false;
            _hasPrevBlade     = false;

            if (useSurgicalTool) EnsureSurgicalTool();
            if (showDebugRay)    SetupDebugLine();

            Debug.Log($"[CuttingToolV3] Init P:{data.NumParticles} T:{data.NumTets}");
        }

        // ══════════════════════════════════════════════════════
        void Update()
        {
            if (_data == null) return;
            if (useSurgicalTool) AutoCutStep();
            else                 MouseCutStep();
        }

        // ══════════════════════════════════════════════════════
        // 自动碰触切割 — 扫掠面版
        // ══════════════════════════════════════════════════════
        void AutoCutStep()
        {
            if (_surgicalTool == null) { EnsureSurgicalTool(); return; }

            Transform tf = _visualizer != null ? _visualizer.transform : transform;

            // 当前刀刃端点 → local space
            Vector3 bladeA_local = tf.InverseTransformPoint(_surgicalTool.BladeA);
            Vector3 bladeB_local = tf.InverseTransformPoint(_surgicalTool.BladeB);

            // 碰触检测: 刀刃(线段)是否接近肝脏
            bool insideMesh = IsBladeNearMesh(bladeA_local, bladeB_local);

            if (insideMesh && !_wasInsideMesh)
            {
                // 进入肝脏 — 开始切割 (不清除 edgeCache, 保持顶点共享)
                _isCutting = true;
                _prevBladeA_local = bladeA_local;
                _prevBladeB_local = bladeB_local;
                _hasPrevBlade = true;
                _cutter.ResetStroke(); // ★ 开始新stroke时重置锁定法线
                Debug.Log("[CuttingToolV3] 碰触检测: 进入肝脏, 开始切割");
            }
            else if (!insideMesh && _wasInsideMesh)
            {
                // ★ 出刀修复：离开肝脏的那一帧必须执行最后一刀！
                // 不做这一刀，刀从最后一个 inside 位置到 outside 位置之间扫过的区域完全空白，
                // 边缘残留的极小 sliver 四面体就永久卡在这个死区里，形成藕断丝连！
                if (_hasPrevBlade)
                {
                    float lastMoveA = (bladeA_local - _prevBladeA_local).magnitude;
                    float lastMoveB = (bladeB_local - _prevBladeB_local).magnitude;
                    if (Mathf.Max(lastMoveA, lastMoveB) > 1e-5f)
                        _cutter.Cut(_prevBladeA_local, _prevBladeB_local,
                                    bladeA_local, bladeB_local);
                }
                _isCutting = false;
                _hasPrevBlade = false;
                Debug.Log("[CuttingToolV3] 碰触检测: 离开肝脏（出刀最后一切完成）");
            }

            _wasInsideMesh = insideMesh;

            if (!_isCutting || !_hasPrevBlade) return;

            // 刀刃移动量
            float moveA = (bladeA_local - _prevBladeA_local).magnitude;
            float moveB = (bladeB_local - _prevBladeB_local).magnitude;
            float maxMove = Mathf.Max(moveA, moveB);

            if (maxMove < 1e-5f) return; // 没动

            // 调用扫掠面切割
            _cutter.Cut(_prevBladeA_local, _prevBladeB_local,
                        bladeA_local, bladeB_local);

            // 更新上帧
            _prevBladeA_local = bladeA_local;
            _prevBladeB_local = bladeB_local;
        }

        // ── 碰触检测: 刀刃线段是否接近肝脏 ──────────────────
        bool IsBladeNearMesh(Vector3 bladeA, Vector3 bladeB)
        {
            // 检测刀刃线段附近是否有 tet 顶点
            // 不要使用 mf.mesh.bounds, 因为在软体形变时 Unity 的 Bounds 可能会缓存过时数据，导致切到一半停止！
            float r2 = contactRadius * contactRadius;
            for (int t = 0; t < _data.NumTets; t++)
            {
                if (!_data.TetActive[t]) continue;
                int b = t * 4;
                Vector3 v0 = _data.Positions[_data.TetIds[b]];
                Vector3 v1 = _data.Positions[_data.TetIds[b+1]];
                Vector3 v2 = _data.Positions[_data.TetIds[b+2]];
                Vector3 v3 = _data.Positions[_data.TetIds[b+3]];

                // 只要有任何一个顶点到线段距离小于 r2
                if (PointSegDistSq(v0, bladeA, bladeB) < r2 ||
                    PointSegDistSq(v1, bladeA, bladeB) < r2 ||
                    PointSegDistSq(v2, bladeA, bladeB) < r2 ||
                    PointSegDistSq(v3, bladeA, bladeB) < r2) 
                    return true;
            }
            return false;
        }

        static float PointSegDistSq(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float sq = ab.sqrMagnitude;
            if (sq < 1e-12f) return (p - a).sqrMagnitude;
            float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / sq);
            return (p - (a + ab * t)).sqrMagnitude;
        }

        // ══════════════════════════════════════════════════════
        // 鼠标切割 (保留, 未改为扫掠面)
        // ══════════════════════════════════════════════════════
        void MouseCutStep()
        {
            // TODO: Phase 2 — mouse切割也改为扫掠面
        }

        // ══════════════════════════════════════════════════════
        // FlushCutToGPU — SoftBody.FixedUpdate 调用
        // ══════════════════════════════════════════════════════
        public void FlushCutToGPU()
        {
            if (_data == null || _solver == null || _cutter == null) return;

            // Continually monitor and automatically break any "lotus root silk" (highly stretched tets)
            // with a smart absolute length threshold (e.g. 2cm) so it NEVER breaks normal small surface slivers.
            if (_cutter is TetSubdivisionCutter tCutter)
            {
                tCutter.RemoveStretchedTets(5.0f, 0.02f);
            }

            if (!_cutter.IsDirty) return;

            _framesSinceFlush++;
            if (_framesSinceFlush < flushInterval) return;

            _cutter.FlushToGPU();
            UpdateSurface();
            _framesSinceFlush = 0;
        }

        // ── 诊断 GUI 属性 ────────────────────────────────────
        public int    TotalSplitVerts         => _cutter?.TotalNewVerts         ?? 0;
        public int    TotalCutEdges           => _cutter?.TotalNewTets          ?? 0;
        public int    TotalCutTets            => _cutter?.TotalCutTets          ?? 0;
        public bool   ToolCutPressed          => _isCutting;
        public float  LastToolMoveDistance    => _cutter?.LastMoveDistance      ?? 0f;
        public int    LastCandidateTetCount   => _cutter?.LastCandidateTetCount ?? 0;
        public int    LastIntersectedTetCount => _cutter?.LastIntersectedTetCount ?? 0;
        public string LastCutRejectReason     => _cutter?.LastRejectReason      ?? "no_cutter";

        // ══════════════════════════════════════════════════════
        void UpdateSurface()
        {
            if (_visualizer == null) return;
            // SurfaceReconstructor 自动检测切口面 (count==1 边界面)
            int[] tris = SurfaceReconstructor.RebuildSurface(_data);
            _visualizer.RebuildTopology(tris);
        }

        void EnsureSurgicalTool()
        {
            if (_surgicalTool != null) return;
            _surgicalTool = GetComponent<SurgicalTool>()
                         ?? gameObject.AddComponent<SurgicalTool>();
        }

        void SetupDebugLine()
        {
            if (GetComponent<LineRenderer>()) return;
            _debugLine = gameObject.AddComponent<LineRenderer>();
            _debugLine.startWidth = 0.003f; _debugLine.endWidth = 0.001f;
            _debugLine.material = new Material(Shader.Find("Sprites/Default"));
            _debugLine.startColor = Color.cyan; _debugLine.endColor = Color.yellow;
            _debugLine.positionCount = 2; _debugLine.enabled = false;
        }

        void OnDestroy()
        {
            if (_debugLine && _debugLine.material) Destroy(_debugLine.material);
        }
    }
}
