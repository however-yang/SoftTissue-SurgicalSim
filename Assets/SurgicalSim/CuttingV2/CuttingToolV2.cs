// CuttingToolV2.cs — 鼠标滑动射线切割
// 左键按住拖动 = 沿鼠标路径射线切割
// 使用 ConstraintCutter: 顶点复制 + 约束断裂

using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.CuttingV2
{
    public class CuttingToolV2 : MonoBehaviour
    {
        [Header("切割参数")]
        [Tooltip("工具切割影响半径")]
        public float cutRadius = 0.015f;

        [Header("鼠标控制")]
        [Tooltip("显示切割射线")]
        public bool showDebugRay = true;

        // ── 私有状态 ─────────────────────────────────────────
        ConstraintCutter   _cutter;
        TetMeshData        _data;
        TetMeshVisualizer  _visualizer;
        XPBDSolverGPU      _solver;
        Camera             _cam;

        // ── 鼠标切割状态 ─────────────────────────────────────
        bool    _isCutting;
        Vector3 _prevHitPoint;    // 上一帧射线命中点 (世界坐标)
        Vector3 _prevMouseWorld;  // 上一帧鼠标世界坐标 (用于方向)
        bool    _hasHit;          // 上一帧是否命中
        int     _cutFrameCount;   // 连续切割帧数

        // ── 切割刀具可视化 ───────────────────────────────────
        LineRenderer _debugLine;

        // ── 生命周期 ─────────────────────────────────────────

        public void Init(TetMeshData data, XPBDSolverGPU solver,
                         TetMeshVisualizer visualizer)
        {
            _data       = data;
            _solver     = solver;
            _visualizer = visualizer;
            _cam        = Camera.main;

            // 预分配容量 (切割会增加顶点)
            _data.EnsureCapacity(data.NumParticles * 3);

            _cutter = new ConstraintCutter();
            _cutter.Init(data, solver);

            // 创建调试线段 (显示切割射线)
            if (showDebugRay)
                SetupDebugLine();

            Debug.Log($"[CuttingToolV2] 鼠标射线切割初始化 | " +
                      $"P:{data.NumParticles} T:{data.NumTets}");
        }

        void Update()
        {
            if (_data == null || _cam == null) return;

            HandleMouseInput();
        }

        // ── 鼠标输入处理 ─────────────────────────────────────

        void HandleMouseInput()
        {
            // 左键按下开始切割
            if (Input.GetMouseButtonDown(0))
            {
                _isCutting = true;
                _hasHit = false;
                _cutFrameCount = 0;
                _cutter.ResetNormal();
            }

            // 左键松开停止切割
            if (Input.GetMouseButtonUp(0))
            {
                _isCutting = false;
                _hasHit = false;
                _cutFrameCount = 0;
                _cutter.ResetNormal();

                // 隐藏调试线
                if (_debugLine != null)
                    _debugLine.enabled = false;
            }

            // 切割中: 射线检测
            if (_isCutting)
            {
                Ray ray = _cam.ScreenPointToRay(Input.mousePosition);

                // 找到射线最近的网格表面点
                Vector3 hitPoint;
                bool hit = FindClosestSurfacePoint(ray, out hitPoint);

                if (hit)
                {
                    // 显示调试线
                    if (_debugLine != null)
                    {
                        _debugLine.enabled = true;
                        _debugLine.SetPosition(0, ray.origin);
                        _debugLine.SetPosition(1, hitPoint);
                    }

                    if (_hasHit)
                    {
                        Vector3 moveDir = hitPoint - _prevHitPoint;
                        float moveDist = moveDir.magnitude;

                        if (moveDist > 0.001f) // 最小移动距离
                        {
                            // 切割方向 = 相机视角方向 (刀片从屏幕刺入)
                            Vector3 viewDir = ray.direction.normalized;

                            _cutFrameCount++;

                            // 执行切割
                            _cutter.Cut(hitPoint, _prevHitPoint, viewDir, cutRadius);
                        }
                    }
                    _prevHitPoint = hitPoint;
                    _hasHit = true;
                }
            }
        }

        // ── 射线-表面交点检测 ─────────────────────────────────

        /// <summary>
        /// 找到射线到网格最近的表面点
        /// 方法: 遍历所有粒子, 找射线最近点
        /// </summary>
        bool FindClosestSurfacePoint(Ray ray, out Vector3 hitPoint)
        {
            hitPoint = Vector3.zero;
            float bestDistSq = float.MaxValue;
            bool found = false;

            int numP = _data.NumParticles;
            Vector3[] pos = _data.Positions;
            Vector3 ro = ray.origin;
            Vector3 rd = ray.direction;

            for (int i = 0; i < numP; i++)
            {
                // 点到射线的最近距离
                Vector3 op = pos[i] - ro;
                float t = Vector3.Dot(op, rd);
                if (t < 0) continue; // 点在射线后方

                Vector3 proj = ro + rd * t;
                float distSq = (pos[i] - proj).sqrMagnitude;

                // 选择最近的点 (在合理范围内)
                if (distSq < bestDistSq && distSq < cutRadius * cutRadius * 25f)
                {
                    bestDistSq = distSq;
                    hitPoint = pos[i]; // 使用粒子位置作为命中点
                    found = true;
                }
            }

            return found;
        }

        // ── 公开 API ─────────────────────────────────────────

        public void FlushCutToGPU()
        {
            if (_data == null || _solver == null || _cutter == null) return;

            if (_cutter.IsDirty)
            {
                // 增量上传 GPU
                _cutter.FlushToGPU();

                // 重建表面
                UpdateSurface();
            }
        }

        // ── GUI 信息 ─────────────────────────────────────────
        public int TotalSplitVerts    => _cutter != null ? _cutter.TotalSplitVerts : 0;
        public int TotalCutEdges      => _cutter != null ? _cutter.TotalCutEdges : 0;
        public int TotalDisabledEdges => TotalCutEdges;
        public int TotalCutTets       => 0; // 方案 C 不删除 tet

        // ── 表面重建 ─────────────────────────────────────────

        void UpdateSurface()
        {
            if (_visualizer == null) return;
            var mf = _visualizer.GetComponent<MeshFilter>();
            if (mf == null || mf.mesh == null) return;

            Mesh mesh = mf.mesh;
            mesh.Clear();

            int n = _data.NumParticles;
            if (_data.Positions.Length > n)
            {
                var trimmed = new Vector3[n];
                System.Array.Copy(_data.Positions, trimmed, n);
                mesh.vertices = trimmed;
            }
            else
            {
                mesh.vertices = _data.Positions;
            }

            mesh.indexFormat = n > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;

            int[] newTris = SurgicalSim.Cutting.SurfaceReconstructor.RebuildSurface(_data);
            mesh.triangles = newTris;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }

        // ── 调试可视化 ──────────────────────────────────────

        void SetupDebugLine()
        {
            _debugLine = gameObject.AddComponent<LineRenderer>();
            _debugLine.startWidth = 0.002f;
            _debugLine.endWidth   = 0.001f;
            _debugLine.material = new Material(Shader.Find("Sprites/Default"));
            _debugLine.startColor = Color.cyan;
            _debugLine.endColor   = Color.yellow;
            _debugLine.positionCount = 2;
            _debugLine.enabled = false;
        }

        void OnDestroy()
        {
            if (_debugLine != null)
                Destroy(_debugLine.material);
        }
    }
}
