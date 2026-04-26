// SoftBody.cs
// 整合主腳本：TetMeshLoader + XPBDSolverGPU + TetMeshVisualizer
// GPU 加速版本（已驗證 CPU 等價性後移植到 GPU）

using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;
using SurgicalSim.Cutting;
using SurgicalSim.CuttingV3;

namespace SurgicalSim
{
    [RequireComponent(typeof(TetMeshLoader))]
    [RequireComponent(typeof(TetMeshVisualizer))]
    public class SoftBody : MonoBehaviour
    {
        // ── Inspector 配置 ──────────────────────────────────
        [Header("GPU 求解器")]
        [Tooltip("拖入 Assets/SurgicalSim/Shaders/XPBDSolver.compute")]
        public ComputeShader xpbdComputeShader;

        [Header("物理參數 (NeoHookean)")]
        [Range(1, 30)]
        [Tooltip("子步數（越多越穩定）")]
        public int numSubSteps = 10;

        [Tooltip("邊約束柔性（額外 damping，0=無限硬）")]
        public float edgeCompliance = 2.0f;

        [Tooltip("楊氏模量 (Pa)。肝臟真實值 ~3000-6000, 較硬 ~1e5")]
        public float youngsModulus = 5000f;

        [Tooltip("泊松比。肝臟 0.45-0.49，越接近 0.5 越不可壓縮")]
        [Range(0.01f, 0.499f)]
        public float poissonsRatio = 0.45f;

        [Tooltip("速度阻尼 (0~1)。論文默認 0.05")]
        [Range(0f, 0.5f)]
        public float damping = 0.05f;

        [Tooltip("密度 (kg/m^3)。肝臟 ~1060")]
        public float density = 1000f;

        [Header("重力")]
        public bool  enableGravity = true;
        public float gravityY      = -9.81f;

        [Header("固定點（吊掛）")]
        [Tooltip("Y 坐標高於此值的頂點固定")]
        public float pinAboveY      = 0.22f;
        public bool  pinTopVertices = false;

        [Tooltip("竖着吊起来：旋转 90° 后固定顶部顶点")]
        public bool  hangVertically = true;

        [Tooltip("吊挂模式的地面 Y（需要比较低才能容纳竖挂的肝脏）")]
        public float hangGroundY    = -3.0f;

        [Header("地面")]
        public float groundY           = 0.0f;
        public bool  createGroundPlane = true;
        public Color groundColor       = new Color(0.3f, 0.7f, 0.3f, 0.8f);

        [Header("切割")]
        [Tooltip("啟用鼠標拖拽切割")]
        public bool enableCutting = true;

        [Tooltip("切割影響半徑")]
        [Range(0.003f, 0.05f)]
        public float cutRadius = 0.01f;

        [Tooltip("Virtual blade radius used for tet intersection; keep this thinner than the visible rod.")]
        [Range(0.0005f, 0.02f)]
        public float virtualBladeRadius = 0.0025f;

        [Header("切面渲染")]
        [Tooltip("切面內部顏色（双面渲染的背面色）")]
        public Color interiorColor = new Color(0.85f, 0.45f, 0.45f, 1f);

        [Tooltip("肝臟表面顏色")]
        public Color liverColor = new Color(0.85f, 0.25f, 0.15f, 1f);

        [Header("調試")]
        public bool pausePhysics = false;
        public bool showStats    = true;

        // ── 私有狀態 ─────────────────────────────────────────
        TetMeshLoader     _loader;
        TetMeshVisualizer _visualizer;
        XPBDSolverGPU     _gpuSolver;
        TetMeshData       _data;
        GameObject        _groundPlane;
        CuttingToolV3     _cuttingTool;



        bool  _physicsReady = false;
        float _avgStepMs    = 0f;
        float _simTime      = 0f;

        Vector3[] _initPositions;
        Vector3[] _initVelocities;

        // ── 生命週期 ─────────────────────────────────────────
        void Awake()
        {
            _loader     = GetComponent<TetMeshLoader>();
            _visualizer = GetComponent<TetMeshVisualizer>();
        }

        void Start()
        {
            if (xpbdComputeShader == null)
            {
                Debug.LogError("[SoftBody] 請在 Inspector 中拖入 XPBDSolver.compute！");
                return;
            }
            _loader.OnMeshLoaded += OnMeshLoaded;
            if (createGroundPlane) CreateGroundPlane();
        }

        void OnDestroy()
        {
            if (_loader != null) _loader.OnMeshLoaded -= OnMeshLoaded;
            _gpuSolver?.Dispose();
        }

        // ── 初始化 ────────────────────────────────────────────
        void OnMeshLoaded(TetMeshData data)
        {
            _data = data;

            // 竖挂模式：旋转肝脏 90° + 平移到合适位置
            if (hangVertically)
            {
                RotateMeshForHanging();
                groundY = hangGroundY;
                // 更新已创建的地面平面位置
                if (_groundPlane != null)
                    _groundPlane.transform.position = new Vector3(0f, groundY, 0f);
            }

            if (pinTopVertices || hangVertically) PinTopVertices();

            _gpuSolver = new XPBDSolverGPU(xpbdComputeShader)
            {
                NumSubSteps      = numSubSteps,
                EdgeCompliance   = edgeCompliance,
                YoungsModulus    = youngsModulus,
                PoissonsRatio    = poissonsRatio,
                Damping          = damping,
                Density          = density,
                Gravity          = enableGravity ? new Vector3(0f, gravityY, 0f) : Vector3.zero,
                GroundY          = groundY
            };
            _gpuSolver.Init(data);

            _initPositions  = (Vector3[])data.Positions.Clone();
            _initVelocities = (Vector3[])data.Velocities.Clone();

            // 初始化切割工具
            if (enableCutting)
            {
                _cuttingTool = GetComponent<CuttingToolV3>();
                if (_cuttingTool == null)
                    _cuttingTool = gameObject.AddComponent<CuttingToolV3>();
                _cuttingTool.cutRadius = cutRadius;
                _cuttingTool.Init(data, _gpuSolver, _visualizer);
            }

            // 設置雙面渲染材質（切面內部顏色）
            SetupTwoSidedMaterial();

            _physicsReady = true;

            Debug.Log($"[SoftBody] GPU 求解器啟動 | 重力: {gravityY} m/s² | " +
                      $"子步: {numSubSteps} | 地面 Y: {groundY}" +
                      $"{(enableCutting ? " | 切割: ON" : "")}");
        }

        // ── 物理更新（FixedUpdate）────────────────────────────
        int _physicsFrame = 0;
        float _maxPosDelta = 0f;

        void FixedUpdate()
        {
            if (!_physicsReady || _data == null || pausePhysics || _gpuSolver == null) return;

            // ★ 切割: 在物理步之前上传脏 buffer 到 GPU
            if (_cuttingTool != null)
            {
                try
                {
                    _cuttingTool.FlushCutToGPU();
                }
                catch (System.Exception ex)
                {
                    Debug.LogError($"[SoftBody] FlushCutToGPU 異常: {ex.Message}\n{ex.StackTrace}");
                }
            }

            // 找一个非固定粒子来诊断
            int diagIdx = 0;
            for (int i = 0; i < _data.NumParticles; i++)
                if (_data.InvMass[i] > 0f) { diagIdx = i; break; }
            Vector3 posBefore = _data.Positions[diagIdx];

            _gpuSolver.NumSubSteps      = numSubSteps;
            _gpuSolver.EdgeCompliance   = edgeCompliance;
            _gpuSolver.Damping          = damping;
            _gpuSolver.Gravity          = enableGravity ? new Vector3(0f, gravityY, 0f) : Vector3.zero;
            _gpuSolver.GroundY          = groundY;

            try
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                _gpuSolver.Step(Time.fixedDeltaTime);
                _gpuSolver.ReadbackPositions(_data);
                sw.Stop();

                // 計算位置變化量（用非固定粒子）
                Vector3 posAfter = _data.Positions[diagIdx];
                float delta = (posAfter - posBefore).magnitude;
                _maxPosDelta = Mathf.Max(_maxPosDelta * 0.99f, delta);

                float ms   = (float)sw.Elapsed.TotalMilliseconds;
                _avgStepMs = _avgStepMs * 0.95f + ms * 0.05f;

                // 前5帧打印详细诊断
                if (_physicsFrame < 5)
                {
                    Debug.Log($"[SoftBody] Frame {_physicsFrame}: " +
                              $"DiagP[{diagIdx}] invM={_data.InvMass[diagIdx]:G4} " +
                              $"before={posBefore} after={posAfter} " +
                              $"Δ={delta:E3} ms={ms:F1}");
                }
            }
            catch (System.Exception ex)
            {
                Debug.LogError($"[SoftBody] 物理步異常（可能是拓撲變化中）: {ex.Message}");
            }

            _simTime += Time.fixedDeltaTime;
            _physicsFrame++;

            _visualizer.Refresh();

        }

        // ── 鍵盤控制 ─────────────────────────────────────────
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.P))
            {
                pausePhysics = !pausePhysics;
                Debug.Log($"[SoftBody] {(pausePhysics ? "暫停" : "繼續")}");
            }
            if (Input.GetKeyDown(KeyCode.R)) ResetMesh();
            if (Input.GetKeyDown(KeyCode.G) && _visualizer != null)
                _visualizer.showWireframe = !_visualizer.showWireframe;

            // F5: 施加冲击力诊断 — 证明物理是否响应
            if (Input.GetKeyDown(KeyCode.F5) && _physicsReady)
            {
                ApplyDiagnosticJolt();
            }

            // 同步切割半径
            if (_cuttingTool != null)
            {
                _cuttingTool.cutRadius = cutRadius;
            }
        }

        /// <summary>
        /// 诊断: 给所有非固定粒子施加瞬间速度冲击
        /// 如果肝脏会弹跳 → 物理正常
        /// 如果肝脏不动 → 求解器有 bug
        /// </summary>
        void ApplyDiagnosticJolt()
        {
            if (_data == null || _gpuSolver == null) return;

            // 先从 GPU 读回最新状态
            _gpuSolver.ReadbackAll(_data);

            int movedCount = 0;
            int pinnedCount = 0;
            float minInvM = float.MaxValue, maxInvM = 0f;

            for (int i = 0; i < _data.NumParticles; i++)
            {
                if (_data.InvMass[i] == 0f)
                {
                    pinnedCount++;
                    continue;
                }

                minInvM = Mathf.Min(minInvM, _data.InvMass[i]);
                maxInvM = Mathf.Max(maxInvM, _data.InvMass[i]);

                // 施加向下的冲击速度
                _data.Velocities[i] += new Vector3(0f, -2f, 0f);
                movedCount++;
            }

            // 上传修改后的速度到 GPU
            _gpuSolver.UploadPositionsAndVelocities(_data);

            Debug.Log($"[SoftBody] ★ F5 冲击诊断 ★ " +
                      $"已施加冲击: {movedCount} 粒子 | " +
                      $"固定: {pinnedCount} | " +
                      $"InvMass范围: [{minInvM:G4}, {maxInvM:G4}]");
        }

        // ── 双面材质设置 ──────────────────────────────────────
        void SetupTwoSidedMaterial()
        {
            var shader = Shader.Find("SurgicalSim/TwoSidedLiver");
            if (shader == null)
            {
                Debug.LogWarning("[SoftBody] 未找到 SurgicalSim/TwoSidedLiver 着色器，使用默认材质");
                return;
            }

            var mat = new Material(shader);
            mat.SetColor("_Color", liverColor);
            mat.SetColor("_InteriorColor", interiorColor);

            var renderer = _visualizer.GetComponent<MeshRenderer>();
            if (renderer != null)
            {
                renderer.material = mat;
                Debug.Log("[SoftBody] 双面渲染材质已设置（外表面 + 切面内部）");
            }
        }

        // ── 地面平面 ──────────────────────────────────────────
        void CreateGroundPlane()
        {
            _groundPlane = GameObject.CreatePrimitive(PrimitiveType.Plane);
            _groundPlane.name = "GroundPlane";
            _groundPlane.transform.position  = new Vector3(0f, groundY, 0f);
            _groundPlane.transform.localScale = new Vector3(0.5f, 1f, 0.5f);

            var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
            mat.color = groundColor;
            _groundPlane.GetComponent<MeshRenderer>().material = mat;

            var col = _groundPlane.GetComponent<Collider>();
            if (col != null) Destroy(col);
        }

        // ── 工具方法 ─────────────────────────────────────────
        void PinTopVertices()
        {
            int pinned = 0;
            for (int i = 0; i < _data.NumParticles; i++)
                if (_data.RestPositions[i].y > pinAboveY)
                { _data.PinParticle(i); pinned++; }
            Debug.Log($"[SoftBody] 固定 {pinned} 頂點 (Y > {pinAboveY:F3})");
        }

        /// <summary>
        /// 旋转肝脏使其竖直悬挂：
        /// 将所有顶点绕 X 轴旋转 90°，使原来水平的肝脏变为竖直
        /// 然后平移使顶部在 Y=0.5 附近，底部自然下垂
        /// </summary>
        void RotateMeshForHanging()
        {
            // 计算当前 bounding box
            Vector3 bboxMin = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 bboxMax = new Vector3(float.MinValue, float.MinValue, float.MinValue);
            for (int i = 0; i < _data.NumParticles; i++)
            {
                Vector3 p = _data.Positions[i];
                bboxMin = Vector3.Min(bboxMin, p);
                bboxMax = Vector3.Max(bboxMax, p);
            }
            Vector3 center = (bboxMin + bboxMax) * 0.5f;

            // 找到最长轴，沿最长轴旋转到 Y 方向
            Vector3 size = bboxMax - bboxMin;
            Debug.Log($"[SoftBody] 旋转前 BBox: min={bboxMin}, max={bboxMax}, size={size}");

            // 绕 X 轴旋转 90°（使 Z 轴方向变成 Y 轴方向 = 竖直）
            for (int i = 0; i < _data.NumParticles; i++)
            {
                Vector3 p = _data.Positions[i] - center;
                // (x, y, z) → (x, -z, y): Z轴变成Y轴（竖直）
                Vector3 rotated = new Vector3(p.x, -p.z, p.y);
                _data.Positions[i] = rotated;
                _data.PrevPositions[i] = rotated;
                _data.RestPositions[i] = rotated;
            }

            // 重新计算 bbox
            bboxMin = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            bboxMax = new Vector3(float.MinValue, float.MinValue, float.MinValue);
            for (int i = 0; i < _data.NumParticles; i++)
            {
                bboxMin = Vector3.Min(bboxMin, _data.Positions[i]);
                bboxMax = Vector3.Max(bboxMax, _data.Positions[i]);
            }

            // 平移：使顶部在 Y=1.0，底部悬空，XZ 居中
            float targetTopY = 1.0f;
            float offsetY = targetTopY - bboxMax.y;
            float offsetX = -(bboxMin.x + bboxMax.x) * 0.5f; // XZ 居中
            float offsetZ = -(bboxMin.z + bboxMax.z) * 0.5f;
            Vector3 offset = new Vector3(offsetX, offsetY, offsetZ);
            for (int i = 0; i < _data.NumParticles; i++)
            {
                _data.Positions[i]     += offset;
                _data.PrevPositions[i] += offset;
                _data.RestPositions[i] += offset;
            }

            // 自动设置 pinAboveY：固定顶部 25% 的顶点（更稳定的锚点）
            float newMax = targetTopY;
            float newMin = bboxMin.y + offsetY;
            float range = newMax - newMin;
            pinAboveY = newMax - range * 0.25f;

            Debug.Log($"[SoftBody] 旋转后 BBox: [{newMin:F3}, {newMax:F3}] | " +
                      $"pinAboveY = {pinAboveY:F3} | 范围: {range:F3}");
        }

        public void ResetMesh()
        {
            if (_data == null || _initPositions == null) return;
            System.Array.Copy(_initPositions,  _data.Positions,     _data.NumParticles);
            System.Array.Copy(_initVelocities, _data.Velocities,    _data.NumParticles);
            System.Array.Copy(_initPositions,  _data.PrevPositions, _data.NumParticles);

            // 恢复所有 tet 为活跃状态
            for (int t = 0; t < _data.NumTets; t++)
                _data.TetActive[t] = true;

            _gpuSolver?.Init(_data);

            // 重新初始化切割工具
            if (_cuttingTool != null)
                _cuttingTool.Init(_data, _gpuSolver, _visualizer);  // V3 reinit

            // 恢复原始表面三角形
            var mf = _visualizer.GetComponent<MeshFilter>();
            if (mf != null && mf.mesh != null)
            {
                mf.mesh.triangles = new int[0];
                mf.mesh.triangles = _data.SurfaceTriIds;
                mf.mesh.RecalculateNormals();
            }


            _visualizer.Refresh();
            Debug.Log("[SoftBody] 重置完成（含切割狀態）");
        }

        // ── GUI ───────────────────────────────────────────────
        float _fps = 60f;

        void OnGUI()
        {
            if (!showStats || _data == null) return;

            // FPS 平滑计算
            float dt = Time.unscaledDeltaTime;
            if (dt > 0f) _fps = _fps * 0.9f + (1f / dt) * 0.1f;

            var style = new GUIStyle(GUI.skin.box) { fontSize = 13 };
            style.normal.textColor = Color.white;
            style.alignment = TextAnchor.UpperLeft;

            int splitVerts = _cuttingTool != null ? _cuttingTool.TotalSplitVerts : 0;
            int cutTets    = _cuttingTool != null ? _cuttingTool.TotalCutTets : 0;
            bool toolPressed = _cuttingTool != null && _cuttingTool.ToolCutPressed;
            float cutMoveDist = _cuttingTool != null ? _cuttingTool.LastToolMoveDistance : 0f;
            int cutCandidates = _cuttingTool != null ? _cuttingTool.LastCandidateTetCount : 0;
            int cutHits = _cuttingTool != null ? _cuttingTool.LastIntersectedTetCount : 0;
            string cutReason = _cuttingTool != null ? _cuttingTool.LastCutRejectReason : "no_tool";

            // FPS 颜色标记
            string fpsColor = _fps >= 30 ? "#00FF00" : (_fps >= 15 ? "#FFFF00" : "#FF0000");

            string info =
                $"<color={fpsColor}>FPS: {_fps:F0}</color>\n" +
                $"XPBD GPU | Tet Subdivision Cut (PG2025)\n" +
                $"粒子:    {_data.NumParticles:N0}\n" +
                $"四面體:  {_data.NumTets:N0}\n" +
                $"子步數:  {numSubSteps}\n" +
                $"步時間: {_avgStepMs:F1} ms\n" +
                $"物理幀: {_physicsFrame} | Δpos: {_maxPosDelta:E2}\n" +
                $"切割:   新顶点: {splitVerts} | 切割tet: {cutTets}\n" +
                $"模式:   {(hangVertically ? "竖挂" : "平放")} | " +
                "UIOJKL移动 Space切割 方向键旋转 P暂停";

            // 使用 richText
            style.richText = true;

            GUI.Box(new Rect(10, 10, 390, 240), info, style);

            string cutDebug =
                $"刀状态: {toolPressed}\n" +
                $"move: {cutMoveDist:E2}\n" +
                $"cand/hit: {cutCandidates}/{cutHits}\n" +
                $"reason: {cutReason}";
            GUI.Box(new Rect(10, 255, 190, 95), cutDebug, style);
        }
    }
}
