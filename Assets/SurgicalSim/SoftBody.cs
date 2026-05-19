// SoftBody.cs
// 整合主腳本：TetMeshLoader + XPBDSolverGPU + TetMeshVisualizer
// GPU 加速版本（已驗證 CPU 等價性後移植到 GPU）

using UnityEngine;
using System.IO;
using SurgicalSim.Core;
using SurgicalSim.Physics;
using SurgicalSim.Cutting;
using SurgicalSim.CuttingV3;
using SurgicalSim.Grasping;
using SurgicalSim.Rendering;

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

        [Header("Tool Contact")]
        [Range(0.001f, 0.08f)]
        public float toolContactDistance = 0.006f;
        [Range(0f, 1e-4f)]
        public float toolContactCompliance = 2e-6f;
        [Range(1, 8)]
        public int toolContactIterations = 4;
        [Range(1, 4)]
        public int toolContactCouplingPasses = 3;

        [Header("Grasping")]
        [Tooltip("Enable gripper visual, GPU tool collision, and particle grasping. Disable for performance isolation.")]
        public bool enableGrasping = true;

        [Header("Performance")]
        [Tooltip("Limit Unity FixedUpdate catch-up so a slow contact step does not run several physics steps before one rendered frame.")]
        public bool limitFixedUpdateCatchUp = true;

        [Tooltip("Maximum FixedUpdate calls Unity may catch up before one rendered frame when the soft-body step is slow.")]
        [Range(1, 5)]
        public int maxFixedUpdatesPerRenderedFrame = 1;

        [Header("切割")]
        [Tooltip("啟用鼠標拖拽切割")]
        public bool enableCutting = false;

        [Tooltip("切割影響半徑")]
        [Range(0.003f, 0.05f)]
        public float cutRadius = 0.01f;

        [Tooltip("Virtual blade radius used for tet intersection; keep this thinner than the visible rod.")]
        [Range(0.0005f, 0.02f)]
        public float virtualBladeRadius = 0.0025f;

        [Header("渲染 - 基础颜色")]
        [Tooltip("切面內部顏色（切割内表面色）")]
        public Color interiorColor = new Color(0.30f, 0.025f, 0.018f, 1f);

        [Tooltip("肝臟表面顏色")]
        public Color liverColor = new Color(0.62f, 0.18f, 0.10f, 1f);

        [Tooltip("肝臟表面紋理（Albedo）。留空時嘗試自動載入 Assets/Texture/liver2.png")]
        public Texture2D liverTexture;

        [Range(0f, 1f)]
        public float liverTextureStrength = 0.98f;

        [Range(0.25f, 2.5f)]
        public float liverTextureContrast = 1.35f;

        [Range(0f, 1f)]
        public float liverTextureColorBlend = 1.0f;

        [Range(0f, 1f)]
        public float liverUvTextureWeight = 0.0f;

        public Vector2 liverTextureTiling = Vector2.one;

        [Range(0.1f, 20f)]
        public float liverTriplanarScale = 5.5f;

        [Range(0.1f, 16f)]
        public float liverTriplanarBlend = 4f;

        [Header("渲染 - 法线贴图")]
        [Tooltip("法线贴图 (Normal Map)，留空则不使用")]
        public Texture2D liverNormalMap;

        [Tooltip("SofaUnity liver2_height.png，用于恢复表面细节明暗")]
        public Texture2D liverHeightMap;

        [Tooltip("SofaUnity liver2_spec.png，用于控制局部湿润高光")]
        public Texture2D liverSpecularMap;

        [Range(0f, 3f)]
        [Tooltip("法线贴图强度")]
        public float liverNormalStrength = 0.12f;

        [Range(0f, 2f)]
        public float liverProceduralNormalStrength = 0.05f;

        [Header("渲染 - PBR 高光")]
        [Range(0f, 1f)]
        [Tooltip("粗糙度 (0=镜面, 1=粗糙漫反射)。肝脏约 0.25-0.35")]
        public float liverRoughness = 0.70f;

        [Range(0f, 3f)]
        [Tooltip("GGX 高光强度")]
        public float liverSpecularStrength = 0.18f;

        [Tooltip("高光颜色（稍偏暖色模拟湿润感）")]
        public Color liverSpecularColor = new Color(0.95f, 0.90f, 0.85f, 1f);

        [Header("渲染 - 次表面散射 SSS")]
        [Tooltip("SSS 散射颜色（肝脏应为橙红色）")]
        public Color liverSSSColor = new Color(0.76f, 0.13f, 0.055f, 1f);

        [Range(0f, 3f)]
        [Tooltip("SSS 包裹漫射强度")]
        public float liverSSSStrength = 0.06f;

        [Range(0f, 2f)]
        [Tooltip("背光透射强度（光从背面穿透薄组织）")]
        public float liverSSSBacklight = 0.04f;

        [Range(1f, 24f)]
        [Tooltip("背光聚焦度（值越大高光越集中）")]
        public float liverSSSPower = 7f;

        [Range(0f, 1f)]
        [Tooltip("漫射 Wrap 因子（光线绕到暗面的程度）")]
        public float liverSSSWrap = 0.32f;

        [Header("渲染 - Fresnel 边缘")]
        [Range(0f, 3f)]
        [Tooltip("Fresnel 边缘光强度（模拟湿润边缘反光）")]
        public float liverFresnelStrength = 0.08f;

        [Range(1f, 8f)]
        [Tooltip("Fresnel 衰减速度")]
        public float liverFresnelPow = 4.2f;

        [Header("Rendering - GPU Tissue Detail")]
        [Range(0f, 1f)]
        public float liverWetness = 0.18f;

        [Range(0f, 1f)]
        public float liverMicroMottleStrength = 0.08f;

        [Range(1f, 80f)]
        public float liverMicroMottleScale = 20f;

        [Range(0.2f, 1.5f)]
        public float liverAlbedoBrightness = 0.82f;

        [Range(0f, 1f)]
        public float liverVeinStrength = 0.10f;

        [Range(1f, 80f)]
        public float liverVeinScale = 22f;

        [Range(0f, 1f)]
        [Tooltip("调试用：1 时直接显示 liver2.png，绕过光照。用于判断是否是 UV/贴图链路问题。")]
        public float liverDebugTextureOnly = 0f;

        [Header("Rendering - SofaUnity Visual Mesh")]
        public bool useSofaUnityVisualMesh = true;
        public bool hideTetSurfaceWhenSofaVisualActive = true;
        [Tooltip("Use the current liver3-HD tetra surface for the SofaUnity visual renderer. This matches Project2's Tetra2TriangleTopologicalMapping path and avoids misbinding an external OBJ shell.")]
        public bool useTetSurfaceForSofaUnityVisual = true;
        [Tooltip("Replicate Project2 Assets/SofaUnity/Core/Resources/Materials/Liver2.mat for the visible liver surface.")]
        public bool useProject2Liver2Material = true;
        public Vector2 sofaUnityVisualUvTiling = Vector2.one;
        public TextAsset sofaUnityVisualObj;
        public Material sofaUnityVisualMaterial;
        public string sofaUnityVisualObjResource = "SurgicalSim/SofaUnity/liver-smooth-obj";

        [Range(0.75f, 1.05f)]
        public float sofaUnityVisualFitPadding = 0.985f;

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
        GripperTool       _gripperTool;
        SofaUnityVisualLiverRenderer _sofaVisualRenderer;
        Texture2D         _resolvedLiverTexture;
        Texture2D         _resolvedProject2BumpTexture;
        Texture2D         _resolvedLiverHeightTexture;



        bool  _physicsReady = false;
        float _avgStepMs    = 0f;
        float _lastSolveDispatchMs = 0f;
        float _lastReadbackMs = 0f;
        float _lastPhysicsTotalMs = 0f;
        float _lastVisualRefreshMs = 0f;
        float _lastToolUploadMs = 0f;
        float _lastGripperStepMs = 0f;
        float _originalMaximumDeltaTime = 0f;
        bool _hasOriginalMaximumDeltaTime = false;
        int _fixedUpdatesThisRenderFrame = 0;
        int _lastFixedUpdatesPerRenderFrame = 0;
        float _simTime      = 0f;

        Vector3[] _initPositions;
        Vector3[] _initVelocities;

        // ── 生命週期 ─────────────────────────────────────────
        void Awake()
        {
            _loader     = GetComponent<TetMeshLoader>();
            _visualizer = GetComponent<TetMeshVisualizer>();
            _originalMaximumDeltaTime = Time.maximumDeltaTime;
            _hasOriginalMaximumDeltaTime = true;
            ApplyFixedUpdateCatchUpLimit();
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
            ApplyFixedUpdateCatchUpLimit();
        }

        void OnDestroy()
        {
            if (_loader != null) _loader.OnMeshLoaded -= OnMeshLoaded;
            _gpuSolver?.Dispose();
            if (_hasOriginalMaximumDeltaTime)
                Time.maximumDeltaTime = _originalMaximumDeltaTime;
        }

        void ApplyFixedUpdateCatchUpLimit()
        {
            if (!Application.isPlaying)
                return;

            if (!limitFixedUpdateCatchUp)
            {
                if (_hasOriginalMaximumDeltaTime)
                    Time.maximumDeltaTime = _originalMaximumDeltaTime;
                return;
            }

            int maxSteps = Mathf.Max(1, maxFixedUpdatesPerRenderedFrame);
            Time.maximumDeltaTime = Mathf.Max(Time.fixedDeltaTime, Time.fixedDeltaTime * maxSteps);
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
                GroundY          = groundY,
                ToolContactDistance = toolContactDistance,
                ToolContactCompliance = toolContactCompliance,
                ToolContactIterations = toolContactIterations,
                ToolContactCouplingPasses = toolContactCouplingPasses
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

            // Gripper tool is optional so pure liver performance can be profiled.
            if (enableGrasping)
            {
                _gripperTool = GetComponent<GripperTool>();
                if (_gripperTool == null)
                    _gripperTool = gameObject.AddComponent<GripperTool>();
                _gripperTool.enabled = true;
                _gripperTool.Init(data, _gpuSolver, _visualizer);
            }
            else
            {
                _gripperTool = null;
                var existingGripper = GetComponent<GripperTool>();
                if (existingGripper != null)
                    existingGripper.enabled = false;
                _gpuSolver.ClearToolCollisionParams();
            }

            // 設置雙面渲染材質（切面內部顏色）
            SetupTwoSidedMaterial();
            SetupSofaUnityVisualRendering();

            _physicsReady = true;

            Debug.Log($"[SoftBody] GPU 求解器啟動 | 重力: {gravityY} m/s² | " +
                      $"子步: {numSubSteps} | 地面 Y: {groundY}" +
                      $"{(enableCutting ? " | 切割: ON" : "")}" +
                      $"{(enableGrasping ? " | 夹取: ON" : " | 夹取: OFF")}");
        }

        // ── 物理更新（FixedUpdate）────────────────────────────
        int _physicsFrame = 0;
        float _maxPosDelta = 0f;

        void FixedUpdate()
        {
            if (!_physicsReady || _data == null || pausePhysics || _gpuSolver == null) return;
            _fixedUpdatesThisRenderFrame++;
            ApplyFixedUpdateCatchUpLimit();

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
            _gpuSolver.ToolContactDistance = toolContactDistance;
            _gpuSolver.ToolContactCompliance = toolContactCompliance;
            _gpuSolver.ToolContactIterations = toolContactIterations;
            _gpuSolver.ToolContactCouplingPasses = toolContactCouplingPasses;

            // ★ 夹爪碰撞: 在 Step 之前上传平面参数到 GPU
            _lastToolUploadMs = 0f;
            _lastGripperStepMs = 0f;
            if (enableGrasping && _gripperTool != null)
            {
                try
                {
                    var toolSw = System.Diagnostics.Stopwatch.StartNew();
                    _gripperTool.UploadToolCollisionToGPU();
                    toolSw.Stop();
                    _lastToolUploadMs = (float)toolSw.Elapsed.TotalMilliseconds;
                }
                catch (System.Exception ex)
                { Debug.LogError($"[SoftBody] UploadToolCollision 异常: {ex.Message}"); }
            }

            try
            {
                var sw = System.Diagnostics.Stopwatch.StartNew();
                var partSw = System.Diagnostics.Stopwatch.StartNew();
                _gpuSolver.Step(Time.fixedDeltaTime);
                partSw.Stop();
                _lastSolveDispatchMs = (float)partSw.Elapsed.TotalMilliseconds;

                partSw.Restart();
                _gpuSolver.ReadbackPositions(_data);
                partSw.Stop();
                _lastReadbackMs = (float)partSw.Elapsed.TotalMilliseconds;
                sw.Stop();

                // 計算位置變化量（用非固定粒子）
                Vector3 posAfter = _data.Positions[diagIdx];
                float delta = (posAfter - posBefore).magnitude;
                _maxPosDelta = Mathf.Max(_maxPosDelta * 0.99f, delta);

                float ms   = (float)sw.Elapsed.TotalMilliseconds;
                _lastPhysicsTotalMs = ms;
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

            // ★ 夹爪夹取逻辑：在 Readback 之后执行（仅处理粒子捕获/释放）
            if (enableGrasping && _gripperTool != null)
            {
                try
                {
                    var gripperSw = System.Diagnostics.Stopwatch.StartNew();
                    _gripperTool.PhysicsStep();
                    gripperSw.Stop();
                    _lastGripperStepMs = (float)gripperSw.Elapsed.TotalMilliseconds;
                }
                catch (System.Exception ex)
                { Debug.LogError($"[SoftBody] GripperTool PhysicsStep 异常: {ex.Message}"); }
            }

            _simTime += Time.fixedDeltaTime;
            _physicsFrame++;

            var visualSw = System.Diagnostics.Stopwatch.StartNew();
            if (!(_sofaVisualRenderer != null &&
                  _sofaVisualRenderer.IsInitialized &&
                  hideTetSurfaceWhenSofaVisualActive))
            {
                _visualizer.Refresh();
            }
            _sofaVisualRenderer?.Refresh();
            visualSw.Stop();
            _lastVisualRefreshMs = (float)visualSw.Elapsed.TotalMilliseconds;

            if (_physicsFrame % 60 == 0)
            {
                int internalDispatchesForLog = _gpuSolver != null ? _gpuSolver.EstimatedInternalDispatchesPerFrame : 0;
                int toolDispatchesForLog = _gpuSolver != null ? _gpuSolver.EstimatedToolContactDispatchesPerFrame : 0;
                float renderFps = _fps;
                Debug.Log($"[Perf] renderFPS={renderFps:F1} toolUpload={_lastToolUploadMs:F3}ms " +
                          $"solveDispatch={_lastSolveDispatchMs:F2}ms readback={_lastReadbackMs:F2}ms " +
                          $"physicsTotal={_lastPhysicsTotalMs:F2}ms visualRefresh={_lastVisualRefreshMs:F2}ms " +
                          $"gripperStep={_lastGripperStepMs:F3}ms " +
                          $"internalDispatches={internalDispatchesForLog} toolDispatches={toolDispatchesForLog} " +
                          $"fixedPerRender={_lastFixedUpdatesPerRenderFrame} maxDt={Time.maximumDeltaTime:F3}");
            }

        }

        // ── 鍵盤控制 ─────────────────────────────────────────
        void LateUpdate()
        {
            _lastFixedUpdatesPerRenderFrame = _fixedUpdatesThisRenderFrame;
            _fixedUpdatesThisRenderFrame = 0;
        }

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

        // SofaUnity's textured liver uses a visual OBJ with authored UVs plus mapping to the tetra body.
        // This runtime surface is generated from tetra faces, so use rest-position triplanar sampling first.
        void SetupTwoSidedMaterial()
        {
            Shader shader = Shader.Find("SurgicalSim/LiverTissueGPU");
            if (shader == null)
                shader = Shader.Find("SurgicalSim/PhotorealisticLiver");
            if (shader == null)
            {
                Debug.LogWarning("[SoftBody] LiverTissueGPU not found, fallback to URP/Lit.");
                shader = Shader.Find("Universal Render Pipeline/Lit");
            }
            if (shader == null) { Debug.LogError("[SoftBody] No shader found!"); return; }

            var mat = new Material(shader) { name = "Runtime_LiverSurface" };

            // ── Albedo 纹理 ─────────────────────────────────────────────────
            Texture2D tex = liverTexture;
#if UNITY_EDITOR
            if (tex == null) tex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver2.png");
#endif
            if (tex == null) tex = LoadTextureFromAssetsPath("Texture/liver2.png");
            _resolvedLiverTexture = tex;

            // ── 法线贴图 ────────────────────────────────────────────────────
            Texture2D normTex = liverNormalMap;
#if UNITY_EDITOR
            if (normTex == null) normTex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver-texture-square_bump.png");
#endif
            if (normTex == null) normTex = LoadTextureFromAssetsPath("Texture/liver-texture-square_bump.png");

            Texture2D heightTex = liverHeightMap;
#if UNITY_EDITOR
            if (heightTex == null) heightTex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver2_height.png");
#endif
            if (heightTex == null) heightTex = LoadTextureFromAssetsPath("Texture/liver2_height.png");
            _resolvedLiverHeightTexture = heightTex;

            // ── 高光贴图 ────────────────────────────────────────────────────
            Texture2D specTex = liverSpecularMap;
#if UNITY_EDITOR
            if (specTex == null) specTex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver2_spec.png");
#endif
            if (specTex == null) specTex = LoadTextureFromAssetsPath("Texture/liver2_spec.png");
            _resolvedProject2BumpTexture = specTex;

            // 纹理诊断
            string _diagTex  = tex     != null ? (tex.name + " " + tex.width + "x" + tex.height) : "NULL! 纹理加载失败!";
            string _diagNorm = normTex != null ? normTex.name : "null";
            string _diagHeight = heightTex != null ? heightTex.name : "null";
            string _diagSpec = specTex != null ? specTex.name : "null";
            Debug.LogWarning("[SoftBody] texture check | tex=" + _diagTex + " | bump=" + _diagNorm +
                             " | height=" + _diagHeight + " | spec=" + _diagSpec);

            // ── 材质参数配置 ────────────────────────────────────────────────
            // 颜色: 深红棕，匹配 SofaUnity 生物组织色相
            Color surfaceColor = EffectiveLiverColor();
            SetColorIf(mat, "_Color",         surfaceColor);
            SetColorIf(mat, "_BaseColor",     surfaceColor);
            SetColorIf(mat, "_InteriorColor",  EffectiveCutColor());
            SetColorIf(mat, "_SSSColor",       liverSSSColor);
            SetColorIf(mat, "_SpecularColor",  liverSpecularColor);
            SetColorIf(mat, "_SpecColor",      new Color(0.20f, 0.18f, 0.16f, 1f));

            // 纹理
            if (tex != null)
            {
                SetTextureIf(mat, "_MainTex", tex);
                SetTextureIf(mat, "_BaseMap", tex);
            }
            if (normTex != null)
            {
                SetTextureIf(mat, "_NormalMap", normTex);
            }
            if (heightTex != null)
            {
                SetTextureIf(mat, "_HeightMap", heightTex);
                SetTextureIf(mat, "_ParallaxMap", heightTex);
            }
            if (specTex != null)
            {
                SetTextureIf(mat, "_SpecMap", specTex);
                SetTextureIf(mat, "_SpecGlossMap", specTex);
                mat.EnableKeyword("_SPECGLOSSMAP");
            }

            SetTextureScaleIf(mat, "_MainTex", liverTextureTiling);
            SetTextureScaleIf(mat, "_BaseMap", liverTextureTiling);
            SetTextureScaleIf(mat, "_NormalMap", liverTextureTiling);
            SetTextureScaleIf(mat, "_BumpMap", liverTextureTiling);
            SetTextureScaleIf(mat, "_HeightMap", liverTextureTiling);
            SetTextureScaleIf(mat, "_ParallaxMap", liverTextureTiling);
            SetTextureScaleIf(mat, "_SpecMap", liverTextureTiling);
            SetTextureScaleIf(mat, "_SpecGlossMap", liverTextureTiling);

            // Triplanar 参数 — 控制纹理在世界空间的缩放
            // TriplanarScale: 越大纹理越密 (SofaUnity 肝脏约 6~10 较合适)
            float effectiveTriplanarScale = Mathf.Max(liverTriplanarScale, 5.5f);
            float effectiveTextureStrength = Mathf.Max(liverTextureStrength, 0.95f);

            SetFloatIf(mat, "_TriplanarScale",  effectiveTriplanarScale);
            SetFloatIf(mat, "_TriplanarBlend",  liverTriplanarBlend > 0 ? liverTriplanarBlend : 5.0f);
            SetFloatIf(mat, "_TextureStrength", effectiveTextureStrength);
            SetFloatIf(mat, "_TextureContrast", Mathf.Max(liverTextureContrast, 1.35f));
            SetFloatIf(mat, "_TextureColorBlend", 1f);
            SetFloatIf(mat, "_UvTextureWeight", 0f);
            SetFloatIf(mat, "_AlbedoBrightness", Mathf.Min(liverAlbedoBrightness, 0.82f));
            SetFloatIf(mat, "_ProceduralNormalStrength", Mathf.Min(liverProceduralNormalStrength, 0.05f));
            SetFloatIf(mat, "_Wetness", Mathf.Min(liverWetness, 0.18f));
            SetFloatIf(mat, "_MicroMottleStrength", Mathf.Min(liverMicroMottleStrength, 0.08f));
            SetFloatIf(mat, "_MicroMottleScale", liverMicroMottleScale);
            SetFloatIf(mat, "_VeinStrength", liverVeinStrength);
            SetFloatIf(mat, "_VeinScale", liverVeinScale);
            SetFloatIf(mat, "_DebugTextureOnly", liverDebugTextureOnly);

            // SofaUnity's material is textured first and only mildly glossy. Too much smoothness washes the texture out.
            SetFloatIf(mat, "_Roughness",        Mathf.Max(liverRoughness, 0.70f));
            SetFloatIf(mat, "_Smoothness",       0.10f);
            SetFloatIf(mat, "_Glossiness",       0.10f);
            SetFloatIf(mat, "_Metallic",         0f);
            SetFloatIf(mat, "_SpecularStrength", Mathf.Min(liverSpecularStrength, 0.18f));
            SetFloatIf(mat, "_SpecMapStrength",  0.65f);
            SetFloatIf(mat, "_NormalStrength",   Mathf.Min(Mathf.Max(liverNormalStrength, 0.04f), 0.12f));

            // SSS 参数
            SetFloatIf(mat, "_SSSStrength", Mathf.Min(liverSSSStrength, 0.06f));
            SetFloatIf(mat, "_SSSDirect",   Mathf.Min(liverSSSBacklight, 0.04f));
            SetFloatIf(mat, "_SSSPower",    liverSSSPower);
            SetFloatIf(mat, "_SSSWrap",     liverSSSWrap);

            // Fresnel
            SetFloatIf(mat, "_FresnelStrength", Mathf.Min(liverFresnelStrength, 0.08f));
            SetFloatIf(mat, "_FresnelPow",      liverFresnelPow);
            ConfigureOpaqueTwoSided(mat);
            mat.doubleSidedGI = true;

            // 应用材质
            if (_visualizer != null)
            {
                _visualizer.SetSurfaceMaterial(mat);
                Material cutMat = CreateCutSurfaceMaterial();
                if (cutMat != null)
                    _visualizer.SetCutSurfaceMaterial(cutMat);
            }
            else
            {
                var mr = GetComponent<MeshRenderer>();
                if (mr != null) mr.sharedMaterial = mat;
            }

            Debug.Log("[SoftBody] SofaUnity-style triplanar liver material applied | Shader=" + shader.name +
                      " | Albedo=" + (tex != null ? tex.name : "MISSING!") +
                      " | Height=" + (heightTex != null ? heightTex.name : "none") +
                      " | Spec=" + (specTex != null ? specTex.name : "none") +
                      " | UVWeight=0.00" +
                      " | TriScale=" + effectiveTriplanarScale.ToString("F2"));
        }

        void SetupSofaUnityVisualRendering()
        {
            if (_visualizer == null || _data == null) return;

            if (!useSofaUnityVisualMesh)
            {
                _visualizer.SetMainSurfaceVisible(true);
                return;
            }

            if (_sofaVisualRenderer == null)
                _sofaVisualRenderer = GetComponent<SofaUnityVisualLiverRenderer>();
            if (_sofaVisualRenderer == null)
                _sofaVisualRenderer = gameObject.AddComponent<SofaUnityVisualLiverRenderer>();

            Texture2D tex = _resolvedLiverTexture != null ? _resolvedLiverTexture : liverTexture;
            Texture2D project2BumpTex = _resolvedProject2BumpTexture;
            Texture2D project2HeightTex = _resolvedLiverHeightTexture;

            if (project2BumpTex == null)
            {
#if UNITY_EDITOR
                project2BumpTex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver2_spec.png");
#endif
                if (project2BumpTex == null) project2BumpTex = LoadTextureFromAssetsPath("Texture/liver2_spec.png");
            }

            if (project2HeightTex == null)
            {
#if UNITY_EDITOR
                project2HeightTex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver2_height.png");
#endif
                if (project2HeightTex == null) project2HeightTex = LoadTextureFromAssetsPath("Texture/liver2_height.png");
            }

            Material visualMaterial = sofaUnityVisualMaterial;
            if (visualMaterial == null && useProject2Liver2Material)
                visualMaterial = SofaUnityVisualLiverRenderer.CreateProject2Liver2Material(
                    tex,
                    project2BumpTex,
                    project2HeightTex);

            Vector2 visualUvTiling = useProject2Liver2Material ? Vector2.one : sofaUnityVisualUvTiling;
            bool initialized = _sofaVisualRenderer.Init(
                _data,
                tex,
                visualMaterial,
                sofaUnityVisualObj,
                sofaUnityVisualObjResource,
                sofaUnityVisualFitPadding,
                useTetSurfaceForSofaUnityVisual,
                visualUvTiling);

            _visualizer.SetMainSurfaceVisible(!(initialized && hideTetSurfaceWhenSofaVisualActive));

            if (initialized)
            {
                string source = useTetSurfaceForSofaUnityVisual
                    ? "current liver3-HD tet surface"
                    : (sofaUnityVisualObj != null ? sofaUnityVisualObj.name : sofaUnityVisualObjResource);
                Debug.Log("[SoftBody] SofaUnity visual renderer applied | Source=" +
                          source +
                          " | Material=" + (visualMaterial != null ? visualMaterial.name : "fallback") +
                          " | Albedo=" + (tex != null ? tex.name : "MISSING!") +
                          " | Bump=" + (project2BumpTex != null ? project2BumpTex.name : "none") +
                          " | Height=" + (project2HeightTex != null ? project2HeightTex.name : "none"));
            }
        }

        Material CreateCutSurfaceMaterial()
        {
            Shader shader = Shader.Find("SurgicalSim/LiverCutSurfaceGPU");
            if (shader == null)
                shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                shader = Shader.Find("Standard");
            if (shader == null)
                return null;

            var mat = new Material(shader)
            {
                name = "Runtime_LiverCutSurfaceGPU"
            };
            ConfigureCutSurfaceMaterial(mat);
            return mat;
        }

        void ConfigureCutSurfaceMaterial(Material mat)
        {
            if (mat == null) return;

            Color cutColor = EffectiveCutColor();

            SetColorIf(mat, "_CutColor", cutColor);
            SetColorIf(mat, "_InteriorColor", cutColor);
            SetColorIf(mat, "_Color", cutColor);
            SetColorIf(mat, "_BaseColor", cutColor);
            SetFloatIf(mat, "_Roughness", Mathf.Clamp01(liverRoughness * 0.75f));
            SetFloatIf(mat, "_Smoothness", Mathf.Clamp01(1f - liverRoughness * 0.75f));
            SetFloatIf(mat, "_Glossiness", Mathf.Clamp01(1f - liverRoughness * 0.75f));
            SetFloatIf(mat, "_Metallic", 0f);
            SetFloatIf(mat, "_Wetness", Mathf.Clamp01(liverWetness + 0.2f));
            SetFloatIf(mat, "_SpecularStrength", Mathf.Clamp(liverSpecularStrength + 0.25f, 0f, 2f));
            SetColorIf(mat, "_SpecularColor", liverSpecularColor);
            SetColorIf(mat, "_SpecColor", new Color(0.04f, 0.03f, 0.025f, 1f));
            SetColorIf(mat, "_SSSColor", liverSSSColor);
            SetFloatIf(mat, "_SSSStrength", Mathf.Clamp01(liverSSSStrength * 0.75f));
            SetFloatIf(mat, "_SSSWrap", Mathf.Clamp01(liverSSSWrap * 0.75f));
            SetFloatIf(mat, "_FiberIntensity", 0.34f);
            SetFloatIf(mat, "_FiberScale", 36f);
            SetFloatIf(mat, "_RimDarkening", 0.18f);
            SetFloatIf(mat, "_TextureStrength", 0f);

            ConfigureOpaqueTwoSided(mat);
            mat.doubleSidedGI = true;
        }

        Color EffectiveLiverColor()
        {
            Color c = liverColor;
            float maxChannel = Mathf.Max(c.r, Mathf.Max(c.g, c.b));
            float minChannel = Mathf.Min(c.r, Mathf.Min(c.g, c.b));
            float saturation = maxChannel <= 1e-5f ? 0f : (maxChannel - minChannel) / maxChannel;
            if (maxChannel > 0.82f || saturation < 0.35f || c.g > c.r * 0.40f || c.b > c.r * 0.32f)
                c = new Color(0.62f, 0.18f, 0.10f, liverColor.a);
            else
                c.a = liverColor.a;
            return c;
        }

        Color EffectiveCutColor()
        {
            Color.RGBToHSV(interiorColor, out float hue, out float saturation, out float value);
            if (saturation < 0.12f || value > 0.5f)
                return new Color(0.30f, 0.025f, 0.018f, interiorColor.a);
            return interiorColor;
        }

        static void SetColorIf(Material mat, string property, Color value)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetColor(property, value);
        }

        static void SetFloatIf(Material mat, string property, float value)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetFloat(property, value);
        }

        static void SetTextureIf(Material mat, string property, Texture texture)
        {
            if (mat != null && texture != null && mat.HasProperty(property))
                mat.SetTexture(property, texture);
        }

        static void SetTextureScaleIf(Material mat, string property, Vector2 scale)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetTextureScale(property, scale);
        }

        static void SetTextureOffsetIf(Material mat, string property, Vector2 offset)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetTextureOffset(property, offset);
        }

        static void ConfigureOpaqueTwoSided(Material mat)
        {
            if (mat == null) return;

            mat.SetOverrideTag("RenderType", "Opaque");
            mat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Geometry;
            SetFloatIf(mat, "_Surface", 0f);
            SetFloatIf(mat, "_Blend", 0f);
            SetFloatIf(mat, "_SrcBlend", (float)UnityEngine.Rendering.BlendMode.One);
            SetFloatIf(mat, "_DstBlend", (float)UnityEngine.Rendering.BlendMode.Zero);
            SetFloatIf(mat, "_ZWrite", 1f);
            SetFloatIf(mat, "_AlphaClip", 0f);
            SetFloatIf(mat, "_Cull", 0f);

            mat.DisableKeyword("_SURFACE_TYPE_TRANSPARENT");
            mat.DisableKeyword("_ALPHATEST_ON");
            mat.DisableKeyword("_ALPHAPREMULTIPLY_ON");
        }

/* Legacy material setup disabled after the LiverTissueGPU migration.
        void SetupTwoSidedMaterial_LegacyDisabled()
        {
            // 优先使用新的真实感着色器，回退到旧版
            var shader = Shader.Find("SurgicalSim/PhotorealisticLiver");
            if (shader == null)
            {
                Debug.LogWarning("[SoftBody] 未找到 PhotorealisticLiver 着色器，尝试 TwoSidedLiver 回退");
                shader = Shader.Find("SurgicalSim/TwoSidedLiver");
            }
            if (shader == null)
            {
                Debug.LogWarning("[SoftBody] 未找到任何肝脏着色器，使用默认材质");
                return;
            }

            var mat = new Material(shader);

            // ── 基础颜色 & 纹理 ──────────────────────────────────
            mat.SetColor("_Color",          liverColor);
            mat.SetColor("_InteriorColor",  interiorColor);
            mat.SetFloat("_TextureStrength", liverTextureStrength);
            mat.SetFloat("_TriplanarScale",  liverTriplanarScale);
            mat.SetFloat("_TriplanarBlend",  liverTriplanarBlend);
            mat.SetTextureScale("_MainTex",  liverTextureTiling);

            // 自动查找 Albedo 纹理
            Texture2D tex = liverTexture;
#if UNITY_EDITOR
            if (tex == null)
                tex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver2.png");
#endif
            if (tex == null)
                tex = LoadTextureFromAssetsPath("Texture/liver2.png");
            if (tex != null)
                mat.SetTexture("_MainTex", tex);

            // ── 法线贴图 ─────────────────────────────────────────
            Texture2D normTex = liverNormalMap;
#if UNITY_EDITOR
            if (normTex == null)
                normTex = UnityEditor.AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Texture/liver_normal.png");
#endif
            if (normTex == null)
                normTex = LoadTextureFromAssetsPath("Texture/liver_normal.png");
            if (normTex != null)
                mat.SetTexture("_NormalMap", normTex);
            mat.SetFloat("_NormalStrength", liverNormalStrength);

            // ── PBR 高光 ─────────────────────────────────────────
            mat.SetFloat("_Roughness",         liverRoughness);
            mat.SetFloat("_SpecularStrength",   liverSpecularStrength);
            mat.SetColor("_SpecularColor",      liverSpecularColor);

            // ── SSS ──────────────────────────────────────────────
            mat.SetColor("_SSSColor",    liverSSSColor);
            mat.SetFloat("_SSSStrength", liverSSSStrength);
            mat.SetFloat("_SSSDirect",   liverSSSBacklight);
            mat.SetFloat("_SSSPower",    liverSSSPower);
            mat.SetFloat("_SSSWrap",     liverSSSWrap);

            // ── Fresnel ──────────────────────────────────────────
            mat.SetFloat("_FresnelStrength", liverFresnelStrength);
            mat.SetFloat("_FresnelPow",      liverFresnelPow);

            var renderer = _visualizer.GetComponent<MeshRenderer>();
            if (renderer != null)
            {
                _visualizer.surfaceMaterial = mat;
                renderer.material = mat;
                Debug.Log($"[SoftBody] ★ PhotorealisticLiver 材质已设置 | " +
                          $"Albedo={(tex != null ? tex.name : "none")} | " +
                          $"Normal={(normTex != null ? normTex.name : "none")} | " +
                          $"Roughness={liverRoughness:F2} | SSSStrength={liverSSSStrength:F2}");
            }
        }

*/

        Texture2D LoadTextureFromAssetsPath(string relativePath)
        {
            string path = Path.Combine(Application.dataPath, relativePath);
            if (!File.Exists(path)) return null;

            byte[] bytes = File.ReadAllBytes(path);
            var tex = new Texture2D(2, 2, TextureFormat.RGBA32, true);
            tex.name = Path.GetFileNameWithoutExtension(path);
            if (!tex.LoadImage(bytes))
            {
                Destroy(tex);
                return null;
            }
            tex.wrapMode = TextureWrapMode.Repeat;
            tex.filterMode = FilterMode.Bilinear;
            return tex;
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

            // 重新初始化夹爪工具
            if (enableGrasping && _gripperTool != null)
                _gripperTool.Init(_data, _gpuSolver, _visualizer);

            // 恢复原始表面三角形
            var mf = _visualizer.GetComponent<MeshFilter>();
            if (mf != null && mf.mesh != null)
            {
                mf.mesh.triangles = new int[0];
                mf.mesh.triangles = _data.SurfaceTriIds;
                mf.mesh.RecalculateNormals();
            }


            _visualizer.Refresh();
            _sofaVisualRenderer?.Refresh();
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
            int cutSeparated = _cuttingTool != null ? _cuttingTool.LastSeparatedVertexCount : 0;
            int cutRemainingShared = _cuttingTool != null ? _cuttingTool.LastRemainingSharedVertexCount : 0;
            float cutQueryMs = _cuttingTool != null ? _cuttingTool.LastCutQueryMs : 0f;
            float cutSplitMs = _cuttingTool != null ? _cuttingTool.LastCutSplitMs : 0f;
            float cutSeparateMs = _cuttingTool != null ? _cuttingTool.LastCutSeparateMs : 0f;
            float surfaceMs = _cuttingTool != null ? _cuttingTool.LastSurfaceUpdateMs : 0f;
            float gpuFlushMs = _cuttingTool != null ? _cuttingTool.LastGpuFlushMs : 0f;

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
            info += $"\nFixed/Frame: {_lastFixedUpdatesPerRenderFrame} | MaxDt: {Time.maximumDeltaTime:F3}";
            style.richText = true;

            GUI.Box(new Rect(10, 10, 390, 240), info, style);

            string cutDebug =
                $"刀状态: {toolPressed}\n" +
                $"move: {cutMoveDist:E2}\n" +
                $"cand/hit: {cutCandidates}/{cutHits}\n" +
                $"sep/rem: {cutSeparated}/{cutRemainingShared}\n" +
                $"cut ms q/s/p: {cutQueryMs:F1}/{cutSplitMs:F1}/{cutSeparateMs:F1}\n" +
                $"surf/gpu: {surfaceMs:F1}/{gpuFlushMs:F1} ms\n" +
                $"reason: {cutReason}";
            GUI.Box(new Rect(10, 255, 230, 145), cutDebug, style);
        }
    }
}
