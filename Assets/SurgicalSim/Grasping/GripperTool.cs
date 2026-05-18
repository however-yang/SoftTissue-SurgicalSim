// GripperTool.cs — Da Vinci 风格夹爪工具（真实 OBJ 模型版）
// 使用 3 个视觉 OBJ + 3 个碰撞 OBJ
// Phase 1: 戳压形变 (Poking) — 碰撞网格三角形 vs 软体粒子
// Phase 2: 摩擦夹取 (Grasping) — InvMass 锁定法
//
// 键盘控制：F/H→X, G/B→Y, V/N→Z, 1/3→旋转, 0→开合
// 与切割工具完全独立运行，可同时使用。

using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Grasping
{
    public class GripperTool : MonoBehaviour
    {
        const float FourXModelScale = 0.01f;
        const float FourXCollisionMargin = FourXModelScale * 1.5f;
        const float FourXJawCapsuleRadius = FourXModelScale * 3.2f;
        const float FourXShaftCapsuleRadius = FourXModelScale * 2.8f;

        [Header("OBJ 模型文件")]
        public string jawUpVisual   = "Haptic_grasper_jaws_up.obj";
        public string jawDownVisual = "Haptic_grasper_jaws_down.obj";
        public string shaftVisual   = "Haptic_grasper_shaft.obj";
        public string jawUpCol      = "Haptic_grasper_jaws_up_collision.obj";
        public string jawDownCol    = "Haptic_grasper_jaws_down_collision.obj";
        public string shaftCol      = "Haptic_grasper_shaft_collision.obj";

        [Header("模型参数")]
        [Tooltip("OBJ unit to world scale. Current preset keeps the enlarged visual gripper at 0.01.")]
        public float modelScale = FourXModelScale;

        [Tooltip("Apply the calibrated enlarged visual/collision gripper preset at runtime.")]
        public bool useFourXGripperPreset = true;

        [Header("物理参数")]
        [Tooltip("碰撞 AABB 外扩边距 (m)")]
        public float collisionMargin = FourXCollisionMargin;

        [Tooltip("Jaw capsule contact radius (m). Kept close to the visual jaw thickness to avoid oversized dents.")]
        public float capsuleRadius = FourXJawCapsuleRadius;

        [Tooltip("杆身胶囊体半径 (m)，防止杆身穿模")]
        public float shaftRadius = FourXShaftCapsuleRadius;

        [Tooltip("显示碰撞胶囊体 Gizmo")]
        public bool showCapsuleGizmo = true;

        [Header("开合")]
        [Tooltip("最大张开角度 (度)")]
        public float maxOpenAngle = 25f;
        [Tooltip("开合角速度 (度/秒)")]
        public float openCloseSpeed = 60f;

        [Header("控制")]
        public float moveSpeed = 0.3f;
        public float rotateSpeed = 60f;
        public float boostMultiplier = 3.0f;

        [Header("可视化")]
        public Color toolColor = new Color(0.7f, 0.7f, 0.78f, 1f);

        // ── 公开状态 ─────────────────────────────────────────
        public bool IsGrasping { get; private set; }
        public float CurrentAngle => _currentAngle;

        // ── 私有状态 ─────────────────────────────────────────
        TetMeshData       _data;
        XPBDSolverGPU     _solver;
        TetMeshVisualizer _visualizer;

        Vector3 _toolPos;
        float   _toolRotY;       // 绕 Y 轴旋转角度
        float   _currentAngle;   // 当前开合角度 (0=闭合, maxOpenAngle=全开)
        bool    _wantClose;
        bool    _initialized;

        // 视觉 GameObjects
        GameObject _jawUpObj, _jawDownObj, _shaftObj;

        // 碰撞网格数据 (本地坐标，已缩放)
        Vector3[] _colVertsUp, _colVertsDown, _colVertsShaft;
        int[]     _colTrisUp,  _colTrisDown,  _colTrisShaft;

        // 铰接参数 (从模型推断)
        // 上下颚绕 Z 轴方向的铰接点旋转
        // 铰接点大约在 Y=9.0, Z=-25.5 (OBJ 坐标)
        Vector3 _pivotLocal; // 缩放后的铰接点 (本地坐标)
        Vector3 _tipLocal;   // 缩放后的尖端 (本地坐标)

        // 胶囊体世界坐标 (用于 Gizmo 可视化)
        Vector3 _dbgCap0A, _dbgCap0B, _dbgCap1A, _dbgCap1B;
        float   _dbgCapR;
        Vector3 _dbgBBoxMin, _dbgBBoxMax;
        bool _hasPrevCapsules;
        Vector3 _prevCap0A, _prevCap0B, _prevCap1A, _prevCap1B, _prevCap2A, _prevCap2B;

        // 夹取状态
        struct GraspedParticle
        {
            public int index;
            public float originalInvMass;
            public Vector3 localOffset; // 相对夹爪中心的偏移
        }
        readonly List<GraspedParticle> _graspedParticles = new List<GraspedParticle>();

        // ══════════════════════════════════════════════════════
        // 初始化
        // ══════════════════════════════════════════════════════
        public void Init(TetMeshData data, XPBDSolverGPU solver, TetMeshVisualizer visualizer)
        {
            _data = data;
            _solver = solver;
            _visualizer = visualizer;

            ApplyGripperScalePreset();

            _toolPos = new Vector3(0.15f, 0.5f, 0f);
            _toolRotY = 0f;
            _currentAngle = maxOpenAngle; // 初始张开
            _wantClose = false;
            IsGrasping = false;
            _graspedParticles.Clear();
            _uploadFrame = 0; // 重置诊断计数器

            // 铰接点 (OBJ 坐标 → 缩放后)
            _pivotLocal = new Vector3(0f, 9.0f, -25.5f) * modelScale;
            // 夹爪尖端 (OBJ 坐标 → 缩放后)
            _tipLocal = new Vector3(0f, 9.0f, -43.0f) * modelScale;
            // 杆身尾端 (手柄方向，延伸足够长以覆盖整个杆身)
            _shaftEndLocal = new Vector3(0f, 9.0f, 307.0f) * modelScale;
            _shaftTipLocal = _pivotLocal;
            _prevToolPos = _toolPos;
            _hasPrevCapsules = false;

            LoadModels();
            LoadCollisionMeshes();
            _initialized = true;

            Debug.Log($"[GripperTool] Init | scale={modelScale} | 3-capsule collision");
        }

        void ApplyGripperScalePreset()
        {
            if (!useFourXGripperPreset) return;

            modelScale = FourXModelScale;
            collisionMargin = FourXCollisionMargin;
            capsuleRadius = FourXJawCapsuleRadius;
            shaftRadius = FourXShaftCapsuleRadius;
        }

        // ══════════════════════════════════════════════════════
        // Update — 键盘输入 + 可视化
        // ══════════════════════════════════════════════════════
        void Update()
        {
            if (!_initialized) return;

            // ── 移动（FGHVBN）──────────────────────────────
            Vector3 move = Vector3.zero;
            if (Input.GetKey(KeyCode.H)) move.x += 1f;
            if (Input.GetKey(KeyCode.F)) move.x -= 1f;
            if (Input.GetKey(KeyCode.G)) move.y += 1f;
            if (Input.GetKey(KeyCode.B)) move.y -= 1f;
            if (Input.GetKey(KeyCode.N)) move.z += 1f;
            if (Input.GetKey(KeyCode.V)) move.z -= 1f;

            bool boost = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            float speed = boost ? moveSpeed * boostMultiplier : moveSpeed;
            if (move.sqrMagnitude > 1e-8f)
            {
                Vector3 desiredMove = move.normalized * speed * Time.deltaTime;
                // ★ 速度钒制: 每帧最大移动 capsuleRadius * 0.5，防止隧穿
                // 参考 SOFA alarmDistance 概念
                float maxMove = capsuleRadius * 0.5f;
                if (desiredMove.magnitude > maxMove)
                    desiredMove = desiredMove.normalized * maxMove;
                _toolPos += desiredMove;
            }

            // ── 旋转（Numpad 1/3）──────────────────────────
            if (Input.GetKey(KeyCode.Keypad1)) _toolRotY -= rotateSpeed * Time.deltaTime;
            if (Input.GetKey(KeyCode.Keypad3)) _toolRotY += rotateSpeed * Time.deltaTime;

            // ── 开合（Numpad 0）──────────────────────────
            if (Input.GetKeyDown(KeyCode.Keypad0))
            {
                _wantClose = !_wantClose;
                Debug.Log($"[GripperTool] {(_wantClose ? "闭合" : "张开")}");
            }

            float targetAngle = _wantClose ? 0f : maxOpenAngle;
            _currentAngle = Mathf.MoveTowards(_currentAngle, targetAngle,
                                               openCloseSpeed * Time.deltaTime);

            UpdateVisual();
        }

        // ══════════════════════════════════════════════════════
        // PhysicsStep — 在 FixedUpdate 中被 SoftBody 调用
        // 碰撞已由 GPU CSToolCollision 处理，这里只做夹取
        // ══════════════════════════════════════════════════════
        public void PhysicsStep()
        {
            if (!_initialized || _data == null || _solver == null) return;

            Transform tf = _visualizer != null ? _visualizer.transform : transform;

            // 夹取逻辑（碰撞已由 GPU 处理）
            bool isFullyClosed = _wantClose && _currentAngle <= 1f;

            if (isFullyClosed && !IsGrasping)
            {
                CaptureParticles(tf);
                IsGrasping = true;
            }
            else if (IsGrasping && isFullyClosed)
            {
                UpdateGraspedParticles(tf);
            }
            else if (IsGrasping && !_wantClose)
            {
                ReleaseParticles();
                IsGrasping = false;
            }
        }

        // ══════════════════════════════════════════════════════
        // GPU 碰撞: 3个胶囊体 (上颚 + 下颚 + 杆身)
        // 参考 SOFA: proximity/contact pipeline + CCD
        // ══════════════════════════════════════════════════════
        int _uploadFrame = 0;
        Vector3 _prevToolPos;
        Vector3 _shaftEndLocal; // 杆身尾端(手柄方向)

        Vector3 _shaftTipLocal; // shaft tip/hinge end in scaled OBJ local space

        public void UploadToolCollisionToGPU()
        {
            if (!_initialized || _solver == null) return;

            Quaternion toolRot = Quaternion.Euler(0, _toolRotY, 0);

            // ── Capsule 0: 上颚 ──
            Quaternion upperRot = Quaternion.AngleAxis(_currentAngle, Vector3.right);
            Vector3 upperOffset = upperRot * (Vector3.up * capsuleRadius);
            Vector3 cap0A = toolRot * (upperRot * (_pivotLocal + upperOffset - _pivotLocal) + _pivotLocal) + _toolPos;
            Vector3 cap0B = toolRot * (upperRot * (_tipLocal + upperOffset - _pivotLocal) + _pivotLocal) + _toolPos;

            // ── Capsule 1: 下颚 ──
            Quaternion lowerRot = Quaternion.AngleAxis(-_currentAngle, Vector3.right);
            Vector3 lowerOffset = lowerRot * (-Vector3.up * capsuleRadius);
            Vector3 cap1A = toolRot * (lowerRot * (_pivotLocal + lowerOffset - _pivotLocal) + _pivotLocal) + _toolPos;
            Vector3 cap1B = toolRot * (lowerRot * (_tipLocal + lowerOffset - _pivotLocal) + _pivotLocal) + _toolPos;

            // ── Capsule 2: 杆身 (shaft) ──
            // 从杆身尾端到铰接点，不随开合角度旋转
            Vector3 cap2A = toolRot * _shaftEndLocal + _toolPos;
            Vector3 cap2B = toolRot * _shaftTipLocal + _toolPos;

            Vector3 prevCap0A = _hasPrevCapsules ? _prevCap0A : cap0A;
            Vector3 prevCap0B = _hasPrevCapsules ? _prevCap0B : cap0B;
            Vector3 prevCap1A = _hasPrevCapsules ? _prevCap1A : cap1A;
            Vector3 prevCap1B = _hasPrevCapsules ? _prevCap1B : cap1B;
            Vector3 prevCap2A = _hasPrevCapsules ? _prevCap2A : cap2A;
            Vector3 prevCap2B = _hasPrevCapsules ? _prevCap2B : cap2B;

            _solver.SetCapsuleCollisionParams(
                cap0A, cap0B, capsuleRadius,
                cap1A, cap1B, capsuleRadius,
                cap2A, cap2B, shaftRadius,
                prevCap0A, prevCap0B,
                prevCap1A, prevCap1B,
                prevCap2A, prevCap2B,
                3);

            // Gizmo
            _dbgCap0A = cap0A; _dbgCap0B = cap0B;
            _dbgCap1A = cap1A; _dbgCap1B = cap1B;
            _dbgCapR  = capsuleRadius;
            float margin = collisionMargin + capsuleRadius;
            _dbgBBoxMin = Vector3.Min(Vector3.Min(Vector3.Min(cap0A, cap0B), Vector3.Min(cap1A, cap1B)), Vector3.Min(cap2A, cap2B)) - Vector3.one * margin;
            _dbgBBoxMax = Vector3.Max(Vector3.Max(Vector3.Max(cap0A, cap0B), Vector3.Max(cap1A, cap1B)), Vector3.Max(cap2A, cap2B)) + Vector3.one * margin;

            if (_uploadFrame < 3)
            {
                Debug.Log($"[GripperTool] 3胶囊碰撞 F{_uploadFrame}: " +
                    $"Jaw0=[{cap0A:F3}..{cap0B:F3}] Jaw1=[{cap1A:F3}..{cap1B:F3}] " +
                    $"Shaft=[{cap2A:F3}..{cap2B:F3}] JawR={capsuleRadius} ShaftR={shaftRadius}");
                _uploadFrame++;
            }
            _prevCap0A = cap0A; _prevCap0B = cap0B;
            _prevCap1A = cap1A; _prevCap1B = cap1B;
            _prevCap2A = cap2A; _prevCap2B = cap2B;
            _hasPrevCapsules = true;
            _prevToolPos = _toolPos;
        }


        // ══════════════════════════════════════════════════════
        // Phase 2: 夹取
        // ══════════════════════════════════════════════════════
        void CaptureParticles(Transform meshTf)
        {
            _graspedParticles.Clear();

            // 夹爪尖端区域 (OBJ Z ≈ -43 → 世界坐标)
            // 上下颚闭合时，夹住它们之间的粒子
            var worldVertsUp   = TransformCollisionVerts(_colVertsUp, true);
            var worldVertsDown = TransformCollisionVerts(_colVertsDown, false);

            // 用两组碰撞顶点的 AABB 交集作为夹取区域
            Vector3 bboxMin = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 bboxMax = new Vector3(float.MinValue, float.MinValue, float.MinValue);
            foreach (var v in worldVertsUp)
            { bboxMin = Vector3.Min(bboxMin, v); bboxMax = Vector3.Max(bboxMax, v); }
            foreach (var v in worldVertsDown)
            { bboxMin = Vector3.Min(bboxMin, v); bboxMax = Vector3.Max(bboxMax, v); }

            // 缩小一点 bbox 只取核心区域
            Vector3 shrink = (bboxMax - bboxMin) * 0.1f;
            bboxMin += shrink;
            bboxMax -= shrink;

            // 转换到 mesh local space
            Vector3 localMin = meshTf.InverseTransformPoint(bboxMin);
            Vector3 localMax = meshTf.InverseTransformPoint(bboxMax);
            // 确保 min < max
            Vector3 realMin = Vector3.Min(localMin, localMax);
            Vector3 realMax = Vector3.Max(localMin, localMax);

            Vector3 jawCenter = meshTf.InverseTransformPoint((bboxMin + bboxMax) * 0.5f);

            int captured = 0;
            for (int i = 0; i < _data.NumParticles; i++)
            {
                if (_data.InvMass[i] == 0f) continue;
                Vector3 p = _data.Positions[i];
                if (p.x >= realMin.x && p.x <= realMax.x &&
                    p.y >= realMin.y && p.y <= realMax.y &&
                    p.z >= realMin.z && p.z <= realMax.z)
                {
                    _graspedParticles.Add(new GraspedParticle
                    {
                        index = i,
                        originalInvMass = _data.InvMass[i],
                        localOffset = p - jawCenter
                    });
                    _data.InvMass[i] = 0f;
                    captured++;
                }
            }

            if (captured > 0)
            {
                _solver.UploadInvMass(_data);
                Debug.Log($"[GripperTool] 夹取了 {captured} 个粒子");
            }
        }

        void UpdateGraspedParticles(Transform meshTf)
        {
            if (_graspedParticles.Count == 0) return;

            // 当前夹爪尖端中心 (世界 → local)
            Vector3 tipWorld = GetJawTipCenter();
            Vector3 tipLocal = meshTf.InverseTransformPoint(tipWorld);

            foreach (var gp in _graspedParticles)
            {
                _data.Positions[gp.index] = tipLocal + gp.localOffset;
                _data.Velocities[gp.index] = Vector3.zero;
            }
            _solver.UploadPositionsAndVelocities(_data);
        }

        void ReleaseParticles()
        {
            foreach (var gp in _graspedParticles)
            {
                _data.InvMass[gp.index] = gp.originalInvMass;
                _data.Velocities[gp.index] *= 0.1f;
            }
            int n = _graspedParticles.Count;
            _graspedParticles.Clear();
            _solver.UploadInvMass(_data);
            _solver.UploadPositionsAndVelocities(_data);
            Debug.Log($"[GripperTool] 释放了 {n} 个粒子");
        }

        // ══════════════════════════════════════════════════════
        // 坐标变换
        // ══════════════════════════════════════════════════════
        /// <summary>将碰撞网格顶点变换到世界坐标（含开合旋转）</summary>
        Vector3[] TransformCollisionVerts(Vector3[] localVerts, bool isUpperJaw)
        {
            var result = new Vector3[localVerts.Length];
            // 开合旋转角度 (上颚向上旋转，下颚向下旋转)
            float jawAngle = isUpperJaw ? _currentAngle : -_currentAngle;
            Quaternion jawRot = Quaternion.AngleAxis(jawAngle, Vector3.right);
            Quaternion toolRot = Quaternion.Euler(0, _toolRotY, 0);

            for (int i = 0; i < localVerts.Length; i++)
            {
                // 1. 相对铰接点旋转（开合）
                Vector3 v = localVerts[i] - _pivotLocal;
                v = jawRot * v;
                v += _pivotLocal;

                // 2. 工具整体旋转
                v = toolRot * v;

                // 3. 平移到世界位置
                result[i] = v + _toolPos;
            }
            return result;
        }

        Vector3 GetJawTipCenter()
        {
            Quaternion toolRot = Quaternion.Euler(0, _toolRotY, 0);
            return toolRot * _tipLocal + _toolPos;
        }

        /// <summary>CPU 端 Point-to-Capsule 距离（用于诊断）</summary>
        static float PointCapsuleDist(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float ab2 = Vector3.Dot(ab, ab);
            float t = ab2 < 1e-10f ? 0f : Mathf.Clamp01(Vector3.Dot(p - a, ab) / ab2);
            Vector3 closest = a + t * ab;
            return (p - closest).magnitude;
        }

        // ══════════════════════════════════════════════════════
        // OBJ 加载
        // ══════════════════════════════════════════════════════
        void LoadModels()
        {
            // 清除旧的
            if (_jawUpObj) Destroy(_jawUpObj);
            if (_jawDownObj) Destroy(_jawDownObj);
            if (_shaftObj) Destroy(_shaftObj);

            var mat = new Material(
                Shader.Find("Universal Render Pipeline/Lit") ??
                Shader.Find("Standard"));
            mat.color = toolColor;

            _jawUpObj   = CreateMeshObj("GripperJawUp",   jawUpVisual,   mat);
            _jawDownObj = CreateMeshObj("GripperJawDown", jawDownVisual, mat);
            _shaftObj   = CreateMeshObj("GripperShaft",   shaftVisual,   mat);
        }

        void LoadCollisionMeshes()
        {
            string dir = Application.streamingAssetsPath;

            LoadOBJData(Path.Combine(dir, jawUpCol),   out _colVertsUp,   out _colTrisUp);
            LoadOBJData(Path.Combine(dir, jawDownCol), out _colVertsDown, out _colTrisDown);
            LoadOBJData(Path.Combine(dir, shaftCol),   out _colVertsShaft, out _colTrisShaft);

            // 缩放碰撞顶点到世界单位
            if (_colVertsUp != null)
                for (int i = 0; i < _colVertsUp.Length; i++)
                    _colVertsUp[i] *= modelScale;
            if (_colVertsDown != null)
                for (int i = 0; i < _colVertsDown.Length; i++)
                    _colVertsDown[i] *= modelScale;
            if (_colVertsShaft != null)
                for (int i = 0; i < _colVertsShaft.Length; i++)
                    _colVertsShaft[i] *= modelScale;

            Debug.Log($"[GripperTool] 碰撞网格: Up {_colVertsUp?.Length}V/{_colTrisUp?.Length/3}F " +
                      $"| Down {_colVertsDown?.Length}V/{_colTrisDown?.Length/3}F " +
                      $"| Shaft {_colVertsShaft?.Length}V/{_colTrisShaft?.Length/3}F");

            FitShaftCapsuleFromCollisionMesh();
        }

        void FitShaftCapsuleFromCollisionMesh()
        {
            if (_colVertsShaft == null || _colVertsShaft.Length == 0)
                return;

            Vector3 min = _colVertsShaft[0];
            Vector3 max = _colVertsShaft[0];
            for (int i = 1; i < _colVertsShaft.Length; i++)
            {
                min = Vector3.Min(min, _colVertsShaft[i]);
                max = Vector3.Max(max, _colVertsShaft[i]);
            }

            float centerX = (min.x + max.x) * 0.5f;
            float centerY = (min.y + max.y) * 0.5f;
            _shaftEndLocal = new Vector3(centerX, centerY, max.z);
            _shaftTipLocal = new Vector3(centerX, centerY, min.z);

            Debug.Log($"[GripperTool] Shaft capsule fitted from collision mesh: " +
                      $"localZ=[{min.z:F3}, {max.z:F3}] center=({centerX:F3}, {centerY:F3}) radius={shaftRadius:F3}");
        }

        GameObject CreateMeshObj(string name, string objFile, Material mat)
        {
            var go = new GameObject(name);
            go.transform.SetParent(transform);
            var mf = go.AddComponent<MeshFilter>();
            var mr = go.AddComponent<MeshRenderer>();
            mr.material = mat;

            string path = Path.Combine(Application.streamingAssetsPath, objFile);
            if (File.Exists(path))
            {
                Mesh mesh = LoadOBJMesh(path);
                if (mesh != null)
                {
                    mf.mesh = mesh;
                    Debug.Log($"[GripperTool] Loaded {name}: {mesh.vertexCount}V");
                }
            }
            else
            {
                Debug.LogWarning($"[GripperTool] OBJ not found: {path}");
            }
            return go;
        }

        // ── 可视化更新 ───────────────────────────────────────
        void UpdateVisual()
        {
            Quaternion toolRot = Quaternion.Euler(0, _toolRotY, 0);

            if (_shaftObj != null)
            {
                _shaftObj.transform.position = _toolPos;
                _shaftObj.transform.rotation = toolRot;
                _shaftObj.transform.localScale = Vector3.one * modelScale;
            }

            if (_jawUpObj != null)
            {
                // 上颚：先铰接旋转（开合），再整体旋转+平移
                // 在 localScale=modelScale 下，铰接点是在 OBJ 原始坐标
                _jawUpObj.transform.localScale = Vector3.one * modelScale;
                _jawUpObj.transform.position = _toolPos;
                _jawUpObj.transform.rotation = toolRot;

                // 用 pivot 做开合：先移到 pivot，旋转，再移回
                // 简化：对整个 jaw mesh 做变换
                ApplyJawRotation(_jawUpObj, _currentAngle, toolRot);
            }

            if (_jawDownObj != null)
            {
                _jawDownObj.transform.localScale = Vector3.one * modelScale;
                _jawDownObj.transform.position = _toolPos;
                _jawDownObj.transform.rotation = toolRot;

                ApplyJawRotation(_jawDownObj, -_currentAngle, toolRot);
            }
        }

        void ApplyJawRotation(GameObject jawObj, float angle, Quaternion toolRot)
        {
            // 铰接点的世界位置
            Vector3 pivotWorld = toolRot * _pivotLocal + _toolPos;

            // 先设置到工具位置
            jawObj.transform.position = _toolPos;
            jawObj.transform.rotation = toolRot;
            jawObj.transform.localScale = Vector3.one * modelScale;

            // 绕铰接点旋转
            jawObj.transform.RotateAround(pivotWorld, toolRot * Vector3.right, angle);
        }

        // ── OBJ 解析 ─────────────────────────────────────────
        static Mesh LoadOBJMesh(string path)
        {
            LoadOBJData(path, out Vector3[] verts, out int[] tris);
            if (verts == null || verts.Length == 0) return null;
            var mesh = new Mesh { name = Path.GetFileNameWithoutExtension(path) };
            mesh.SetVertices(new List<Vector3>(verts));
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        static void LoadOBJData(string path, out Vector3[] verts, out int[] tris)
        {
            verts = null; tris = null;
            if (!File.Exists(path)) return;

            var vertList = new List<Vector3>();
            var triList  = new List<int>();
            foreach (string line in File.ReadAllLines(path))
            {
                string s = line.Trim();
                if (s.StartsWith("v "))
                {
                    var p = s.Split(new[]{' '}, System.StringSplitOptions.RemoveEmptyEntries);
                    if (p.Length >= 4)
                        vertList.Add(new Vector3(
                            float.Parse(p[1], CultureInfo.InvariantCulture),
                            float.Parse(p[2], CultureInfo.InvariantCulture),
                            float.Parse(p[3], CultureInfo.InvariantCulture)));
                }
                else if (s.StartsWith("f "))
                {
                    var p  = s.Split(new[]{' '}, System.StringSplitOptions.RemoveEmptyEntries);
                    var fv = new List<int>();
                    for (int i = 1; i < p.Length; i++)
                        fv.Add(int.Parse(p[i].Split('/')[0], CultureInfo.InvariantCulture) - 1);
                    for (int i = 1; i < fv.Count - 1; i++)
                    { triList.Add(fv[0]); triList.Add(fv[i]); triList.Add(fv[i+1]); }
                }
            }
            verts = vertList.ToArray();
            tris = triList.ToArray();
        }

        void OnDestroy()
        {
            if (_jawUpObj) Destroy(_jawUpObj);
            if (_jawDownObj) Destroy(_jawDownObj);
            if (_shaftObj) Destroy(_shaftObj);
        }

        // ── Gizmo: 碰撞胶囊体可视化 ─────────────────────────
        void OnDrawGizmos()
        {
            if (!_initialized || !showCapsuleGizmo) return;

            // 上颚胶囊体 — 青色
            Gizmos.color = new Color(0f, 1f, 1f, 0.5f);
            DrawCapsuleGizmo(_dbgCap0A, _dbgCap0B, _dbgCapR);

            // 下颚胶囊体 — 品红色
            Gizmos.color = new Color(1f, 0f, 1f, 0.5f);
            DrawCapsuleGizmo(_dbgCap1A, _dbgCap1B, _dbgCapR);

            // 杆身胶囊体 — 绿色
            Gizmos.color = new Color(0f, 1f, 0f, 0.3f);
            Quaternion toolRot = Quaternion.Euler(0, _toolRotY, 0);
            Vector3 shaftA = toolRot * _shaftEndLocal + _toolPos;
            Vector3 shaftB = toolRot * _shaftTipLocal + _toolPos;
            DrawCapsuleGizmo(shaftA, shaftB, shaftRadius);

            // AABB — 黄色线框
            Gizmos.color = Color.yellow;
            Vector3 center = (_dbgBBoxMin + _dbgBBoxMax) * 0.5f;
            Vector3 size   = _dbgBBoxMax - _dbgBBoxMin;
            Gizmos.DrawWireCube(center, size);
        }

        static void DrawCapsuleGizmo(Vector3 a, Vector3 b, float r)
        {
            Gizmos.DrawWireSphere(a, r);
            Gizmos.DrawWireSphere(b, r);
            Gizmos.DrawLine(a, b);
            // 画几条平行线表示胶囊体轮廓
            Vector3 dir = (b - a).normalized;
            Vector3 up = Vector3.up;
            if (Mathf.Abs(Vector3.Dot(dir, up)) > 0.9f) up = Vector3.right;
            Vector3 side = Vector3.Cross(dir, up).normalized * r;
            Vector3 topDir = Vector3.Cross(dir, side).normalized * r;
            Gizmos.DrawLine(a + side, b + side);
            Gizmos.DrawLine(a - side, b - side);
            Gizmos.DrawLine(a + topDir, b + topDir);
            Gizmos.DrawLine(a - topDir, b - topDir);
        }

        // ── GUI ──────────────────────────────────────────────
        void OnGUI()
        {
            if (!_initialized) return;
            var style = new GUIStyle(GUI.skin.box) { fontSize = 12 };
            style.normal.textColor = Color.white;
            style.alignment = TextAnchor.UpperLeft;
            style.richText = true;

            string state = IsGrasping ? "<color=#FF4444>夹取中</color>" :
                          (_wantClose ? "<color=#FFFF00>闭合中</color>" : "张开");

            string info =
                $"夹爪: {state}\n" +
                $"角度: {_currentAngle:F1}°\n" +
                $"粒子: {_graspedParticles.Count}\n" +
                $"R={capsuleRadius:F4}\n" +
                $"FGHVBN移动 1/3旋转 0开合";
            GUI.Box(new Rect(10, 360, 240, 100), info, style);
        }
    }
}
