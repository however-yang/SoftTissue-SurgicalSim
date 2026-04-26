// SurgicalTool.cs — 手术刀刃控制
// 刀刃表示为线段 (BladeA → BladeB)
// 移动基于世界坐标系, 不旋转
//   J/L → X, I/K → Y, U/O → Z, Shift → 加速

using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

namespace SurgicalSim.Cutting
{
    public class SurgicalTool : MonoBehaviour
    {
        [Header("Tool Model")]
        public string objFileName = "xiaogun_v1.obj";
        public float  modelScale  = 0.15f;

        [Header("Blade")]
        [Tooltip("刀刃物理长度 (世界单位, 不受 modelScale 影响)")]
        public float bladeLength = 3.0f;

        [Header("Visual")]
        public Color toolColor = new Color(0.75f, 0.75f, 0.82f, 1f);

        [Header("Control")]
        public float moveSpeed       = 0.45f;
        public float boostMultiplier = 3.0f;

        // ── 刀刃端点 (世界坐标) ─────────────────────────────
        /// <summary>刀刃顶端 (远离尖端的一端)</summary>
        public Vector3 BladeA     { get; private set; }
        /// <summary>刀刃尖端</summary>
        public Vector3 BladeB     { get; private set; }
        /// <summary>上一帧的 BladeA</summary>
        public Vector3 PrevBladeA { get; private set; }
        /// <summary>上一帧的 BladeB</summary>
        public Vector3 PrevBladeB { get; private set; }

        // 保留向后兼容
        public Vector3 TipPosition     => BladeB;
        public Vector3 PrevTipPosition => PrevBladeB;
        public Vector3 ToolDirection   => Vector3.down;
        public bool    IsCutting       { get; set; }

        // ── 内部 ─────────────────────────────────────────────
        GameObject   _toolObj;
        MeshFilter   _toolMF;
        MeshRenderer _toolMR;
        Vector3      _toolPos;
        float        _meshHalfLen = 0.5f;

        void Start()
        {
            _toolPos = new Vector3(0f, 0.2f, 0f);
            CreateToolFromOBJ();
            UpdateBladeEndpoints();
            PrevBladeA = BladeA;
            PrevBladeB = BladeB;
            ApplyTransform();
        }

        void Update()
        {
            // 1. 保存上帧刀刃
            PrevBladeA = BladeA;
            PrevBladeB = BladeB;

            // 2. 世界坐标移动
            Vector3 move = Vector3.zero;
            if (Input.GetKey(KeyCode.L)) move.x += 1f;
            if (Input.GetKey(KeyCode.J)) move.x -= 1f;
            if (Input.GetKey(KeyCode.I)) move.y += 1f;
            if (Input.GetKey(KeyCode.K)) move.y -= 1f;
            if (Input.GetKey(KeyCode.O)) move.z += 1f;
            if (Input.GetKey(KeyCode.U)) move.z -= 1f;

            bool boost = Input.GetKey(KeyCode.LeftShift) || Input.GetKey(KeyCode.RightShift);
            float speed = boost ? moveSpeed * boostMultiplier : moveSpeed;
            if (move.sqrMagnitude > 1e-8f)
                _toolPos += move.normalized * speed * Time.deltaTime;

            IsCutting = false; // 由 CuttingToolV3 碰触检测控制

            // 3. 更新刀刃端点
            UpdateBladeEndpoints();
            ApplyTransform();
        }

        void FixedUpdate()
        {
            UpdateBladeEndpoints();
            ApplyTransform();
        }

        void UpdateBladeEndpoints()
        {
            // bladeLength 是物理刀刃长度（不乘 modelScale）
            // modelScale 只影响可视模型大小
            float halfBlade = bladeLength * 0.5f;
            // BladeA = 上端, BladeB = 下端(尖端)
            BladeA = _toolPos - ToolDirection * halfBlade;  // 上
            BladeB = _toolPos + ToolDirection * halfBlade;  // 下(尖端)
        }

        void ApplyTransform()
        {
            if (_toolObj == null) return;
            _toolObj.transform.position   = _toolPos;
            _toolObj.transform.rotation   = Quaternion.LookRotation(ToolDirection);
            _toolObj.transform.localScale = Vector3.one * modelScale;
        }

        // ═══════════════ OBJ 加载 ═══════════════════════════

        void CreateToolFromOBJ()
        {
            _toolObj = new GameObject("SurgicalTool_Model");
            _toolObj.transform.SetParent(transform);
            _toolMF = _toolObj.AddComponent<MeshFilter>();
            _toolMR = _toolObj.AddComponent<MeshRenderer>();

            string objPath = Path.Combine(Application.streamingAssetsPath, objFileName);
            if (File.Exists(objPath))
            {
                Mesh mesh = LoadOBJ(objPath);
                if (mesh != null)
                {
                    _toolMF.mesh = mesh;
                    _meshHalfLen = mesh.bounds.extents.z;
                    Debug.Log($"[SurgicalTool] OBJ V:{mesh.vertexCount} HalfLen:{_meshHalfLen:F3}");
                }
            }
            else
            {
                Debug.LogWarning($"[SurgicalTool] OBJ not found: {objPath}");
                var cyl = GameObject.CreatePrimitive(PrimitiveType.Capsule);
                _toolMF.mesh = cyl.GetComponent<MeshFilter>().mesh;
                _meshHalfLen = 0.5f;
                Destroy(cyl);
            }

            var mat = new Material(
                Shader.Find("Universal Render Pipeline/Lit") ??
                Shader.Find("Standard"));
            mat.color = toolColor;
            _toolMR.material = mat;
        }

        static Mesh LoadOBJ(string path)
        {
            var verts = new List<Vector3>();
            var tris  = new List<int>();
            foreach (string line in File.ReadAllLines(path))
            {
                string s = line.Trim();
                if (s.StartsWith("v "))
                {
                    var p = s.Split(new[]{' '}, System.StringSplitOptions.RemoveEmptyEntries);
                    if (p.Length >= 4)
                        verts.Add(new Vector3(
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
                    { tris.Add(fv[0]); tris.Add(fv[i]); tris.Add(fv[i+1]); }
                }
            }
            if (verts.Count == 0) return null;
            var mesh = new Mesh { name = Path.GetFileNameWithoutExtension(path) };
            mesh.SetVertices(verts);
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            return mesh;
        }

        void OnDestroy() { if (_toolObj) Destroy(_toolObj); }
    }
}
