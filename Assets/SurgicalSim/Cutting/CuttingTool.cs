// CuttingTool.cs — 切割工具管理器
// 使用 TetRemovalCutter: 删除 tet + 双面 Cap 重建
// 不做 Solver Reinit — 只更新 TetActive buffer

using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Physics;

namespace SurgicalSim.Cutting
{
    public class CuttingTool : MonoBehaviour
    {
        [Header("切割参数")]
        [Tooltip("工具切割影响半径")]
        public float cutRadius = 0.015f;

        [Header("Cap 渲染")]
        [Tooltip("切面内部颜色")]
        public Color capColor = new Color(0.85f, 0.15f, 0.10f, 1f);

        // ── 私有状态 ─────────────────────────────────────────
        SurgicalTool        _surgicalTool;
        TetRemovalCutter    _cutter;
        TetMeshData         _data;
        TetMeshVisualizer   _visualizer;
        XPBDSolverGPU       _solver;

        // Cap 专用 Mesh
        Mesh       _capMesh;
        GameObject _capObject;

        // ── 生命周期 ─────────────────────────────────────────

        void Awake()
        {
            _surgicalTool = GetComponent<SurgicalTool>();
            if (_surgicalTool == null)
                _surgicalTool = gameObject.AddComponent<SurgicalTool>();
        }

        public void Init(TetMeshData data, XPBDSolverGPU solver,
                         TetMeshVisualizer visualizer)
        {
            _data       = data;
            _solver     = solver;
            _visualizer = visualizer;

            _cutter = new TetRemovalCutter();
            _cutter.Init(data, solver);

            // 创建 Cap 渲染对象
            CreateCapObject();

            Debug.Log($"[CuttingTool] TetRemoval 初始化 | " +
                      $"P:{data.NumParticles} T:{data.NumTets}");
        }

        void Update() { }

        // ── 公开 API ─────────────────────────────────────────

        public void FlushCutToGPU()
        {
            if (_data == null || _solver == null || _cutter == null) return;

            if (_surgicalTool != null && _surgicalTool.IsCutting)
            {
                var result = _cutter.Cut(
                    _surgicalTool.TipPosition,
                    _surgicalTool.PrevTipPosition,
                    _surgicalTool.ToolDirection,
                    cutRadius);

                if (result.removedTets > 0)
                {
                    // 增量更新 GPU (不做 Dispose+Init!)
                    _cutter.FlushToGPU();

                    // 重建表面 Mesh
                    UpdateSurface();

                    // 更新 Cap Mesh
                    UpdateCapMesh();
                }
            }
            else
            {
                _cutter.ResetNormal();
            }
        }

        // ── GUI 信息 ─────────────────────────────────────────
        public int TotalSplitVerts => 0;
        public int TotalCutEdges => _cutter != null ? _cutter.TotalRemovedTets : 0;
        public int TotalDisabledEdges => TotalCutEdges;
        public int TotalCutTets => _cutter != null ? _cutter.TotalRemovedTets : 0;

        // ── 内部方法 ─────────────────────────────────────────

        void CreateCapObject()
        {
            _capObject = new GameObject("CutCap");
            _capObject.transform.SetParent(_visualizer != null ?
                _visualizer.transform : transform, false);

            var mf = _capObject.AddComponent<MeshFilter>();
            var mr = _capObject.AddComponent<MeshRenderer>();

            _capMesh = new Mesh();
            _capMesh.name = "CutCapMesh";
            mf.mesh = _capMesh;

            // 双面渲染材质
            var mat = new Material(Shader.Find("Standard"));
            mat.color = capColor;
            mat.SetFloat("_Glossiness", 0.3f);
            mat.SetFloat("_Metallic", 0f);
            // 关闭背面剔除实现双面渲染
            mat.SetInt("_Cull", 0); // Cull Off
            mr.material = mat;
        }

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

            int[] newTris = SurfaceReconstructor.RebuildSurface(_data);
            mesh.triangles = newTris;
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
        }

        void UpdateCapMesh()
        {
            if (_capMesh == null || _cutter == null) return;

            var verts = _cutter.CapVertices;
            var tris = _cutter.CapTriangles;
            var normals = _cutter.CapNormals;

            if (verts.Count == 0) return;

            _capMesh.Clear();
            _capMesh.SetVertices(verts);
            _capMesh.SetNormals(normals);
            _capMesh.indexFormat = verts.Count > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;
            _capMesh.SetTriangles(tris, 0);
            _capMesh.RecalculateBounds();
        }

        void OnDestroy()
        {
            if (_capMesh != null) Destroy(_capMesh);
            if (_capObject != null) Destroy(_capObject);
        }
    }
}
