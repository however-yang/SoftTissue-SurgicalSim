// TetMeshVisualizer.cs
// 把四面體網格的邊界表面渲染成 Unity Mesh
// 網格頂點每幀跟隨 TetMeshData.Positions 更新（XPBD 求解後調用 Refresh）
// Phase 1: 靜態顯示（無物理）
// Phase 2+ : 動態更新（物理求解後調用 Refresh()）

using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim.Core
{
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public class TetMeshVisualizer : MonoBehaviour
    {
        // ── Inspector 配置 ──────────────────────────────────
        [Header("引用")]
        [Tooltip("四面體網格加載器（同 GameObject 或手動指定）")]
        public TetMeshLoader meshLoader;

        [Header("視覺配置")]
        [Tooltip("表面材質（留空則使用默認 URP Lit）")]
        public Material surfaceMaterial;

        [Tooltip("是否顯示四面體邊框（Debug 用，性能消耗大）")]
        public bool showWireframe = false;

        // ── 私有狀態 ─────────────────────────────────────────
        MeshFilter   _meshFilter;
        MeshRenderer _meshRenderer;
        Mesh         _surfaceMesh;

        // 頂點緩存（避免每幀 GC）
        Vector3[] _surfaceVerts;
        Vector3[] _surfaceNormals;

        TetMeshData _data;
        bool        _initialized = false;

        // ── 生命週期 ─────────────────────────────────────────
        void Awake()
        {
            _meshFilter   = GetComponent<MeshFilter>();
            _meshRenderer = GetComponent<MeshRenderer>();

            if (meshLoader == null)
                meshLoader = GetComponent<TetMeshLoader>();
        }

        void Start()
        {
            if (meshLoader == null)
            {
                Debug.LogError("[TetMeshVisualizer] 未找到 TetMeshLoader，請掛載到同一個 GameObject");
                return;
            }

            // 等待加載完成後初始化
            meshLoader.OnMeshLoaded += Init;

            // 如果已經加載完成（同幀），直接初始化
            if (meshLoader.IsLoaded)
                Init(meshLoader.MeshData);
        }

        void OnDestroy()
        {
            if (meshLoader != null)
                meshLoader.OnMeshLoaded -= Init;
        }

        // ── 初始化 ──────────────────────────────────────────
        void Init(TetMeshData data)
        {
            _data = data;

            if (data.NumSurfaceTris == 0)
            {
                Debug.LogWarning("[TetMeshVisualizer] 沒有表面三角形數據，無法渲染");
                return;
            }

            // 構建 Unity Mesh
            _surfaceMesh = new Mesh
            {
                name = "TetSurface",
                // 超過 65535 頂點時需要 32-bit 索引
                indexFormat = data.NumParticles > 65535
                            ? UnityEngine.Rendering.IndexFormat.UInt32
                            : UnityEngine.Rendering.IndexFormat.UInt16
            };

            // 頂點緩存
            _surfaceVerts   = new Vector3[data.NumParticles];
            _surfaceNormals = new Vector3[data.NumParticles];

            // 複製初始頂點位置
            System.Array.Copy(data.Positions, _surfaceVerts, data.NumParticles);

            _surfaceMesh.vertices  = _surfaceVerts;
            _surfaceMesh.triangles = data.SurfaceTriIds;
            _surfaceMesh.RecalculateNormals();
            _surfaceMesh.RecalculateBounds();

            // ★ 关键修复: mesh setter 会创建实例副本,
            // 必须读回 getter 拿到渲染器实际使用的那个实例
            _meshFilter.mesh = _surfaceMesh;
            _surfaceMesh = _meshFilter.mesh;  // 捕获实际渲染实例

            // 設置材質
            if (surfaceMaterial != null)
                _meshRenderer.material = surfaceMaterial;
            else
            {
                // 使用 URP Lit 默認材質（紅色，用於快速識別）
                var mat = new Material(Shader.Find("Universal Render Pipeline/Lit"));
                mat.color = new Color(0.85f, 0.25f, 0.15f); // 肝臟紅
                _meshRenderer.material = mat;
            }

            _initialized = true;
            Debug.Log($"[TetMeshVisualizer] ✅ 表面網格初始化完成 | " +
                      $"頂點: {data.NumParticles} | 三角形: {data.NumSurfaceTris}");
        }

        // ── 公開 API ─────────────────────────────────────────

        /// <summary>
        /// 每幀物理求解後調用此方法刷新視覺網格
        /// 直接讀取 TetMeshData.Positions，無 GC 分配
        /// </summary>
        int _refreshCount = 0;

        public void Refresh()
        {
            if (!_initialized || _data == null) return;

            // 检查顶点数是否变化（vertex splitting 会增加顶点）
            if (_surfaceVerts == null || _surfaceVerts.Length < _data.NumParticles)
            {
                _surfaceVerts   = new Vector3[_data.NumParticles];
                _surfaceNormals = new Vector3[_data.NumParticles];
                _surfaceMesh.indexFormat = _data.NumParticles > 65535
                    ? UnityEngine.Rendering.IndexFormat.UInt32
                    : UnityEngine.Rendering.IndexFormat.UInt16;
            }

            // 只拷贝有效的粒子数
            System.Array.Copy(_data.Positions, _surfaceVerts, _data.NumParticles);

            _surfaceMesh.SetVertices(_surfaceVerts, 0, _data.NumParticles);
            _surfaceMesh.RecalculateNormals();
            _surfaceMesh.RecalculateBounds();
        }

        /// <summary>
        /// 切割后调用: 重建拓扑 (三角形索引 + 顶点数组)
        /// 直接操作 _surfaceMesh, 避免 mf.mesh 副本问题
        /// </summary>
        public void RebuildTopology(int[] newTriangles)
        {
            if (!_initialized || _data == null || _surfaceMesh == null) return;

            // 扩容顶点缓存
            if (_surfaceVerts == null || _surfaceVerts.Length < _data.NumParticles)
            {
                _surfaceVerts   = new Vector3[_data.NumParticles];
                _surfaceNormals = new Vector3[_data.NumParticles];
            }

            // 复制所有粒子位置
            System.Array.Copy(_data.Positions, _surfaceVerts, _data.NumParticles);

            // 重建 mesh
            _surfaceMesh.Clear();
            _surfaceMesh.indexFormat = _data.NumParticles > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;
            _surfaceMesh.SetVertices(_surfaceVerts, 0, _data.NumParticles);
            _surfaceMesh.triangles = newTriangles;
            _surfaceMesh.RecalculateNormals();
            _surfaceMesh.RecalculateBounds();

            Debug.Log($"[Visualizer] RebuildTopology: V={_data.NumParticles} T={newTriangles.Length/3}");
        }

        // ── Gizmos（Debug 用）────────────────────────────────
        void OnDrawGizmos()
        {
            if (!showWireframe || _data == null || !_initialized) return;

            Gizmos.color = new Color(0f, 1f, 0f, 0.3f);

            // 畫出四面體邊框（只畫 active 的）
            int drawnCount = 0;
            for (int t = 0; t < _data.NumTets && drawnCount < 500; t++)
            {
                if (!_data.TetActive[t]) continue;

                Vector3 p0 = _data.Positions[_data.TetIds[t*4+0]];
                Vector3 p1 = _data.Positions[_data.TetIds[t*4+1]];
                Vector3 p2 = _data.Positions[_data.TetIds[t*4+2]];
                Vector3 p3 = _data.Positions[_data.TetIds[t*4+3]];

                Gizmos.DrawLine(p0, p1); Gizmos.DrawLine(p0, p2);
                Gizmos.DrawLine(p0, p3); Gizmos.DrawLine(p1, p2);
                Gizmos.DrawLine(p1, p3); Gizmos.DrawLine(p2, p3);
                drawnCount++;
            }
        }
    }
}
