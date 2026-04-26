// SoftBodyRoot.cs
// Phase 1 整合入口：加載 + 顯示四面體網格
// Phase 2+ 將在此添加 XPBD 求解器調用
// 使用說明：
//   1. 在場景中創建一個空的 GameObject，命名為 "SoftBody"
//   2. 把此腳本和 TetMeshLoader + TetMeshVisualizer 都掛載到同一個 GO
//   3. 把肝臟 JSON 文件放到 Assets/StreamingAssets/liver.json
//   4. 運行場景，肝臟表面網格將自動顯示

using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim
{
    public class SoftBodyRoot : MonoBehaviour
    {
        [Header("組件引用（留空則自動查找同 GO 的組件）")]
        public TetMeshLoader     meshLoader;
        public TetMeshVisualizer visualizer;

        [Header("Phase 1 驗證 - 統計信息")]
        [SerializeField, Tooltip("加載完成後顯示網格統計")]
        bool showStatsOnLoad = true;

        void Awake()
        {
            // 自動獲取同 GameObject 上的組件
            if (meshLoader  == null) meshLoader  = GetComponent<TetMeshLoader>();
            if (visualizer  == null) visualizer  = GetComponent<TetMeshVisualizer>();

            if (meshLoader == null)
            {
                Debug.LogError("[SoftBodyRoot] 缺少 TetMeshLoader 組件！");
                return;
            }

            // 訂閱加載事件
            meshLoader.OnMeshLoaded += OnMeshLoaded;
        }

        void OnDestroy()
        {
            if (meshLoader != null)
                meshLoader.OnMeshLoaded -= OnMeshLoaded;
        }

        void OnMeshLoaded(TetMeshData data)
        {
            if (!showStatsOnLoad) return;

            // 計算網格包圍盒
            Vector3 min = Vector3.one * float.MaxValue;
            Vector3 max = Vector3.one * float.MinValue;
            foreach (var p in data.Positions)
            {
                min = Vector3.Min(min, p);
                max = Vector3.Max(max, p);
            }
            Vector3 size = max - min;
            Vector3 center = (min + max) * 0.5f;

            // 計算總體積
            float totalVol = 0f;
            for (int t = 0; t < data.NumTets; t++)
                if (data.TetActive[t])
                    totalVol += data.ComputeTetVolume(t);

            Debug.Log($"[SoftBodyRoot] ══ Phase 1 網格統計 ══\n" +
                      $"  粒子數: {data.NumParticles}\n" +
                      $"  四面體: {data.NumTets}\n" +
                      $"  表面三角形: {data.NumSurfaceTris}\n" +
                      $"  包圍盒大小: {size:F3} m\n" +
                      $"  包圍盒中心: {center:F3}\n" +
                      $"  總體積: {totalVol * 1e6f:F1} cm³\n" +
                      $"  ══════════════════════");

            // 居中模型到場景原點
            CenterMesh(data, center);
        }

        /// <summary>將網格平移到場景原點</summary>
        void CenterMesh(TetMeshData data, Vector3 center)
        {
            for (int i = 0; i < data.NumParticles; i++)
            {
                data.Positions[i]     -= center;
                data.RestPositions[i] -= center;
                data.PrevPositions[i] -= center;
            }

            // 刷新視覺
            if (visualizer != null)
                visualizer.Refresh();

            Debug.Log($"[SoftBodyRoot] 網格已居中到原點");
        }

        // ── Phase 2 預留：物理更新入口 ─────────────────────
        // void FixedUpdate()
        // {
        //     if (!meshLoader.IsLoaded) return;
        //     xpbdSolver.Step(meshLoader.MeshData, Time.fixedDeltaTime);
        //     visualizer.Refresh();
        // }
    }
}
