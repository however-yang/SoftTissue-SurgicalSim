// TetMeshLoader.cs
// 加載四面體網格文件
// 支持格式：
//   1. Matthias Müller Blender TetPlugin JSON（推薦，最成熟）
//      插件地址: https://github.com/matthias-research/pages/tenMinutePhysics/BlenderTetPlugin.py
//   2. TetGen .node + .ele 文件對（高精度離線生成）
//      工具地址: http://wias-berlin.de/software/tetgen/

using System;
using System.Collections;
using System.IO;
using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim.Core
{
    public class TetMeshLoader : MonoBehaviour
    {
        // ── Inspector 配置 ──────────────────────────────────
        [Header("網格文件配置")]
        [Tooltip("JSON 文件路徑（Blender TetPlugin 導出）\n" +
                 "相對於 StreamingAssets 目錄，例如: liver.json")]
        public string jsonFileName = "liver.json";

        [Tooltip("是否在 Start 時自動加載")]
        public bool autoLoad = true;

        [Header("物理參數")]
        [Tooltip("組織密度 kg/m³（肝臟≈1050, 水=1000）")]
        [Range(500f, 2000f)]
        public float massDensity = 1050f;

        [Tooltip("模型縮放（如果導出時單位是 mm 請設 0.001）")]
        public float meshScale = 1f;

        // ── 公開結果 ────────────────────────────────────────
        /// <summary>加載完成後可訪問的網格數據</summary>
        public TetMeshData MeshData { get; private set; }

        /// <summary>加載是否已完成</summary>
        public bool IsLoaded { get; private set; } = false;

        /// <summary>加載完成回調（可選）</summary>
        public event Action<TetMeshData> OnMeshLoaded;

        // ──────────────────────────────────────────────────────
        void Start()
        {
            if (autoLoad)
                StartCoroutine(LoadJsonAsync(jsonFileName));
        }

        // ── 公開 API ────────────────────────────────────────

        /// <summary>異步加載 JSON（Blender TetPlugin 格式）</summary>
        public IEnumerator LoadJsonAsync(string fileName)
        {
            string path = Path.Combine(Application.streamingAssetsPath, fileName);
            Debug.Log($"[TetMeshLoader] 加載 JSON: {path}");

            if (!File.Exists(path))
            {
                Debug.LogError($"[TetMeshLoader] 文件不存在: {path}\n" +
                               $"請將 JSON 文件放到 Assets/StreamingAssets/ 目錄下");
                yield break;
            }

            // 讀取文件（大文件在後台線程讀取避免卡頓）
            string json = null;
            bool readDone = false;
            System.Threading.ThreadPool.QueueUserWorkItem(_ =>
            {
                try   { json = File.ReadAllText(path); }
                catch (Exception e) { Debug.LogError($"[TetMeshLoader] 讀取失敗: {e.Message}"); }
                readDone = true;
            });

            // 等待讀取完成
            while (!readDone) yield return null;

            if (json == null) yield break;

            // 反序列化 JSON
            TetMeshJson jsonData = null;
            try
            {
                jsonData = JsonUtility.FromJson<TetMeshJson>(json);
            }
            catch (Exception e)
            {
                Debug.LogError($"[TetMeshLoader] JSON 解析失敗: {e.Message}");
                yield break;
            }

            // 驗證數據完整性
            if (!ValidateJson(jsonData)) yield break;

            // 構建 TetMeshData
            MeshData = new TetMeshData();

            // 縮放頂點
            if (!Mathf.Approximately(meshScale, 1f))
                ScaleVerts(jsonData.verts, meshScale);

            MeshData.InitFromJson(jsonData, massDensity);
            IsLoaded = true;

            Debug.Log($"[TetMeshLoader] ✅ 加載完成 | " +
                      $"粒子: {MeshData.NumParticles} | " +
                      $"四面體: {MeshData.NumTets} | " +
                      $"表面三角形: {MeshData.NumSurfaceTris}");

            OnMeshLoaded?.Invoke(MeshData);
        }

        /// <summary>
        /// 加載 TetGen 格式（.node + .ele 文件對）
        /// 用於 TetGen 命令行生成的高精度網格
        /// </summary>
        public IEnumerator LoadTetGenAsync(string nodeFileName, string eleFileName)
        {
            string nodePath = Path.Combine(Application.streamingAssetsPath, nodeFileName);
            string elePath  = Path.Combine(Application.streamingAssetsPath, eleFileName);

            Debug.Log($"[TetMeshLoader] 加載 TetGen: {nodePath}");

            if (!File.Exists(nodePath) || !File.Exists(elePath))
            {
                Debug.LogError($"[TetMeshLoader] 文件不存在: {nodePath} 或 {elePath}");
                yield break;
            }

            float[]  verts  = null;
            int[]    tetIds = null;
            bool     done   = false;

            System.Threading.ThreadPool.QueueUserWorkItem(_ =>
            {
                try
                {
                    verts  = ParseNodeFile(nodePath);
                    tetIds = ParseEleFile(elePath);
                }
                catch (Exception e)
                {
                    Debug.LogError($"[TetMeshLoader] TetGen 解析失敗: {e.Message}");
                }
                done = true;
            });

            while (!done) yield return null;
            if (verts == null || tetIds == null) yield break;

            // TetGen 格式沒有內置的邊界三角形，需要額外計算
            int[] surfaceTris = ExtractBoundaryTrisFromTets(tetIds, verts.Length / 3);

            var jsonData = new TetMeshJson
            {
                verts            = verts,
                tetIds           = tetIds,
                tetSurfaceTriIds = surfaceTris,
                tetEdgeIds       = null
            };

            if (meshScale != 1f) ScaleVerts(jsonData.verts, meshScale);

            MeshData = new TetMeshData();
            MeshData.InitFromJson(jsonData, massDensity);
            IsLoaded = true;

            Debug.Log($"[TetMeshLoader] ✅ TetGen 加載完成 | " +
                      $"粒子: {MeshData.NumParticles} | " +
                      $"四面體: {MeshData.NumTets}");

            OnMeshLoaded?.Invoke(MeshData);
        }

        // ── 私有方法 ─────────────────────────────────────────

        bool ValidateJson(TetMeshJson data)
        {
            if (data == null)
            {
                Debug.LogError("[TetMeshLoader] JSON 為 null"); return false;
            }
            if (data.verts == null || data.verts.Length == 0)
            {
                Debug.LogError("[TetMeshLoader] verts 數組為空"); return false;
            }
            if (data.tetIds == null || data.tetIds.Length == 0)
            {
                Debug.LogError("[TetMeshLoader] tetIds 數組為空"); return false;
            }
            if (data.verts.Length % 3 != 0)
            {
                Debug.LogError($"[TetMeshLoader] verts 長度 {data.verts.Length} 不是 3 的倍數"); return false;
            }
            if (data.tetIds.Length % 4 != 0)
            {
                Debug.LogError($"[TetMeshLoader] tetIds 長度 {data.tetIds.Length} 不是 4 的倍數"); return false;
            }
            return true;
        }

        static void ScaleVerts(float[] verts, float scale)
        {
            for (int i = 0; i < verts.Length; i++)
                verts[i] *= scale;
        }

        // ── TetGen .node 文件解析 ────────────────────────────
        // 格式: 第一行 = "numNodes dim attrs markers"
        //       後續行 = "idx x y z [attrs] [marker]"
        static float[] ParseNodeFile(string path)
        {
            string[] lines = File.ReadAllLines(path);
            int li = 0;

            // 跳過注釋行
            while (li < lines.Length && lines[li].TrimStart().StartsWith("#")) li++;

            string[] header = lines[li++].Split(new char[]{' ','\t'},
                StringSplitOptions.RemoveEmptyEntries);
            int numNodes = int.Parse(header[0]);

            float[] verts = new float[numNodes * 3];
            int vi = 0;

            for (int i = 0; i < numNodes; i++)
            {
                while (li < lines.Length && lines[li].TrimStart().StartsWith("#")) li++;
                string[] parts = lines[li++].Split(new char[]{' ','\t'},
                    StringSplitOptions.RemoveEmptyEntries);
                // TetGen 索引從 1 開始，跳過 parts[0]
                verts[vi++] = float.Parse(parts[1], System.Globalization.CultureInfo.InvariantCulture);
                verts[vi++] = float.Parse(parts[2], System.Globalization.CultureInfo.InvariantCulture);
                verts[vi++] = float.Parse(parts[3], System.Globalization.CultureInfo.InvariantCulture);
            }
            return verts;
        }

        // ── TetGen .ele 文件解析 ─────────────────────────────
        // 格式: 第一行 = "numTets nodesPerTet attrs"
        //       後續行 = "idx n0 n1 n2 n3 [attr]"
        static int[] ParseEleFile(string path)
        {
            string[] lines = File.ReadAllLines(path);
            int li = 0;

            while (li < lines.Length && lines[li].TrimStart().StartsWith("#")) li++;

            string[] header = lines[li++].Split(new char[]{' ','\t'},
                StringSplitOptions.RemoveEmptyEntries);
            int numTets = int.Parse(header[0]);

            int[] tetIds = new int[numTets * 4];
            int ti = 0;

            for (int i = 0; i < numTets; i++)
            {
                while (li < lines.Length && lines[li].TrimStart().StartsWith("#")) li++;
                string[] parts = lines[li++].Split(new char[]{' ','\t'},
                    StringSplitOptions.RemoveEmptyEntries);
                // TetGen 索引從 1 開始，減 1 轉為 0-based
                tetIds[ti++] = int.Parse(parts[1]) - 1;
                tetIds[ti++] = int.Parse(parts[2]) - 1;
                tetIds[ti++] = int.Parse(parts[3]) - 1;
                tetIds[ti++] = int.Parse(parts[4]) - 1;
            }
            return tetIds;
        }

        // ── 從四面體網格提取邊界三角形 ──────────────────────
        // 邊界面 = 只屬於一個四面體的三角形面
        // 使用 Dictionary 統計每個面出現次數
        static int[] ExtractBoundaryTrisFromTets(int[] tetIds, int numVerts)
        {
            int numTets = tetIds.Length / 4;

            // 每個 tet 有 4 個面
            var faceCount = new System.Collections.Generic.Dictionary<long, int>();
            var faceOrder = new System.Collections.Generic.Dictionary<long, int[]>();

            int[][] tetFaces = new int[][]
            {
                new int[]{0,2,1}, new int[]{0,1,3},
                new int[]{0,3,2}, new int[]{1,2,3}
            };

            for (int t = 0; t < numTets; t++)
            {
                int b = t * 4;
                int[] tv = { tetIds[b], tetIds[b+1], tetIds[b+2], tetIds[b+3] };

                foreach (var face in tetFaces)
                {
                    int[] f = { tv[face[0]], tv[face[1]], tv[face[2]] };
                    // 排序後生成唯一 key
                    int[] sorted = { f[0], f[1], f[2] };
                    Array.Sort(sorted);
                    long key = (long)sorted[0] * numVerts * numVerts
                             + (long)sorted[1] * numVerts
                             + sorted[2];

                    if (faceCount.ContainsKey(key))
                        faceCount[key]++;
                    else
                    {
                        faceCount[key] = 1;
                        faceOrder[key] = f; // 保存原始順序（保持法線方向）
                    }
                }
            }

            // 只保留出現 1 次的面（邊界面）
            var boundary = new System.Collections.Generic.List<int>();
            foreach (var kv in faceCount)
            {
                if (kv.Value == 1)
                {
                    int[] f = faceOrder[kv.Key];
                    boundary.Add(f[0]);
                    boundary.Add(f[1]);
                    boundary.Add(f[2]);
                }
            }

            Debug.Log($"[TetMeshLoader] 提取邊界三角形: {boundary.Count / 3} 個");
            return boundary.ToArray();
        }
    }
}
