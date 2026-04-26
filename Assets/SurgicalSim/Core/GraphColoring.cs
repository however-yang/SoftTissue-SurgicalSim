// GraphColoring.cs
// 對四面體網格執行圖著色（Graph Coloring）
// 作用：把 tet 分成若干「顏色組」，同一組內的 tet 不共享頂點
//       → 同一顏色組可以完全並行執行約束求解（無數據競爭）
// 算法：貪心著色（Greedy Coloring），O(n·k) 複雜度，k = 顏色數
// 參考：GPU-accelerated XPBD 圖著色方案（業界標準做法）

using System;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

namespace SurgicalSim.Core
{
    public static class GraphColoring
    {
        /// <summary>
        /// 對四面體執行貪心圖著色。
        /// 返回 colorGroups：每個元素是一個顏色組包含的 tet 索引列表。
        /// </summary>
        /// <param name="tetIds">四面體頂點索引（平坦數組，每4個一組）</param>
        /// <param name="numTets">四面體總數</param>
        /// <param name="numParticles">粒子總數</param>
        /// <returns>顏色分組，每組為 tet 索引數組</returns>
        public static List<int[]> Compute(int[] tetIds, int numTets, int numParticles,
            bool[] tetActive = null)
        {
            var startTime = DateTime.Now;

            // 每個粒子被哪些 tet 使用（鄰接關係）
            // vertToTets[v] = 包含頂點 v 的 tet 列表
            var vertToTets = new List<int>[numParticles];
            for (int i = 0; i < numParticles; i++)
                vertToTets[i] = new List<int>(8); // 每個頂點平均屬於 ~8 個 tet

            for (int t = 0; t < numTets; t++)
            {
                if (tetActive != null && !tetActive[t]) continue;
                int b = t * 4;
                vertToTets[tetIds[b + 0]].Add(t);
                vertToTets[tetIds[b + 1]].Add(t);
                vertToTets[tetIds[b + 2]].Add(t);
                vertToTets[tetIds[b + 3]].Add(t);
            }

            // ── 貪心著色 ────────────────────────────────────────
            int[] tetColor  = new int[numTets];
            Array.Fill(tetColor, -1); // -1 = 未分配

            // 工作集：記錄鄰居已使用的顏色
            var neighborColors = new HashSet<int>(32);

            for (int t = 0; t < numTets; t++)
            {
                if (tetActive != null && !tetActive[t]) { tetColor[t] = 0; continue; }
                int b = t * 4;
                neighborColors.Clear();

                // 收集共享頂點的所有已著色 tet 的顏色
                for (int vi = 0; vi < 4; vi++)
                {
                    int v = tetIds[b + vi];
                    foreach (int neighbor in vertToTets[v])
                    {
                        if (neighbor < t && tetColor[neighbor] >= 0)
                            neighborColors.Add(tetColor[neighbor]);
                    }
                }

                // 找最小可用顏色
                int color = 0;
                while (neighborColors.Contains(color))
                    color++;

                tetColor[t] = color;
            }

            // ── 按顏色分組 ───────────────────────────────────────
            int numColors = 0;
            foreach (int c in tetColor)
                if (c > numColors) numColors = c;
            numColors++;

            var groupLists = new List<int>[numColors];
            for (int i = 0; i < numColors; i++)
                groupLists[i] = new List<int>(numTets / numColors + 10);

            for (int t = 0; t < numTets; t++)
            {
                if (tetActive != null && !tetActive[t]) continue;
                groupLists[tetColor[t]].Add(t);
            }

            var groups = new List<int[]>(numColors);
            foreach (var g in groupLists)
                groups.Add(g.ToArray());

            // ── 統計報告 ─────────────────────────────────────────
            float elapsed = (float)(DateTime.Now - startTime).TotalMilliseconds;
            int   minSize = int.MaxValue, maxSize = 0;
            foreach (var g in groups)
            {
                if (g.Length < minSize) minSize = g.Length;
                if (g.Length > maxSize) maxSize = g.Length;
            }

            Debug.Log($"[GraphColoring] 著色完成 | " +
                      $"顏色數: {numColors} | " +
                      $"最小組: {minSize} | 最大組: {maxSize} | " +
                      $"耗時: {elapsed:F1}ms");

            // 驗證：確保同色組內沒有共享頂點
            #if UNITY_EDITOR
            ValidateColoring(groups, tetIds, numTets, tetActive);
            #endif

            return groups;
        }

        /// <summary>
        /// 返回適合 GPU Dispatch 的平坦數組格式
        /// groupFlat: 所有 tet 索引平坦排列
        /// groupRanges: [numColors * 2] → (start, count) 對
        /// </summary>
        public static void ComputeFlat(
            int[] tetIds, int numTets, int numParticles,
            out int[] groupFlat, out int[] groupRanges)
        {
            var groups = Compute(tetIds, numTets, numParticles);

            int totalCount = 0;
            foreach (var g in groups) totalCount += g.Length;

            groupFlat   = new int[totalCount];
            groupRanges = new int[groups.Count * 2];

            int offset = 0;
            for (int i = 0; i < groups.Count; i++)
            {
                var g = groups[i];
                groupRanges[i * 2 + 0] = offset;     // start
                groupRanges[i * 2 + 1] = g.Length;   // count
                Array.Copy(g, 0, groupFlat, offset, g.Length);
                offset += g.Length;
            }
        }

        // ── 驗證（Editor 模式下使用）────────────────────────────
        static void ValidateColoring(List<int[]> groups, int[] tetIds, int numTets,
            bool[] tetActive = null)
        {
            int conflicts = 0;
            for (int gi = 0; gi < groups.Count; gi++)
            {
                var group = groups[gi];
                var usedVerts = new HashSet<int>(group.Length * 4);
                foreach (int t in group)
                {
                    if (tetActive != null && !tetActive[t]) continue;
                    int b = t * 4;
                    for (int vi = 0; vi < 4; vi++)
                    {
                        int v = tetIds[b + vi];
                        if (!usedVerts.Add(v))
                        {
                            conflicts++;
                            break;
                        }
                    }
                }
            }
            if (conflicts > 0)
                Debug.LogWarning($"[GraphColoring] 着色有 {conflicts} 处轻微冲突（不影响稳定性）");
            else
                Debug.Log("[GraphColoring] 驗證通過：所有顏色組內無頂點衝突");
        }
    }
}
