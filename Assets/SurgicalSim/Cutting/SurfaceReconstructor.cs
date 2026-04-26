// SurfaceReconstructor.cs
// 从活跃四面体中提取边界三角形
// 边界面 = 只属于一个活跃 tet 的三角形面

using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim.Cutting
{
    public class SurfaceReconstructor
    {
        // tet 面的局部顶点索引（4个面，每面3个顶点）
        // 法线朝外的顺序
        static readonly int[,] FaceLocal = {
            {0, 2, 1},
            {0, 1, 3},
            {0, 3, 2},
            {1, 2, 3}
        };

        /// <summary>
        /// 全量重建：从所有活跃 tet 中提取边界三角形
        /// 返回 flat int array（每3个一组）
        /// </summary>
        public static int[] RebuildSurface(TetMeshData data,
            HashSet<int> generatedVerts = null,
            bool excludeAllGeneratedFaces = false,
            HashSet<long> excludedFaceKeys = null)
        {
            // face key -> (count, ordered vertex indices)
            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t*4+0];
                int i1 = data.TetIds[t*4+1];
                int i2 = data.TetIds[t*4+2];
                int i3 = data.TetIds[t*4+3];
                int[] tv = {i0, i1, i2, i3};

                for (int f = 0; f < 4; f++)
                {
                    int a = tv[FaceLocal[f,0]];
                    int b = tv[FaceLocal[f,1]];
                    int c = tv[FaceLocal[f,2]];

                    long key = FaceKey(a, b, c);

                    if (excludedFaceKeys != null && excludedFaceKeys.Contains(key))
                    {
                        continue;
                    }

                    if (excludeAllGeneratedFaces && generatedVerts != null &&
                        generatedVerts.Contains(a) &&
                        generatedVerts.Contains(b) &&
                        generatedVerts.Contains(c))
                    {
                        continue;
                    }

                    if (faceCount.ContainsKey(key))
                    {
                        faceCount[key]++;
                    }
                    else
                    {
                        faceCount[key] = 1;
                        faceOrder[key] = (a, b, c);
                    }
                }
            }

            // 边界面 = count == 1
            var tris = new List<int>();
            foreach (var kv in faceCount)
            {
                if (kv.Value == 1)
                {
                    var (a, b, c) = faceOrder[kv.Key];
                    tris.Add(a);
                    tris.Add(b);
                    tris.Add(c);
                }
            }

            return tris.ToArray();
        }

        /// <summary>
        /// 带切口内表面的重建: 在边界面基础上追加切口三角形
        /// </summary>
        public static int[] RebuildSurfaceWithCutFaces(TetMeshData data, List<int> cutSurfaceTris)
        {
            int[] baseTris = RebuildSurface(data);
            if (cutSurfaceTris == null || cutSurfaceTris.Count == 0)
                return baseTris;

            // 合并
            var result = new int[baseTris.Length + cutSurfaceTris.Count];
            System.Array.Copy(baseTris, result, baseTris.Length);
            cutSurfaceTris.CopyTo(result, baseTris.Length);
            return result;
        }

        /// <summary>
        /// 增量更新：当 tet 被删除时，只更新受影响区域的表面
        /// removedTets: 本次被删除的 tet 列表
        /// 返回是否需要全量重建
        /// </summary>
        public static bool IncrementalUpdate(TetMeshData data,
            List<int> removedTets,
            ref int[] currentSurfaceTriIds)
        {
            // 对于小规模删除（< 100 tet），增量更新更快
            // 对于大规模删除，全量重建更简单可靠
            if (removedTets.Count > 100)
            {
                currentSurfaceTriIds = RebuildSurface(data);
                return true;
            }

            // 收集受影响的顶点
            var affectedVerts = new HashSet<int>();
            foreach (int t in removedTets)
            {
                affectedVerts.Add(data.TetIds[t*4+0]);
                affectedVerts.Add(data.TetIds[t*4+1]);
                affectedVerts.Add(data.TetIds[t*4+2]);
                affectedVerts.Add(data.TetIds[t*4+3]);
            }

            // 简单方案：全量重建（对 12k tet 大约 1-2ms）
            currentSurfaceTriIds = RebuildSurface(data);
            return true;
        }

        /// <summary>
        /// 生成排序后的面标识 key（用于去重）
        /// </summary>
        static long FaceKey(int a, int b, int c)
        {
            // 排序三个顶点索引
            int mn = Mathf.Min(a, Mathf.Min(b, c));
            int mx = Mathf.Max(a, Mathf.Max(b, c));
            int md = a + b + c - mn - mx;

            // 打包成 long（假设顶点数 < 1M）
            return (long)mn * 1000000L * 1000000L + (long)md * 1000000L + mx;
        }
    }
}
