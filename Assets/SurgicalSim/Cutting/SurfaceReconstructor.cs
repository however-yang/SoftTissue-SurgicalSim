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

        static readonly int[] FaceOpposite = { 3, 2, 1, 0 };

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
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);

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
        /// Rebuild all active tet boundary faces once, then classify them for
        /// rendering. This keeps the renderer's source of truth identical to
        /// RebuildSurface(): every count==1 boundary face is emitted exactly
        /// once, so missed cut-face metadata can only affect material choice,
        /// not create visual holes.
        /// </summary>
        public static void RebuildBoundarySurfaceSplitByCut(
            TetMeshData data,
            CutFaceRegistry cutFaceRegistry,
            HashSet<int> cutSurfaceVerts,
            HashSet<long> cutSurfaceEdges,
            List<int> originalTris,
            List<int> cutTris,
            IList<Vector3> cutSurfaceNormals = null,
            float normalDotThreshold = 0.5f)
        {
            originalTris?.Clear();
            cutTris?.Clear();
            if (data == null || originalTris == null || cutTris == null) return;

            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);
                    if (FaceAreaSq(data, a, b, c) < 1e-16f) continue;

                    long key = FaceKey(a, b, c);
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

            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;

                var (a, b, c) = faceOrder[kv.Key];
                List<int> dst = IsBoundaryCutFace(
                    data,
                    kv.Key,
                    a,
                    b,
                    c,
                    cutFaceRegistry,
                    cutSurfaceVerts,
                    cutSurfaceEdges,
                    cutSurfaceNormals,
                    normalDotThreshold)
                    ? cutTris
                    : originalTris;

                dst.Add(a);
                dst.Add(b);
                dst.Add(c);
            }
        }

        public static int[] RebuildGeneratedBoundarySurface(
            TetMeshData data,
            HashSet<int> generatedVerts)
        {
            if (data == null || generatedVerts == null || generatedVerts.Count == 0)
                return System.Array.Empty<int>();

            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);

                    if (!generatedVerts.Contains(a) ||
                        !generatedVerts.Contains(b) ||
                        !generatedVerts.Contains(c))
                    {
                        continue;
                    }

                    long key = FaceKey(a, b, c);
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

            var tris = new List<int>();
            var emittedGeometry = new HashSet<GeometryFaceKey>();
            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;
                var (a, b, c) = faceOrder[kv.Key];
                if (!emittedGeometry.Add(MakeGeometryFaceKey(data, a, b, c)))
                    continue;
                tris.Add(a);
                tris.Add(b);
                tris.Add(c);
            }

            return tris.ToArray();
        }

        public static int[] RebuildCutBoundarySurface(
            TetMeshData data,
            HashSet<int> cutSurfaceVerts,
            IList<Vector3> cutSurfaceNormals,
            HashSet<long> includedFaceKeys = null,
            float normalDotThreshold = 0.8f)
        {
            if (data == null || cutSurfaceVerts == null || cutSurfaceVerts.Count == 0)
                return System.Array.Empty<int>();

            bool useExplicitFaceKeys = includedFaceKeys != null && includedFaceKeys.Count > 0;
            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);
                    long key = FaceKey(a, b, c);

                    bool isCutFace = useExplicitFaceKeys
                        ? includedFaceKeys.Contains(key)
                        : IsCutFace(data, a, b, c, cutSurfaceVerts, cutSurfaceNormals, normalDotThreshold);
                    if (!isCutFace) continue;

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

            var tris = new List<int>();
            var emittedGeometry = new HashSet<GeometryFaceKey>();
            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;
                var (a, b, c) = faceOrder[kv.Key];
                if (!emittedGeometry.Add(MakeGeometryFaceKey(data, a, b, c)))
                    continue;
                tris.Add(a);
                tris.Add(b);
                tris.Add(c);
            }

            return tris.ToArray();
        }

        public static int[] RebuildCutBoundarySurface(
            TetMeshData data,
            CutFaceRegistry cutFaceRegistry)
        {
            if (data == null || cutFaceRegistry == null || cutFaceRegistry.CutFaceCount == 0)
                return System.Array.Empty<int>();

            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);
                    long key = FaceKey(a, b, c);
                    if (!cutFaceRegistry.IsCutFace(key)) continue;

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

            var tris = new List<int>();
            var emittedGeometry = new HashSet<GeometryFaceKey>();
            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;
                var (a, b, c) = faceOrder[kv.Key];
                if (!emittedGeometry.Add(MakeGeometryFaceKey(data, a, b, c)))
                    continue;
                tris.Add(a);
                tris.Add(b);
                tris.Add(c);
            }

            return tris.ToArray();
        }

        public static int[] RebuildSurfaceExcludingCutFaces(
            TetMeshData data,
            HashSet<int> cutSurfaceVerts,
            IList<Vector3> cutSurfaceNormals,
            float normalDotThreshold = 0.8f,
            HashSet<long> excludedFaceKeys = null,
            HashSet<int> originalSurfaceVerts = null,
            bool requireOriginalSurfaceAnchor = false,
            int originalVertexCount = -1,
            bool excludeAllNewVertexFaces = false)
        {
            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);

                    if (excludeAllNewVertexFaces &&
                        originalVertexCount >= 0 &&
                        a >= originalVertexCount &&
                        b >= originalVertexCount &&
                        c >= originalVertexCount)
                    {
                        continue;
                    }

                    if (requireOriginalSurfaceAnchor &&
                        originalSurfaceVerts != null &&
                        originalSurfaceVerts.Count > 0 &&
                        !originalSurfaceVerts.Contains(a) &&
                        !originalSurfaceVerts.Contains(b) &&
                        !originalSurfaceVerts.Contains(c))
                    {
                        continue;
                    }

                    if (FaceAreaSq(data, a, b, c) < 1e-16f)
                        continue;

                    long key = FaceKey(a, b, c);
                    if (excludedFaceKeys != null && excludedFaceKeys.Contains(key))
                        continue;

                    if (IsCutFace(data, a, b, c, cutSurfaceVerts, cutSurfaceNormals, normalDotThreshold))
                        continue;

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

            var tris = new List<int>();
            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;
                var (a, b, c) = faceOrder[kv.Key];
                tris.Add(a);
                tris.Add(b);
                tris.Add(c);
            }

            return tris.ToArray();
        }

        public static int[] RebuildRootedOriginalSurface(
            TetMeshData data,
            IList<int> surfaceRootIds,
            HashSet<long> originalSurfaceFaceKeys,
            HashSet<long> excludedFaceKeys = null)
        {
            if (data == null ||
                surfaceRootIds == null ||
                originalSurfaceFaceKeys == null ||
                originalSurfaceFaceKeys.Count == 0)
            {
                return data != null ? RebuildSurface(data, excludedFaceKeys: excludedFaceKeys) : System.Array.Empty<int>();
            }

            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);
                    if (FaceAreaSq(data, a, b, c) < 1e-16f) continue;

                    if (!TryOriginalSurfaceFaceKey(surfaceRootIds, a, b, c, out long rootFaceKey))
                        continue;
                    if (!originalSurfaceFaceKeys.Contains(rootFaceKey))
                        continue;

                    long key = FaceKey(a, b, c);
                    if (excludedFaceKeys != null && excludedFaceKeys.Contains(key))
                        continue;

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

            var tris = new List<int>();
            var emittedGeometry = new HashSet<GeometryFaceKey>();
            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;
                var (a, b, c) = faceOrder[kv.Key];
                if (!emittedGeometry.Add(MakeGeometryFaceKey(data, a, b, c)))
                    continue;
                tris.Add(a);
                tris.Add(b);
                tris.Add(c);
            }

            return tris.ToArray();
        }

        public static int[] RebuildOriginalSurfaceFromSupports(
            TetMeshData data,
            IList<int> surfaceSupport0,
            IList<int> surfaceSupport1,
            IList<int> surfaceSupport2,
            HashSet<long> originalSurfaceFaceKeys,
            HashSet<long> excludedFaceKeys = null,
            CutFaceRegistry faceRegistry = null,
            HashSet<int> cutSurfaceVertexIds = null)
        {
            if (data == null ||
                surfaceSupport0 == null ||
                surfaceSupport1 == null ||
                surfaceSupport2 == null ||
                originalSurfaceFaceKeys == null ||
                originalSurfaceFaceKeys.Count == 0)
            {
                return System.Array.Empty<int>();
            }

            var faceCount = new Dictionary<long, int>();
            var faceOrder = new Dictionary<long, (int a, int b, int c)>();

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];
                int[] tv = { i0, i1, i2, i3 };

                for (int f = 0; f < 4; f++)
                {
                    GetCurrentOrientedFace(data, tv, f, out int a, out int b, out int c);
                    if (FaceAreaSq(data, a, b, c) < 1e-16f) continue;

                    if (!TrySupportedOriginalSurfaceFaceKey(
                        surfaceSupport0,
                        surfaceSupport1,
                        surfaceSupport2,
                        a,
                        b,
                        c,
                        out long supportFaceKey,
                        out int supportA,
                        out int supportB,
                        out int supportC))
                    {
                        continue;
                    }

                    if (!originalSurfaceFaceKeys.Contains(supportFaceKey))
                        continue;
                    if (!FaceLiesOnOriginalSurfaceTriangle(
                        data,
                        a,
                        b,
                        c,
                        supportA,
                        supportB,
                        supportC))
                    {
                        continue;
                    }
                    if (!FaceHasRenderableCurrentShape(data, a, b, c))
                        continue;

                    long key = FaceKey(a, b, c);
                    if (excludedFaceKeys != null && excludedFaceKeys.Contains(key))
                        continue;
                    if (faceRegistry != null && faceRegistry.ClassifyFace(key) == SurfaceFaceClass.CutSurface)
                        continue;
                    if (AllVerticesInSet(cutSurfaceVertexIds, a, b, c))
                        continue;

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

            var tris = new List<int>();
            var emittedGeometry = new HashSet<GeometryFaceKey>();
            foreach (var kv in faceCount)
            {
                if (kv.Value != 1) continue;
                var (a, b, c) = faceOrder[kv.Key];
                if (!emittedGeometry.Add(MakeGeometryFaceKey(data, a, b, c)))
                    continue;
                tris.Add(a);
                tris.Add(b);
                tris.Add(c);
            }

            return tris.ToArray();
        }

        static bool AllVerticesInSet(HashSet<int> vertices, int a, int b, int c)
        {
            return vertices != null &&
                   vertices.Contains(a) &&
                   vertices.Contains(b) &&
                   vertices.Contains(c);
        }

        static bool IsBoundaryCutFace(
            TetMeshData data,
            long faceKey,
            int a,
            int b,
            int c,
            CutFaceRegistry cutFaceRegistry,
            HashSet<int> cutSurfaceVerts,
            HashSet<long> cutSurfaceEdges,
            IList<Vector3> cutSurfaceNormals,
            float normalDotThreshold)
        {
            if (cutFaceRegistry != null && cutFaceRegistry.IsCutFace(faceKey))
                return true;
            if (cutSurfaceVerts == null || cutSurfaceVerts.Count == 0)
                return false;

            bool va = cutSurfaceVerts.Contains(a);
            bool vb = cutSurfaceVerts.Contains(b);
            bool vc = cutSurfaceVerts.Contains(c);
            int cutVertexCount = (va ? 1 : 0) + (vb ? 1 : 0) + (vc ? 1 : 0);
            if (cutVertexCount < 3) return false;

            if (cutSurfaceEdges == null || cutSurfaceEdges.Count == 0)
                return false;

            int cutEdgeCount = 0;
            if (cutSurfaceEdges.Contains(EdgeKey(a, b))) cutEdgeCount++;
            if (cutSurfaceEdges.Contains(EdgeKey(b, c))) cutEdgeCount++;
            if (cutSurfaceEdges.Contains(EdgeKey(c, a))) cutEdgeCount++;
            if (cutEdgeCount < 3) return false;

            // Registry can miss a cloned cut face, but "all three vertices are
            // cut vertices" is too broad: outer boundary faces near the rim can
            // satisfy that and then get rendered with the interior material.
            // Require the full cut-edge loop and, when available, alignment with
            // the committed cut normals.
            if (cutSurfaceNormals == null || cutSurfaceNormals.Count == 0)
                return true;
            return IsCutFace(data, a, b, c, cutSurfaceVerts, cutSurfaceNormals, normalDotThreshold);
        }

        static bool IsCutFace(
            TetMeshData data,
            int a,
            int b,
            int c,
            HashSet<int> cutSurfaceVerts,
            IList<Vector3> cutSurfaceNormals,
            float normalDotThreshold)
        {
            if (cutSurfaceVerts == null || cutSurfaceNormals == null) return false;
            if (!cutSurfaceVerts.Contains(a) ||
                !cutSurfaceVerts.Contains(b) ||
                !cutSurfaceVerts.Contains(c))
            {
                return false;
            }

            Vector3 pa = data.Positions[a];
            Vector3 pb = data.Positions[b];
            Vector3 pc = data.Positions[c];
            Vector3 n = Vector3.Cross(pb - pa, pc - pa);
            if (n.sqrMagnitude < 1e-12f) return true;
            n.Normalize();

            float threshold = Mathf.Clamp01(normalDotThreshold);
            for (int i = 0; i < cutSurfaceNormals.Count; i++)
            {
                Vector3 cn = cutSurfaceNormals[i];
                if (cn.sqrMagnitude < 1e-10f) continue;
                cn.Normalize();
                if (Mathf.Abs(Vector3.Dot(n, cn)) >= threshold)
                    return true;
            }

            return false;
        }

        static bool TryOriginalSurfaceFaceKey(
            IList<int> surfaceRootIds,
            int a,
            int b,
            int c,
            out long key)
        {
            key = 0L;
            int ra = SurfaceRootOf(surfaceRootIds, a);
            int rb = SurfaceRootOf(surfaceRootIds, b);
            int rc = SurfaceRootOf(surfaceRootIds, c);
            if (ra < 0 || rb < 0 || rc < 0) return false;
            if (ra == rb || ra == rc || rb == rc) return false;

            key = FaceKey(ra, rb, rc);
            return true;
        }

        static int SurfaceRootOf(IList<int> surfaceRootIds, int vertex)
        {
            if (vertex < 0 || vertex >= surfaceRootIds.Count) return -1;
            return surfaceRootIds[vertex];
        }

        static bool TrySupportedOriginalSurfaceFaceKey(
            IList<int> surfaceSupport0,
            IList<int> surfaceSupport1,
            IList<int> surfaceSupport2,
            int a,
            int b,
            int c,
            out long key,
            out int r0,
            out int r1,
            out int r2)
        {
            key = 0L;
            r0 = -1;
            r1 = -1;
            r2 = -1;
            int count = 0;

            if (!AppendVertexSupports(surfaceSupport0, surfaceSupport1, surfaceSupport2, a, ref r0, ref r1, ref r2, ref count))
                return false;
            if (!AppendVertexSupports(surfaceSupport0, surfaceSupport1, surfaceSupport2, b, ref r0, ref r1, ref r2, ref count))
                return false;
            if (!AppendVertexSupports(surfaceSupport0, surfaceSupport1, surfaceSupport2, c, ref r0, ref r1, ref r2, ref count))
                return false;

            if (count != 3) return false;
            key = FaceKey(r0, r1, r2);
            return true;
        }

        static bool FaceLiesOnOriginalSurfaceTriangle(
            TetMeshData data,
            int a,
            int b,
            int c,
            int s0,
            int s1,
            int s2)
        {
            if (data == null ||
                data.RestPositions == null ||
                s0 < 0 || s1 < 0 || s2 < 0 ||
                s0 >= data.RestPositions.Length ||
                s1 >= data.RestPositions.Length ||
                s2 >= data.RestPositions.Length ||
                a >= data.RestPositions.Length ||
                b >= data.RestPositions.Length ||
                c >= data.RestPositions.Length)
            {
                return false;
            }

            Vector3 p0 = data.RestPositions[s0];
            Vector3 p1 = data.RestPositions[s1];
            Vector3 p2 = data.RestPositions[s2];
            Vector3 n = Vector3.Cross(p1 - p0, p2 - p0);
            float area2 = n.magnitude;
            if (area2 < 1e-10f) return false;
            n /= area2;

            float longest = Mathf.Max(
                (p1 - p0).magnitude,
                Mathf.Max((p2 - p1).magnitude, (p0 - p2).magnitude));
            float planeTol = Mathf.Max(1e-6f, longest * 1e-4f);

            if (Mathf.Abs(Vector3.Dot(data.RestPositions[a] - p0, n)) > planeTol) return false;
            if (Mathf.Abs(Vector3.Dot(data.RestPositions[b] - p0, n)) > planeTol) return false;
            if (Mathf.Abs(Vector3.Dot(data.RestPositions[c] - p0, n)) > planeTol) return false;

            const float baryTol = -1e-3f;
            return PointInsideTriangleRest(data.RestPositions[a], p0, p1, p2, baryTol) &&
                   PointInsideTriangleRest(data.RestPositions[b], p0, p1, p2, baryTol) &&
                   PointInsideTriangleRest(data.RestPositions[c], p0, p1, p2, baryTol);
        }

        static bool PointInsideTriangleRest(
            Vector3 p,
            Vector3 a,
            Vector3 b,
            Vector3 c,
            float tolerance)
        {
            Vector3 v0 = b - a;
            Vector3 v1 = c - a;
            Vector3 v2 = p - a;

            float d00 = Vector3.Dot(v0, v0);
            float d01 = Vector3.Dot(v0, v1);
            float d11 = Vector3.Dot(v1, v1);
            float d20 = Vector3.Dot(v2, v0);
            float d21 = Vector3.Dot(v2, v1);
            float denom = d00 * d11 - d01 * d01;
            if (Mathf.Abs(denom) < 1e-12f) return false;

            float v = (d11 * d20 - d01 * d21) / denom;
            float w = (d00 * d21 - d01 * d20) / denom;
            float u = 1f - v - w;
            return u >= tolerance && v >= tolerance && w >= tolerance;
        }

        static bool AppendVertexSupports(
            IList<int> surfaceSupport0,
            IList<int> surfaceSupport1,
            IList<int> surfaceSupport2,
            int vertex,
            ref int r0,
            ref int r1,
            ref int r2,
            ref int count)
        {
            bool any = false;
            if (!AppendSupport(SupportOf(surfaceSupport0, vertex), ref r0, ref r1, ref r2, ref count, ref any))
                return false;
            if (!AppendSupport(SupportOf(surfaceSupport1, vertex), ref r0, ref r1, ref r2, ref count, ref any))
                return false;
            if (!AppendSupport(SupportOf(surfaceSupport2, vertex), ref r0, ref r1, ref r2, ref count, ref any))
                return false;
            return any;
        }

        static int SupportOf(IList<int> supports, int vertex)
        {
            if (supports == null || vertex < 0 || vertex >= supports.Count) return -1;
            return supports[vertex];
        }

        static bool AppendSupport(
            int root,
            ref int r0,
            ref int r1,
            ref int r2,
            ref int count,
            ref bool any)
        {
            if (root < 0) return true;
            any = true;
            if ((count > 0 && r0 == root) ||
                (count > 1 && r1 == root) ||
                (count > 2 && r2 == root))
            {
                return true;
            }

            if (count >= 3) return false;
            if (count == 0) r0 = root;
            else if (count == 1) r1 = root;
            else r2 = root;
            count++;
            return true;
        }

        static float FaceAreaSq(TetMeshData data, int a, int b, int c)
        {
            Vector3 pa = data.Positions[a];
            Vector3 pb = data.Positions[b];
            Vector3 pc = data.Positions[c];
            return Vector3.Cross(pb - pa, pc - pa).sqrMagnitude * 0.25f;
        }

        static bool FaceHasRenderableCurrentShape(TetMeshData data, int a, int b, int c)
        {
            Vector3 pa = data.Positions[a];
            Vector3 pb = data.Positions[b];
            Vector3 pc = data.Positions[c];

            Vector3 ab = pb - pa;
            Vector3 ac = pc - pa;
            Vector3 bc = pc - pb;
            float maxEdgeSq = Mathf.Max(ab.sqrMagnitude, Mathf.Max(ac.sqrMagnitude, bc.sqrMagnitude));
            if (maxEdgeSq < 1e-14f) return false;

            float crossSq = Vector3.Cross(ab, ac).sqrMagnitude;
            if (crossSq < 1e-16f) return false;

            // Suppress needle-like visual triangles from duplicated boundary
            // support faces. They are the usual source of flickering dark
            // shards on otherwise valid outer liver surface.
            return crossSq >= maxEdgeSq * maxEdgeSq * 1e-5f;
        }

        static GeometryFaceKey MakeGeometryFaceKey(TetMeshData data, int a, int b, int c)
        {
            return new GeometryFaceKey(
                QuantizedPoint(data.Positions[a]),
                QuantizedPoint(data.Positions[b]),
                QuantizedPoint(data.Positions[c]));
        }

        static PointKey QuantizedPoint(Vector3 p)
        {
            const float invTol = 100000f;
            return new PointKey(
                Mathf.RoundToInt(p.x * invTol),
                Mathf.RoundToInt(p.y * invTol),
                Mathf.RoundToInt(p.z * invTol));
        }

        struct GeometryFaceKey : System.IEquatable<GeometryFaceKey>
        {
            readonly PointKey a;
            readonly PointKey b;
            readonly PointKey c;

            public GeometryFaceKey(PointKey p0, PointKey p1, PointKey p2)
            {
                if (Compare(p1, p0) < 0) Swap(ref p0, ref p1);
                if (Compare(p2, p1) < 0) Swap(ref p1, ref p2);
                if (Compare(p1, p0) < 0) Swap(ref p0, ref p1);

                a = p0;
                b = p1;
                c = p2;
            }

            public bool Equals(GeometryFaceKey other)
            {
                return a.Equals(other.a) && b.Equals(other.b) && c.Equals(other.c);
            }

            public override bool Equals(object obj)
            {
                return obj is GeometryFaceKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = a.GetHashCode();
                    hash = hash * 397 ^ b.GetHashCode();
                    hash = hash * 397 ^ c.GetHashCode();
                    return hash;
                }
            }

            static int Compare(PointKey x, PointKey y)
            {
                if (x.x != y.x) return x.x < y.x ? -1 : 1;
                if (x.y != y.y) return x.y < y.y ? -1 : 1;
                if (x.z != y.z) return x.z < y.z ? -1 : 1;
                return 0;
            }

            static void Swap(ref PointKey x, ref PointKey y)
            {
                PointKey tmp = x;
                x = y;
                y = tmp;
            }
        }

        struct PointKey : System.IEquatable<PointKey>
        {
            public readonly int x;
            public readonly int y;
            public readonly int z;

            public PointKey(int x, int y, int z)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }

            public bool Equals(PointKey other)
            {
                return x == other.x && y == other.y && z == other.z;
            }

            public override bool Equals(object obj)
            {
                return obj is PointKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = x;
                    hash = hash * 397 ^ y;
                    hash = hash * 397 ^ z;
                    return hash;
                }
            }
        }

        static void GetCurrentOrientedFace(
            TetMeshData data,
            int[] tetVerts,
            int faceIndex,
            out int a,
            out int b,
            out int c)
        {
            a = tetVerts[FaceLocal[faceIndex, 0]];
            b = tetVerts[FaceLocal[faceIndex, 1]];
            c = tetVerts[FaceLocal[faceIndex, 2]];

            int opposite = tetVerts[FaceOpposite[faceIndex]];
            Vector3 pa = data.Positions[a];
            Vector3 pb = data.Positions[b];
            Vector3 pc = data.Positions[c];
            Vector3 po = data.Positions[opposite];

            Vector3 n = Vector3.Cross(pb - pa, pc - pa);
            if (n.sqrMagnitude < 1e-12f) return;

            Vector3 faceCenter = (pa + pb + pc) / 3f;
            if (Vector3.Dot(n, faceCenter - po) >= 0f) return;

            int tmp = b;
            b = c;
            c = tmp;
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
        public static long FaceKey(int a, int b, int c)
        {
            // 排序三个顶点索引
            int mn = Mathf.Min(a, Mathf.Min(b, c));
            int mx = Mathf.Max(a, Mathf.Max(b, c));
            int md = a + b + c - mn - mx;

            // 打包成 long（假设顶点数 < 1M）
            return (long)mn * 1000000L * 1000000L + (long)md * 1000000L + mx;
        }

        public static long EdgeKey(int a, int b)
        {
            int mn = Mathf.Min(a, b);
            int mx = Mathf.Max(a, b);
            return (long)mn * 1000000L + mx;
        }
    }
}
