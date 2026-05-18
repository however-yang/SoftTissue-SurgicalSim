// CutSurfaceRenderer.cs
// 切面内部组织渲染器
// 在切割处生成一个独立的 Mesh 来渲染内部组织表面
// 支持：累积多次切割、跟随物理变形

using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

namespace SurgicalSim.Cutting
{
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public class CutSurfaceRenderer : MonoBehaviour
    {
        [Header("切面材质")]
        public Color cutSurfaceColor = new Color(0.30f, 0.025f, 0.018f, 1f);
        public float smoothness = 0.66f;
        public float metallic = 0.0f;
        [Tooltip("Visual vertices on the cut surface are shared only when their patch normals are this close. Keeps straight cuts smooth while preserving sharp S/turn transitions.")]
        public float normalReuseAngleDeg = 18f;
        [Tooltip("Runtime visual guard. Triangles with an edge longer than this are hidden so stale cut patches cannot be dragged into long ribbons. 0 disables it.")]
        public float maxRuntimeEdgeLength = 0.25f;

        MeshFilter _meshFilter;
        MeshRenderer _meshRenderer;
        Mesh _cutMesh;

        // 累积的切面几何数据
        List<Vector3> _allVerts = new List<Vector3>();
        List<int>     _allTris  = new List<int>();
        List<Vector3> _allNormals = new List<Vector3>();
        List<int>     _particleIndices = new List<int>();
        Dictionary<int, List<int>> _particleToVerts = new Dictionary<int, List<int>>();
        readonly List<int> _visibleTris = new List<int>();
        readonly List<Vector3> _runtimeNormals = new List<Vector3>();

        // 每个切面顶点的 tet 嵌入信息（用于跟随变形）
        // embedTetIdx: 顶点所在的 tet 索引
        // embedWeights: 重心坐标 (w0,w1,w2,w3)
        public struct EmbedInfo
        {
            public int tetIdx;
            public Vector4 weights; // 重心坐标
        }
        List<EmbedInfo> _embedInfos = new List<EmbedInfo>();

        void Awake()
        {
            _meshFilter = GetComponent<MeshFilter>();
            _meshRenderer = GetComponent<MeshRenderer>();

            _cutMesh = new Mesh();
            _cutMesh.name = "CutSurface";
            _meshFilter.mesh = _cutMesh;

            SetupMaterial();
        }

        void SetupMaterial()
        {
            // 使用双面渲染的 shader
            var shader = Shader.Find("SurgicalSim/LiverCutSurfaceGPU");
            if (shader == null)
                shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null)
                shader = Shader.Find("Universal Render Pipeline/Unlit");
            if (shader == null)
                shader = Shader.Find("Unlit/Color");
            if (shader == null)
                shader = Shader.Find("Sprites/Default");
            if (shader == null)
                shader = Shader.Find("Standard");

            if (shader != null)
            {
                var mat = new Material(shader);
                mat.name = "Runtime_CutSurfaceWetTissue";
                if (mat.HasProperty("_CutColor"))
                    mat.SetColor("_CutColor", cutSurfaceColor);
                if (mat.HasProperty("_Color"))
                    mat.SetColor("_Color", cutSurfaceColor);
                if (mat.HasProperty("_BaseColor"))
                    mat.SetColor("_BaseColor", cutSurfaceColor);
                if (mat.HasProperty("_InteriorColor"))
                    mat.SetColor("_InteriorColor", cutSurfaceColor);
                if (mat.HasProperty("_SpecColor"))
                    mat.SetColor("_SpecColor", Color.black);
                if (mat.HasProperty("_TextureStrength"))
                    mat.SetFloat("_TextureStrength", 0f);
                if (mat.HasProperty("_Roughness"))
                    mat.SetFloat("_Roughness", Mathf.Clamp01(1f - smoothness));
                if (mat.HasProperty("_Smoothness"))
                    mat.SetFloat("_Smoothness", smoothness);
                if (mat.HasProperty("_Glossiness"))
                    mat.SetFloat("_Glossiness", smoothness);
                if (mat.HasProperty("_Metallic"))
                    mat.SetFloat("_Metallic", metallic);
                if (mat.HasProperty("_Wetness"))
                    mat.SetFloat("_Wetness", 0.78f);
                if (mat.HasProperty("_SpecularStrength"))
                    mat.SetFloat("_SpecularStrength", 0.85f);
                if (mat.HasProperty("_FiberIntensity"))
                    mat.SetFloat("_FiberIntensity", 0.34f);
                if (mat.HasProperty("_RimDarkening"))
                    mat.SetFloat("_RimDarkening", 0.18f);
                mat.doubleSidedGI = true;

                // 关闭背面剔除
                if (mat.HasProperty("_Cull"))
                    mat.SetFloat("_Cull", 0); // Off
                mat.SetOverrideTag("RenderType", "Opaque");
                mat.renderQueue = (int)RenderQueue.Geometry;
                if (mat.HasProperty("_Surface"))
                    mat.SetFloat("_Surface", 0f);
                if (mat.HasProperty("_Blend"))
                    mat.SetFloat("_Blend", 0f);
                if (mat.HasProperty("_ZWrite"))
                    mat.SetFloat("_ZWrite", 1f);
                _meshRenderer.material = mat;
                _meshRenderer.shadowCastingMode = ShadowCastingMode.Off;
                _meshRenderer.receiveShadows = true;
            }
        }

        /// <summary>
        /// 添加新的切面几何（从 VNACutResult 获取）
        /// </summary>
        public void AddCutSurface(VNACutResult result)
        {
            if (result.CutSurfaceVerts == null || result.CutSurfaceVerts.Count == 0)
                return;

            int baseIdx = _allVerts.Count;
            _allVerts.AddRange(result.CutSurfaceVerts);
            _allNormals.AddRange(result.CutSurfaceNormals);
            for (int i = 0; i < result.CutSurfaceVerts.Count; i++)
                _particleIndices.Add(-1);

            foreach (int idx in result.CutSurfaceTris)
                _allTris.Add(idx + baseIdx);

            RebuildMesh();
        }

        /// <summary>
        /// 添加切面几何并附带嵌入信息
        /// </summary>
        public void AddCutSurfaceWithEmbed(List<Vector3> verts, List<int> tris,
            List<Vector3> normals, List<EmbedInfo> embeds)
        {
            int baseIdx = _allVerts.Count;
            _allVerts.AddRange(verts);
            _allNormals.AddRange(normals);
            for (int i = 0; i < verts.Count; i++)
                _particleIndices.Add(-1);
            _embedInfos.AddRange(embeds);

            foreach (int idx in tris)
                _allTris.Add(idx + baseIdx);

            RebuildMesh();
        }

        /// <summary>
        /// 添加直接绑定到粒子索引的切面 patch。
        /// 适合 V3 细分切割：切面顶点本身就是求解粒子。
        /// </summary>
        public void AddParticlePatch(Core.TetMeshData data,
            IList<int> particleIds, Vector3 normal, bool reverseWinding,
            bool rebuildMesh = true)
        {
            if (data == null || particleIds == null) return;
            if (particleIds.Count < 3 || particleIds.Count > 4) return;

            Vector3 patchNormal = reverseWinding ? -normal : normal;
            int[] patchIndices = new int[particleIds.Count];

            for (int i = 0; i < particleIds.Count; i++)
            {
                int pid = particleIds[i];
                if (pid < 0 || pid >= data.NumParticles) return;

                int existingIndex = FindReusableParticleVertex(pid, patchNormal);
                if (existingIndex >= 0)
                {
                    patchIndices[i] = existingIndex;
                    _allNormals[existingIndex] += patchNormal;
                    continue;
                }

                int newIndex = _allVerts.Count;
                if (!_particleToVerts.TryGetValue(pid, out var group))
                {
                    group = new List<int>(2);
                    _particleToVerts[pid] = group;
                }
                group.Add(newIndex);
                patchIndices[i] = newIndex;
                _allVerts.Add(data.Positions[pid]);
                _allNormals.Add(patchNormal);
                _particleIndices.Add(pid);
            }

            if (particleIds.Count == 3)
            {
                if (reverseWinding)
                {
                    _allTris.Add(patchIndices[0]);
                    _allTris.Add(patchIndices[2]);
                    _allTris.Add(patchIndices[1]);
                }
                else
                {
                    _allTris.Add(patchIndices[0]);
                    _allTris.Add(patchIndices[1]);
                    _allTris.Add(patchIndices[2]);
                }
            }
            else
            {
                if (reverseWinding)
                {
                    _allTris.Add(patchIndices[0]);
                    _allTris.Add(patchIndices[2]);
                    _allTris.Add(patchIndices[1]);
                    _allTris.Add(patchIndices[0]);
                    _allTris.Add(patchIndices[3]);
                    _allTris.Add(patchIndices[2]);
                }
                else
                {
                    _allTris.Add(patchIndices[0]);
                    _allTris.Add(patchIndices[1]);
                    _allTris.Add(patchIndices[2]);
                    _allTris.Add(patchIndices[0]);
                    _allTris.Add(patchIndices[2]);
                    _allTris.Add(patchIndices[3]);
                }
            }

            if (rebuildMesh)
                RebuildMesh();
        }

        /// <summary>
        /// 更新切面顶点位置（跟随物理变形）
        /// </summary>
        public void UpdatePositions(Core.TetMeshData data)
        {
            if (_allVerts.Count == 0) return;

            bool changed = false;
            int embedCursor = 0;
            int count = Mathf.Min(_allVerts.Count, _particleIndices.Count);
            for (int i = 0; i < count; i++)
            {
                int particleIndex = _particleIndices[i];
                if (particleIndex >= 0)
                {
                    if (particleIndex >= data.NumParticles) continue;

                    _allVerts[i] = data.Positions[particleIndex];
                    changed = true;
                    continue;
                }

                if (embedCursor >= _embedInfos.Count) continue;
                var embed = _embedInfos[embedCursor++];
                if (embed.tetIdx < 0 || embed.tetIdx >= data.NumTets) continue;
                if (!data.TetActive[embed.tetIdx]) continue;

                int tetBase = embed.tetIdx * 4;
                Vector3 p0 = data.Positions[data.TetIds[tetBase]];
                Vector3 p1 = data.Positions[data.TetIds[tetBase + 1]];
                Vector3 p2 = data.Positions[data.TetIds[tetBase + 2]];
                Vector3 p3 = data.Positions[data.TetIds[tetBase + 3]];

                Vector3 newPos = p0 * embed.weights.x + p1 * embed.weights.y +
                                 p2 * embed.weights.z + p3 * embed.weights.w;
                _allVerts[i] = newPos;
                changed = true;
            }

            if (changed && _cutMesh != null)
            {
                ApplyMeshData();
            }
        }

        public void SetParticleTriangles(
            Core.TetMeshData data,
            IList<int> particleTris,
            float maxEdgeLength = 0f)
        {
            Clear();
            if (data == null || particleTris == null || particleTris.Count < 3) return;

            float maxEdgeSq = maxEdgeLength > 0f ? maxEdgeLength * maxEdgeLength : 0f;
            var emittedGeometry = new HashSet<GeometryFaceKey>();
            int[] tri = new int[3];

            for (int i = 0; i + 2 < particleTris.Count; i += 3)
            {
                int a = particleTris[i];
                int b = particleTris[i + 1];
                int c = particleTris[i + 2];
                if (a < 0 || b < 0 || c < 0) continue;
                if (a >= data.NumParticles || b >= data.NumParticles || c >= data.NumParticles) continue;
                if (a == b || a == c || b == c) continue;

                Vector3 pa = data.Positions[a];
                Vector3 pb = data.Positions[b];
                Vector3 pc = data.Positions[c];

                if (maxEdgeSq > 0f)
                {
                    float ab = (pa - pb).sqrMagnitude;
                    float ac = (pa - pc).sqrMagnitude;
                    float bc = (pb - pc).sqrMagnitude;
                    if (ab > maxEdgeSq || ac > maxEdgeSq || bc > maxEdgeSq)
                        continue;
                }

                Vector3 normal = Vector3.Cross(pb - pa, pc - pa);
                if (normal.sqrMagnitude < 1e-12f) continue;
                if (!emittedGeometry.Add(MakeGeometryFaceKey(pa, pb, pc)))
                    continue;
                normal.Normalize();

                tri[0] = a;
                tri[1] = b;
                tri[2] = c;
                AddParticlePatch(data, tri, normal, reverseWinding: false, rebuildMesh: false);
            }

            RebuildMesh();
        }

        /// <summary>
        /// 清除所有切面数据
        /// </summary>
        public void Clear()
        {
            _allVerts.Clear();
            _allTris.Clear();
            _allNormals.Clear();
            _particleIndices.Clear();
            _particleToVerts.Clear();
            _embedInfos.Clear();
            _visibleTris.Clear();
            _runtimeNormals.Clear();

            if (_cutMesh != null)
            {
                _cutMesh.Clear();
            }
        }

        void RebuildMesh()
        {
            _cutMesh.Clear();

            if (_allVerts.Count == 0) return;
            ApplyMeshData();
        }

        void ApplyMeshData()
        {
            if (_cutMesh == null) return;

            _cutMesh.SetVertices(_allVerts);
            BuildVisibleTriangles();
            _cutMesh.SetTriangles(_visibleTris, 0);

            RebuildRuntimeNormals();
            _cutMesh.SetNormals(_runtimeNormals);

            _cutMesh.RecalculateBounds();
        }

        void BuildVisibleTriangles()
        {
            _visibleTris.Clear();
            float maxEdgeSq = maxRuntimeEdgeLength > 0f
                ? maxRuntimeEdgeLength * maxRuntimeEdgeLength
                : 0f;
            var emittedGeometry = new HashSet<GeometryFaceKey>();

            for (int i = 0; i + 2 < _allTris.Count; i += 3)
            {
                int a = _allTris[i];
                int b = _allTris[i + 1];
                int c = _allTris[i + 2];
                if (a < 0 || b < 0 || c < 0) continue;
                if (a >= _allVerts.Count || b >= _allVerts.Count || c >= _allVerts.Count) continue;

                if (maxEdgeSq > 0f)
                {
                    Vector3 pa = _allVerts[a];
                    Vector3 pb = _allVerts[b];
                    Vector3 pc = _allVerts[c];
                    if ((pa - pb).sqrMagnitude > maxEdgeSq ||
                        (pa - pc).sqrMagnitude > maxEdgeSq ||
                        (pb - pc).sqrMagnitude > maxEdgeSq)
                    {
                        continue;
                    }
                }

                if (!emittedGeometry.Add(MakeGeometryFaceKey(_allVerts[a], _allVerts[b], _allVerts[c])))
                    continue;

                _visibleTris.Add(a);
                _visibleTris.Add(b);
                _visibleTris.Add(c);
            }
        }

        void RebuildRuntimeNormals()
        {
            _runtimeNormals.Clear();
            for (int i = 0; i < _allVerts.Count; i++)
            {
                Vector3 n = i < _allNormals.Count && _allNormals[i].sqrMagnitude > 1e-10f
                    ? _allNormals[i].normalized
                    : Vector3.up;
                _runtimeNormals.Add(n);
            }
        }

        public void RebuildNow()
        {
            RebuildMesh();
        }

        int FindReusableParticleVertex(int particleId, Vector3 patchNormal)
        {
            if (!_particleToVerts.TryGetValue(particleId, out var group)) return -1;

            Vector3 n = patchNormal.sqrMagnitude > 1e-10f
                ? patchNormal.normalized
                : Vector3.up;
            float minDot = Mathf.Cos(Mathf.Max(0f, normalReuseAngleDeg) * Mathf.Deg2Rad);

            for (int i = 0; i < group.Count; i++)
            {
                int candidate = group[i];
                if (candidate < 0 || candidate >= _allNormals.Count) continue;

                Vector3 existing = _allNormals[candidate];
                if (existing.sqrMagnitude < 1e-10f) continue;
                existing.Normalize();
                if (Vector3.Dot(existing, n) >= minDot)
                    return candidate;
            }

            return -1;
        }

        /// <summary>
        /// 切面顶点数
        /// </summary>
        public int VertexCount => _allVerts.Count;

        /// <summary>
        /// 切面三角形数
        /// </summary>
        public int TriangleCount => _allTris.Count / 3;

        static GeometryFaceKey MakeGeometryFaceKey(Vector3 a, Vector3 b, Vector3 c)
        {
            return new GeometryFaceKey(
                QuantizedPoint(a),
                QuantizedPoint(b),
                QuantizedPoint(c));
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
    }
}
