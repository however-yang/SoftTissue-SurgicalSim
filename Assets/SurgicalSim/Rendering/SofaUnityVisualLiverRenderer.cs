using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim.Rendering
{
    /// <summary>
    /// SofaUnity-style liver renderer: a separate UV-authored visual mesh follows the tet body
    /// through barycentric mapping, matching Sofa's OglModel + BarycentricMapping pattern.
    /// </summary>
    public sealed class SofaUnityVisualLiverRenderer : MonoBehaviour
    {
        const string DefaultObjResourcePath = "SurgicalSim/SofaUnity/liver-smooth-obj";

        struct Binding
        {
            public int i0;
            public int i1;
            public int i2;
            public int i3;
            public Vector4 weights;
        }

        struct FaceKey : IEquatable<FaceKey>
        {
            public int a;
            public int b;
            public int c;

            public FaceKey(int i0, int i1, int i2)
            {
                if (i0 > i1) Swap(ref i0, ref i1);
                if (i1 > i2) Swap(ref i1, ref i2);
                if (i0 > i1) Swap(ref i0, ref i1);
                a = i0;
                b = i1;
                c = i2;
            }

            public bool Equals(FaceKey other)
            {
                return a == other.a && b == other.b && c == other.c;
            }

            public override bool Equals(object obj)
            {
                return obj is FaceKey other && Equals(other);
            }

            public override int GetHashCode()
            {
                unchecked
                {
                    int hash = 17;
                    hash = hash * 31 + a;
                    hash = hash * 31 + b;
                    hash = hash * 31 + c;
                    return hash;
                }
            }
        }

        struct FaceInfo
        {
            public int oppositeVertex;
            public int count;
        }

        MeshFilter _meshFilter;
        MeshRenderer _meshRenderer;
        Mesh _mesh;
        Material _material;

        Vector3[] _sourceVertices;
        Vector3[] _restVertices;
        Vector3[] _deformedVertices;
        Vector2[] _uvs;
        Vector3[] _normals;
        int[] _triangles;
        Binding[] _bindings;

        TetMeshData _data;
        bool _usesTetSurfaceGeometry;
        int _boundParticleCount = -1;
        int _boundTetCount = -1;
        int _boundSurfaceTriLength = -1;
        int[] _boundSurfaceTriRef;
        float _fitPadding = 0.985f;
        Vector2 _surfaceUvTiling = Vector2.one;
        int _fallbackBindings;
        float _maxSurfaceSnapDistance;
        float _avgSurfaceSnapDistance;

        public bool IsInitialized => _mesh != null && (_usesTetSurfaceGeometry || _bindings != null);

        public bool Init(
            TetMeshData data,
            Texture2D albedo,
            Material materialOverride = null,
            TextAsset objAsset = null,
            string objResourcePath = DefaultObjResourcePath,
            float fitPadding = 0.985f,
            bool useTetSurfaceGeometry = true,
            Vector2 surfaceUvTiling = default)
        {
            if (data == null)
            {
                Debug.LogWarning("[SofaUnityVisualLiverRenderer] Missing tet data.");
                return false;
            }

            _data = data;
            _fitPadding = Mathf.Clamp(fitPadding, 0.75f, 1.05f);
            _surfaceUvTiling = surfaceUvTiling == default ? Vector2.one : surfaceUvTiling;
            _usesTetSurfaceGeometry = useTetSurfaceGeometry &&
                                      data.SurfaceTriIds != null &&
                                      data.SurfaceTriIds.Length >= 3;

            if (!_usesTetSurfaceGeometry && (_sourceVertices == null || _triangles == null))
            {
                TextAsset resolvedObj = objAsset;
                if (resolvedObj == null)
                    resolvedObj = Resources.Load<TextAsset>(string.IsNullOrEmpty(objResourcePath)
                        ? DefaultObjResourcePath
                        : objResourcePath);

                if (resolvedObj == null)
                {
                    Debug.LogWarning("[SofaUnityVisualLiverRenderer] SofaUnity liver OBJ resource not found: " +
                                     (string.IsNullOrEmpty(objResourcePath) ? DefaultObjResourcePath : objResourcePath));
                    return false;
                }

                if (!TryParseObj(resolvedObj.text, out _sourceVertices, out _uvs, out _normals, out _triangles))
                {
                    Debug.LogWarning("[SofaUnityVisualLiverRenderer] Failed to parse SofaUnity liver OBJ.");
                    return false;
                }

                int flipped = OrientTrianglesOutward(_sourceVertices, _triangles);
                Debug.Log("[SofaUnityVisualLiverRenderer] OBJ parsed | vertices=" + _sourceVertices.Length +
                          " | triangles=" + (_triangles.Length / 3) +
                          " | outwardFlipped=" + flipped);
            }

            EnsureObjects(_usesTetSurfaceGeometry ? data.NumParticles : _sourceVertices.Length);
            _material = materialOverride != null
                ? new Material(materialOverride) { name = materialOverride.name + "_RuntimeDoubleSided" }
                : CreateSofaUnityLiverMaterial(albedo);
            ConfigureDoubleSided(_material);
            if (_material != null)
                _meshRenderer.sharedMaterial = _material;

            if (_usesTetSurfaceGeometry)
            {
                BuildTetSurfaceMesh(data);
                Refresh();

                Debug.Log("[SofaUnityVisualLiverRenderer] SofaUnity tet-surface visual mesh active | vertices=" +
                          data.NumParticles + " | triangles=" + (data.SurfaceTriIds.Length / 3) +
                          " | UVTiling=" + _surfaceUvTiling);
                return true;
            }

            BuildRestVertices(data);
            BuildBindings(data);
            BuildMesh();
            Refresh();

            Debug.Log("[SofaUnityVisualLiverRenderer] SofaUnity visual mesh active | vertices=" +
                      _restVertices.Length + " | triangles=" + (_triangles.Length / 3) +
                      " | fallbackBindings=" + _fallbackBindings);
            return true;
        }

        public void Refresh()
        {
            if (_data == null || _mesh == null) return;

            if (_usesTetSurfaceGeometry)
            {
                RefreshTetSurfaceMesh();
                return;
            }

            if (_bindings == null) return;

            if (_data.NumParticles != _boundParticleCount || _data.NumTets != _boundTetCount)
            {
                BuildBindings(_data);
                _boundParticleCount = _data.NumParticles;
                _boundTetCount = _data.NumTets;
            }

            for (int i = 0; i < _bindings.Length; i++)
            {
                Binding b = _bindings[i];
                if (b.i0 < 0 || b.i0 >= _data.Positions.Length)
                {
                    _deformedVertices[i] = _restVertices[i];
                    continue;
                }

                Vector4 w = b.weights;
                _deformedVertices[i] =
                    _data.Positions[b.i0] * w.x +
                    (b.i1 >= 0 && b.i1 < _data.Positions.Length ? _data.Positions[b.i1] * w.y : Vector3.zero) +
                    (b.i2 >= 0 && b.i2 < _data.Positions.Length ? _data.Positions[b.i2] * w.z : Vector3.zero) +
                    (b.i3 >= 0 && b.i3 < _data.Positions.Length ? _data.Positions[b.i3] * w.w : Vector3.zero);
            }

            _mesh.SetVertices(_deformedVertices);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
        }

        public void SetMaterial(Material material)
        {
            if (material == null) return;
            _material = material;
            if (_meshRenderer != null)
                _meshRenderer.sharedMaterial = material;
        }

        public static Material CreateProject2Liver2Material(Texture2D albedo, Texture2D bumpMap, Texture2D heightMap)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Standard");
            if (shader == null) return null;

            var mat = new Material(shader) { name = "Runtime_Project2_Liver2" };

            if (albedo != null)
            {
                albedo.wrapMode = TextureWrapMode.Repeat;
                albedo.filterMode = FilterMode.Bilinear;
                SetTextureIf(mat, "_BaseMap", albedo);
                SetTextureIf(mat, "_MainTex", albedo);
            }

            if (bumpMap != null)
            {
                bumpMap.wrapMode = TextureWrapMode.Repeat;
                bumpMap.filterMode = FilterMode.Bilinear;
                SetTextureIf(mat, "_BumpMap", bumpMap);
                mat.EnableKeyword("_NORMALMAP");
            }

            if (heightMap != null)
            {
                heightMap.wrapMode = TextureWrapMode.Repeat;
                heightMap.filterMode = FilterMode.Bilinear;
                SetTextureIf(mat, "_ParallaxMap", heightMap);
                mat.EnableKeyword("_PARALLAXMAP");
            }

            Vector2 baseTiling = new Vector2(2f, 2f);
            Vector2 baseOffset = new Vector2(0.15f, 0f);
            SetTextureScaleIf(mat, "_BaseMap", baseTiling);
            SetTextureOffsetIf(mat, "_BaseMap", baseOffset);
            SetTextureScaleIf(mat, "_MainTex", baseTiling);
            SetTextureOffsetIf(mat, "_MainTex", baseOffset);
            SetTextureScaleIf(mat, "_EmissionMap", baseTiling);
            SetTextureOffsetIf(mat, "_EmissionMap", baseOffset);
            SetTextureScaleIf(mat, "_DetailAlbedoMap", new Vector2(0.5f, 0.5f));
            SetTextureOffsetIf(mat, "_DetailAlbedoMap", new Vector2(0.29500002f, 0f));

            SetColorIf(mat, "_BaseColor", new Color(0.7205882f, 0.7205882f, 0.7205882f, 1f));
            SetColorIf(mat, "_Color", new Color(0.7205882f, 0.7205882f, 0.7205882f, 1f));
            SetColorIf(mat, "_EmissionColor", Color.black);
            SetColorIf(mat, "_SpecColor", new Color(0.2f, 0.2f, 0.2f, 1f));

            SetFloatIf(mat, "_WorkflowMode", 1f);
            SetFloatIf(mat, "_Surface", 0f);
            SetFloatIf(mat, "_Blend", 0f);
            SetFloatIf(mat, "_AlphaClip", 0f);
            SetFloatIf(mat, "_SrcBlend", (float)UnityEngine.Rendering.BlendMode.One);
            SetFloatIf(mat, "_DstBlend", (float)UnityEngine.Rendering.BlendMode.Zero);
            SetFloatIf(mat, "_ZWrite", 1f);
            // Project2's asset uses Front-face rendering. The deformable runtime mesh needs two-sided
            // rendering so the back side does not disappear when viewed from the opposite side.
            SetFloatIf(mat, "_Cull", 0f);
            SetFloatIf(mat, "_ReceiveShadows", 1f);
            SetFloatIf(mat, "_Metallic", 0.184f);
            SetFloatIf(mat, "_Smoothness", 0.939f);
            SetFloatIf(mat, "_Glossiness", 0.939f);
            SetFloatIf(mat, "_BumpScale", 1.03f);
            SetFloatIf(mat, "_Parallax", 0.08f);
            SetFloatIf(mat, "_SmoothnessTextureChannel", 0f);
            SetFloatIf(mat, "_GlossMapScale", 1f);
            SetFloatIf(mat, "_OcclusionStrength", 1f);
            SetFloatIf(mat, "_SpecularHighlights", 1f);
            SetFloatIf(mat, "_EnvironmentReflections", 1f);
            SetFloatIf(mat, "_QueueOffset", 0f);

            mat.EnableKeyword("_EMISSION");
            mat.SetOverrideTag("RenderType", "Opaque");
            mat.renderQueue = (int)UnityEngine.Rendering.RenderQueue.Geometry;
            mat.doubleSidedGI = true;
            return mat;
        }

        public static Material CreateSofaUnityLiverMaterial(Texture2D albedo)
        {
            Shader shader = Shader.Find("Universal Render Pipeline/Lit");
            if (shader == null) shader = Shader.Find("Standard");
            if (shader == null) return null;

            var mat = new Material(shader) { name = "Runtime_SofaUnity_Liver2" };
            if (albedo != null)
            {
                albedo.wrapMode = TextureWrapMode.Repeat;
                albedo.filterMode = FilterMode.Bilinear;
                if (mat.HasProperty("_BaseMap")) mat.SetTexture("_BaseMap", albedo);
                if (mat.HasProperty("_MainTex")) mat.SetTexture("_MainTex", albedo);
            }

            SetColorIf(mat, "_BaseColor", Color.white);
            SetColorIf(mat, "_Color", Color.white);
            SetColorIf(mat, "_SpecColor", new Color(0.2f, 0.2f, 0.2f, 1f));
            SetFloatIf(mat, "_Smoothness", 0.10f);
            SetFloatIf(mat, "_Glossiness", 0.10f);
            SetFloatIf(mat, "_Metallic", 0f);
            SetFloatIf(mat, "_ReceiveShadows", 0f);
            ConfigureDoubleSided(mat);
            return mat;
        }

        static void ConfigureDoubleSided(Material mat)
        {
            if (mat == null) return;
            SetFloatIf(mat, "_Cull", 0f);
            mat.doubleSidedGI = true;
        }

        void EnsureObjects(int vertexCapacity)
        {
            if (_meshFilter == null || _meshRenderer == null)
            {
                Transform child = transform.Find("SofaUnityVisualLiver");
                GameObject go = child != null ? child.gameObject : new GameObject("SofaUnityVisualLiver");
                go.transform.SetParent(transform, false);
                _meshFilter = go.GetComponent<MeshFilter>();
                if (_meshFilter == null) _meshFilter = go.AddComponent<MeshFilter>();
                _meshRenderer = go.GetComponent<MeshRenderer>();
                if (_meshRenderer == null) _meshRenderer = go.AddComponent<MeshRenderer>();
            }

            if (_mesh == null)
            {
                _mesh = new Mesh
                {
                    name = "SofaUnity_LiverVisual",
                    indexFormat = vertexCapacity > 65535
                        ? UnityEngine.Rendering.IndexFormat.UInt32
                        : UnityEngine.Rendering.IndexFormat.UInt16
                };
                _mesh.MarkDynamic();
                _meshFilter.sharedMesh = _mesh;
            }
        }

        void BuildTetSurfaceMesh(TetMeshData data)
        {
            int count = data.NumParticles;
            _bindings = null;
            _sourceVertices = null;
            _normals = null;

            _restVertices = new Vector3[count];
            _deformedVertices = new Vector3[count];
            Array.Copy(data.RestPositions, _restVertices, count);
            Array.Copy(data.Positions, _deformedVertices, count);

            _triangles = (int[])data.SurfaceTriIds.Clone();
            int fallbackFaces;
            int flipped = OrientSurfaceTrianglesFromTetTopology(data, _restVertices, _triangles, out fallbackFaces);
            _uvs = BuildSofaProjectionUVs(_restVertices, count, _surfaceUvTiling);

            _mesh.Clear();
            _mesh.indexFormat = count > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;
            _mesh.SetVertices(_deformedVertices, 0, count);
            _mesh.SetUVs(0, new List<Vector2>(_uvs));
            _mesh.SetUVs(1, new List<Vector3>(_restVertices));
            _mesh.SetTriangles(_triangles, 0);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();

            _boundParticleCount = data.NumParticles;
            _boundTetCount = data.NumTets;
            _boundSurfaceTriLength = data.SurfaceTriIds.Length;
            _boundSurfaceTriRef = data.SurfaceTriIds;

            Debug.Log("[SofaUnityVisualLiverRenderer] Tet surface renderer built from current liver3-HD mesh | vertices=" +
                      count + " | triangles=" + (_triangles.Length / 3) +
                      " | topologyOutwardFlipped=" + flipped +
                      " | fallbackFaces=" + fallbackFaces);
        }

        void RefreshTetSurfaceMesh()
        {
            if (_data == null || _mesh == null) return;

            bool topologyChanged =
                _data.NumParticles != _boundParticleCount ||
                _data.NumTets != _boundTetCount ||
                _data.SurfaceTriIds == null ||
                _data.SurfaceTriIds.Length != _boundSurfaceTriLength ||
                !ReferenceEquals(_data.SurfaceTriIds, _boundSurfaceTriRef) ||
                _deformedVertices == null ||
                _deformedVertices.Length != _data.NumParticles;

            if (topologyChanged)
            {
                if (_data.SurfaceTriIds != null && _data.SurfaceTriIds.Length >= 3)
                    BuildTetSurfaceMesh(_data);
                return;
            }

            Array.Copy(_data.Positions, _deformedVertices, _data.NumParticles);
            _mesh.SetVertices(_deformedVertices, 0, _data.NumParticles);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
        }

        void BuildRestVertices(TetMeshData data)
        {
            _restVertices = new Vector3[_sourceVertices.Length];
            _deformedVertices = new Vector3[_sourceVertices.Length];

            Bounds sourceBounds = ComputeBounds(_sourceVertices, _sourceVertices.Length);
            Bounds targetBounds = ComputeBounds(data.RestPositions, data.NumParticles);

            Vector3 sourceSize = sourceBounds.size;
            Vector3 targetSize = targetBounds.size * _fitPadding;
            Vector3 scale = new Vector3(
                SafeDiv(targetSize.x, sourceSize.x),
                SafeDiv(targetSize.y, sourceSize.y),
                SafeDiv(targetSize.z, sourceSize.z));

            for (int i = 0; i < _sourceVertices.Length; i++)
            {
                Vector3 centered = _sourceVertices[i] - sourceBounds.center;
                _restVertices[i] = targetBounds.center + Vector3.Scale(centered, scale);
            }
        }

        void BuildMesh()
        {
            _mesh.Clear();
            _mesh.indexFormat = _restVertices.Length > 65535
                ? UnityEngine.Rendering.IndexFormat.UInt32
                : UnityEngine.Rendering.IndexFormat.UInt16;
            _mesh.SetVertices(_restVertices);
            if (_uvs != null && _uvs.Length == _restVertices.Length)
                _mesh.SetUVs(0, new List<Vector2>(_uvs));
            if (_normals != null && _normals.Length == _restVertices.Length)
                _mesh.SetNormals(new List<Vector3>(_normals));
            _mesh.SetTriangles(_triangles, 0);
            _mesh.RecalculateBounds();
            if (_normals == null || _normals.Length != _restVertices.Length)
                _mesh.RecalculateNormals();
        }

        void BuildBindings(TetMeshData data)
        {
            _bindings = new Binding[_restVertices.Length];
            _fallbackBindings = 0;
            _maxSurfaceSnapDistance = 0f;
            _avgSurfaceSnapDistance = 0f;

            if (data.SurfaceTriIds != null && data.SurfaceTriIds.Length >= 3)
            {
                BuildSurfaceBindings(data);
                Debug.Log("[SofaUnityVisualLiverRenderer] Surface visual binding | vertices=" +
                          _restVertices.Length + " | avgSnap=" + _avgSurfaceSnapDistance.ToString("F5") +
                          " | maxSnap=" + _maxSurfaceSnapDistance.ToString("F5"));
                _boundParticleCount = data.NumParticles;
                _boundTetCount = data.NumTets;
                return;
            }

            for (int i = 0; i < _restVertices.Length; i++)
            {
                if (!FindContainingTet(data, _restVertices[i], out Binding binding))
                {
                    binding = FindNearestTet(data, _restVertices[i]);
                    binding.weights = ClampBarycentric(binding.weights);
                    _fallbackBindings++;
                }

                _bindings[i] = binding;
            }

            _boundParticleCount = data.NumParticles;
            _boundTetCount = data.NumTets;
        }

        void BuildSurfaceBindings(TetMeshData data)
        {
            float totalDistance = 0f;

            for (int i = 0; i < _restVertices.Length; i++)
            {
                Binding binding = FindNearestSurfaceTriangle(data, _restVertices[i], out float distance);
                _bindings[i] = binding;
                totalDistance += distance;
                _maxSurfaceSnapDistance = Mathf.Max(_maxSurfaceSnapDistance, distance);
            }

            _avgSurfaceSnapDistance = _restVertices.Length > 0 ? totalDistance / _restVertices.Length : 0f;
        }

        static Binding FindNearestSurfaceTriangle(TetMeshData data, Vector3 p, out float distance)
        {
            int bestA = -1;
            int bestB = -1;
            int bestC = -1;
            Vector3 bestBary = new Vector3(1f, 0f, 0f);
            float bestDistSq = float.MaxValue;

            int[] tris = data.SurfaceTriIds;
            for (int i = 0; i + 2 < tris.Length; i += 3)
            {
                int a = tris[i + 0];
                int b = tris[i + 1];
                int c = tris[i + 2];
                if (a < 0 || b < 0 || c < 0 ||
                    a >= data.RestPositions.Length ||
                    b >= data.RestPositions.Length ||
                    c >= data.RestPositions.Length)
                {
                    continue;
                }

                Vector3 bary;
                Vector3 closest = ClosestPointOnTriangle(
                    p,
                    data.RestPositions[a],
                    data.RestPositions[b],
                    data.RestPositions[c],
                    out bary);
                float distSq = (closest - p).sqrMagnitude;
                if (distSq < bestDistSq)
                {
                    bestDistSq = distSq;
                    bestA = a;
                    bestB = b;
                    bestC = c;
                    bestBary = bary;
                }
            }

            distance = Mathf.Sqrt(Mathf.Max(0f, bestDistSq));
            if (bestA < 0)
            {
                return new Binding
                {
                    i0 = 0,
                    i1 = -1,
                    i2 = -1,
                    i3 = -1,
                    weights = new Vector4(1f, 0f, 0f, 0f)
                };
            }

            return new Binding
            {
                i0 = bestA,
                i1 = bestB,
                i2 = bestC,
                i3 = -1,
                weights = new Vector4(bestBary.x, bestBary.y, bestBary.z, 0f)
            };
        }

        static bool FindContainingTet(TetMeshData data, Vector3 p, out Binding binding)
        {
            const float tolerance = -0.08f;
            binding = new Binding
            {
                i0 = -1,
                i1 = -1,
                i2 = -1,
                i3 = -1,
                weights = new Vector4(1f, 0f, 0f, 0f)
            };

            for (int t = 0; t < data.NumTets; t++)
            {
                if (data.TetActive != null && t < data.TetActive.Length && !data.TetActive[t])
                    continue;

                int b = t * 4;
                if (b + 3 >= data.TetIds.Length) break;

                Vector3 w;
                if (!TryBarycentric(
                        p,
                        data.RestPositions[data.TetIds[b + 0]],
                        data.RestPositions[data.TetIds[b + 1]],
                        data.RestPositions[data.TetIds[b + 2]],
                        data.RestPositions[data.TetIds[b + 3]],
                        out w))
                {
                    continue;
                }

                float w0 = 1f - w.x - w.y - w.z;
                if (w0 >= tolerance && w.x >= tolerance && w.y >= tolerance && w.z >= tolerance)
                {
                    binding = new Binding
                    {
                        i0 = data.TetIds[b + 0],
                        i1 = data.TetIds[b + 1],
                        i2 = data.TetIds[b + 2],
                        i3 = data.TetIds[b + 3],
                        weights = new Vector4(w0, w.x, w.y, w.z)
                    };
                    return true;
                }
            }

            return false;
        }

        static Binding FindNearestTet(TetMeshData data, Vector3 p)
        {
            int bestTet = -1;
            float bestDist = float.MaxValue;

            for (int t = 0; t < data.NumTets; t++)
            {
                if (data.TetActive != null && t < data.TetActive.Length && !data.TetActive[t])
                    continue;

                int b = t * 4;
                if (b + 3 >= data.TetIds.Length) break;

                Vector3 center =
                    (data.RestPositions[data.TetIds[b + 0]] +
                     data.RestPositions[data.TetIds[b + 1]] +
                     data.RestPositions[data.TetIds[b + 2]] +
                     data.RestPositions[data.TetIds[b + 3]]) * 0.25f;

                float dist = (center - p).sqrMagnitude;
                if (dist < bestDist)
                {
                    bestDist = dist;
                    bestTet = t;
                }
            }

            if (bestTet < 0)
            {
                return new Binding
                {
                    i0 = 0,
                    i1 = -1,
                    i2 = -1,
                    i3 = -1,
                    weights = new Vector4(1f, 0f, 0f, 0f)
                };
            }

            int bb = bestTet * 4;
            Vector4 weights;
            if (TryBarycentric(
                    p,
                    data.RestPositions[data.TetIds[bb + 0]],
                    data.RestPositions[data.TetIds[bb + 1]],
                    data.RestPositions[data.TetIds[bb + 2]],
                    data.RestPositions[data.TetIds[bb + 3]],
                    out Vector3 bary))
            {
                weights = new Vector4(1f - bary.x - bary.y - bary.z, bary.x, bary.y, bary.z);
            }
            else
            {
                weights = new Vector4(1f, 0f, 0f, 0f);
            }

            return new Binding
            {
                i0 = data.TetIds[bb + 0],
                i1 = data.TetIds[bb + 1],
                i2 = data.TetIds[bb + 2],
                i3 = data.TetIds[bb + 3],
                weights = weights
            };
        }

        static Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, out Vector3 bary)
        {
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;
            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f)
            {
                bary = new Vector3(1f, 0f, 0f);
                return a;
            }

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3)
            {
                bary = new Vector3(0f, 1f, 0f);
                return b;
            }

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                bary = new Vector3(1f - v, v, 0f);
                return a + ab * v;
            }

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6)
            {
                bary = new Vector3(0f, 0f, 1f);
                return c;
            }

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                bary = new Vector3(1f - w, 0f, w);
                return a + ac * w;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                bary = new Vector3(0f, 1f - w, w);
                return b + (c - b) * w;
            }

            float denom = 1f / (va + vb + vc);
            float vFace = vb * denom;
            float wFace = vc * denom;
            bary = new Vector3(1f - vFace - wFace, vFace, wFace);
            return a + ab * vFace + ac * wFace;
        }

        static bool TryBarycentric(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d, out Vector3 bary)
        {
            Vector3 v0 = b - a;
            Vector3 v1 = c - a;
            Vector3 v2 = d - a;
            Vector3 vp = p - a;
            float det = Vector3.Dot(v0, Vector3.Cross(v1, v2));

            if (Mathf.Abs(det) < 1e-12f)
            {
                bary = Vector3.zero;
                return false;
            }

            float invDet = 1f / det;
            float u = Vector3.Dot(vp, Vector3.Cross(v1, v2)) * invDet;
            float v = Vector3.Dot(v0, Vector3.Cross(vp, v2)) * invDet;
            float w = Vector3.Dot(v0, Vector3.Cross(v1, vp)) * invDet;
            bary = new Vector3(u, v, w);
            return true;
        }

        static Vector4 ClampBarycentric(Vector4 weights)
        {
            weights.x = Mathf.Max(0f, weights.x);
            weights.y = Mathf.Max(0f, weights.y);
            weights.z = Mathf.Max(0f, weights.z);
            weights.w = Mathf.Max(0f, weights.w);
            float sum = weights.x + weights.y + weights.z + weights.w;
            return sum > 1e-8f ? weights / sum : new Vector4(1f, 0f, 0f, 0f);
        }

        static bool TryParseObj(string text, out Vector3[] vertices, out Vector2[] uvs, out Vector3[] normals, out int[] triangles)
        {
            var positions = new List<Vector3>(4096);
            var texcoords = new List<Vector2>(4096);
            var normalSrc = new List<Vector3>(4096);
            var outVertices = new List<Vector3>(4096);
            var outUvs = new List<Vector2>(4096);
            var outNormals = new List<Vector3>(4096);
            var outTriangles = new List<int>(8192);
            var lookup = new Dictionary<string, int>(4096);
            bool hasAnyNormal = false;

            string[] lines = text.Split(new[] { '\n', '\r' }, StringSplitOptions.RemoveEmptyEntries);
            for (int i = 0; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (line.Length == 0 || line[0] == '#') continue;

                string[] parts = line.Split((char[])null, StringSplitOptions.RemoveEmptyEntries);
                if (parts.Length == 0) continue;

                if (parts[0] == "v" && parts.Length >= 4)
                {
                    positions.Add(new Vector3(ParseFloat(parts[1]), ParseFloat(parts[2]), ParseFloat(parts[3])));
                }
                else if (parts[0] == "vt" && parts.Length >= 3)
                {
                    texcoords.Add(new Vector2(ParseFloat(parts[1]), ParseFloat(parts[2])));
                }
                else if (parts[0] == "vn" && parts.Length >= 4)
                {
                    normalSrc.Add(new Vector3(ParseFloat(parts[1]), ParseFloat(parts[2]), ParseFloat(parts[3])).normalized);
                }
                else if (parts[0] == "f" && parts.Length >= 4)
                {
                    int first = ResolveObjVertex(parts[1], positions, texcoords, normalSrc, outVertices, outUvs, outNormals, lookup, ref hasAnyNormal);
                    int prev = ResolveObjVertex(parts[2], positions, texcoords, normalSrc, outVertices, outUvs, outNormals, lookup, ref hasAnyNormal);
                    for (int k = 3; k < parts.Length; k++)
                    {
                        int next = ResolveObjVertex(parts[k], positions, texcoords, normalSrc, outVertices, outUvs, outNormals, lookup, ref hasAnyNormal);
                        outTriangles.Add(first);
                        outTriangles.Add(prev);
                        outTriangles.Add(next);
                        prev = next;
                    }
                }
            }

            vertices = outVertices.ToArray();
            uvs = outUvs.Count == outVertices.Count ? outUvs.ToArray() : null;
            normals = hasAnyNormal && outNormals.Count == outVertices.Count ? outNormals.ToArray() : null;
            triangles = outTriangles.ToArray();
            return vertices.Length > 0 && triangles.Length >= 3;
        }

        static int OrientTrianglesOutward(Vector3[] vertices, int[] triangles)
        {
            if (vertices == null || triangles == null || vertices.Length == 0)
                return 0;

            Vector3 center = ComputeBounds(vertices, vertices.Length).center;
            int flipped = 0;

            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                int ia = triangles[i + 0];
                int ib = triangles[i + 1];
                int ic = triangles[i + 2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    ia >= vertices.Length || ib >= vertices.Length || ic >= vertices.Length)
                {
                    continue;
                }

                Vector3 a = vertices[ia];
                Vector3 b = vertices[ib];
                Vector3 c = vertices[ic];
                Vector3 normal = Vector3.Cross(b - a, c - a);
                Vector3 centroid = (a + b + c) / 3f;

                if (Vector3.Dot(normal, centroid - center) < 0f)
                {
                    triangles[i + 1] = ic;
                    triangles[i + 2] = ib;
                    flipped++;
                }
            }

            return flipped;
        }

        static int OrientSurfaceTrianglesFromTetTopology(
            TetMeshData data,
            Vector3[] vertices,
            int[] triangles,
            out int fallbackFaces)
        {
            fallbackFaces = 0;
            if (data == null || data.TetIds == null || vertices == null || triangles == null)
                return OrientTrianglesOutward(vertices, triangles);

            var faceInfo = new Dictionary<FaceKey, FaceInfo>(data.NumTets * 4);
            for (int t = 0; t < data.NumTets; t++)
            {
                if (data.TetActive != null && t < data.TetActive.Length && !data.TetActive[t])
                    continue;

                int baseIndex = t * 4;
                if (baseIndex + 3 >= data.TetIds.Length)
                    break;

                int v0 = data.TetIds[baseIndex + 0];
                int v1 = data.TetIds[baseIndex + 1];
                int v2 = data.TetIds[baseIndex + 2];
                int v3 = data.TetIds[baseIndex + 3];

                AddTetFace(faceInfo, v1, v2, v3, v0);
                AddTetFace(faceInfo, v0, v3, v2, v1);
                AddTetFace(faceInfo, v0, v1, v3, v2);
                AddTetFace(faceInfo, v0, v2, v1, v3);
            }

            int flipped = 0;
            Vector3 center = ComputeBounds(vertices, vertices.Length).center;

            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                int ia = triangles[i + 0];
                int ib = triangles[i + 1];
                int ic = triangles[i + 2];
                if (!ValidVertex(vertices, ia) || !ValidVertex(vertices, ib) || !ValidVertex(vertices, ic))
                {
                    fallbackFaces++;
                    continue;
                }

                Vector3 a = vertices[ia];
                Vector3 b = vertices[ib];
                Vector3 c = vertices[ic];
                Vector3 normal = Vector3.Cross(b - a, c - a);
                Vector3 centroid = (a + b + c) / 3f;

                FaceInfo info;
                bool shouldFlip;
                if (faceInfo.TryGetValue(new FaceKey(ia, ib, ic), out info) &&
                    ValidVertex(vertices, info.oppositeVertex))
                {
                    Vector3 inward = vertices[info.oppositeVertex] - centroid;
                    shouldFlip = Vector3.Dot(normal, inward) > 0f;
                }
                else
                {
                    fallbackFaces++;
                    shouldFlip = Vector3.Dot(normal, centroid - center) < 0f;
                }

                if (shouldFlip)
                {
                    triangles[i + 1] = ic;
                    triangles[i + 2] = ib;
                    flipped++;
                }
            }

            return flipped;
        }

        static void AddTetFace(Dictionary<FaceKey, FaceInfo> faceInfo, int a, int b, int c, int opposite)
        {
            var key = new FaceKey(a, b, c);
            FaceInfo info;
            if (faceInfo.TryGetValue(key, out info))
            {
                info.count++;
                faceInfo[key] = info;
            }
            else
            {
                faceInfo[key] = new FaceInfo
                {
                    oppositeVertex = opposite,
                    count = 1
                };
            }
        }

        static bool ValidVertex(Vector3[] vertices, int index)
        {
            return vertices != null && index >= 0 && index < vertices.Length;
        }

        static Vector2[] BuildSofaProjectionUVs(Vector3[] positions, int count, Vector2 tiling)
        {
            var uvs = new Vector2[count];
            if (positions == null || count <= 0)
                return uvs;

            Bounds bounds = ComputeBounds(positions, count);
            Vector3 min = bounds.min;
            Vector3 size = bounds.size;

            int uAxis;
            int vAxis;
            if (size.x <= size.y && size.x <= size.z)
            {
                uAxis = 1;
                vAxis = 2;
            }
            else if (size.y <= size.x && size.y <= size.z)
            {
                uAxis = 0;
                vAxis = 2;
            }
            else
            {
                uAxis = 0;
                vAxis = 1;
            }

            float minU = Axis(min, uAxis);
            float minV = Axis(min, vAxis);
            float invU = 1f / Mathf.Max(Axis(size, uAxis), 1e-6f);
            float invV = 1f / Mathf.Max(Axis(size, vAxis), 1e-6f);
            Vector2 safeTiling = new Vector2(
                Mathf.Abs(tiling.x) > 1e-5f ? tiling.x : 1f,
                Mathf.Abs(tiling.y) > 1e-5f ? tiling.y : 1f);

            for (int i = 0; i < count; i++)
            {
                float u = (Axis(positions[i], uAxis) - minU) * invU;
                float v = (Axis(positions[i], vAxis) - minV) * invV;
                uvs[i] = new Vector2(u * safeTiling.x, v * safeTiling.y);
            }

            return uvs;
        }

        static int ResolveObjVertex(
            string token,
            List<Vector3> positions,
            List<Vector2> texcoords,
            List<Vector3> normalSrc,
            List<Vector3> outVertices,
            List<Vector2> outUvs,
            List<Vector3> outNormals,
            Dictionary<string, int> lookup,
            ref bool hasAnyNormal)
        {
            if (lookup.TryGetValue(token, out int index))
                return index;

            string[] fields = token.Split('/');
            int vi = ObjIndex(fields, 0, positions.Count);
            int ti = ObjIndex(fields, 1, texcoords.Count);
            int ni = ObjIndex(fields, 2, normalSrc.Count);

            outVertices.Add(vi >= 0 ? positions[vi] : Vector3.zero);
            outUvs.Add(ti >= 0 ? texcoords[ti] : Vector2.zero);
            if (ni >= 0)
            {
                outNormals.Add(normalSrc[ni]);
                hasAnyNormal = true;
            }
            else
            {
                outNormals.Add(Vector3.up);
            }

            index = outVertices.Count - 1;
            lookup[token] = index;
            return index;
        }

        static int ObjIndex(string[] fields, int slot, int count)
        {
            if (slot >= fields.Length || string.IsNullOrEmpty(fields[slot]))
                return -1;

            int parsed = int.Parse(fields[slot], CultureInfo.InvariantCulture);
            if (parsed > 0) return parsed - 1;
            if (parsed < 0) return count + parsed;
            return -1;
        }

        static float ParseFloat(string value)
        {
            return float.Parse(value, NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        static Bounds ComputeBounds(Vector3[] points, int count)
        {
            if (points == null || count <= 0)
                return new Bounds(Vector3.zero, Vector3.one);

            Vector3 min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector3 max = new Vector3(float.MinValue, float.MinValue, float.MinValue);
            int n = Mathf.Min(count, points.Length);
            for (int i = 0; i < n; i++)
            {
                min = Vector3.Min(min, points[i]);
                max = Vector3.Max(max, points[i]);
            }

            var bounds = new Bounds();
            bounds.SetMinMax(min, max);
            return bounds;
        }

        static float SafeDiv(float value, float divisor)
        {
            return Mathf.Abs(divisor) > 1e-8f ? value / divisor : 1f;
        }

        static float Axis(Vector3 value, int axis)
        {
            return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
        }

        static void Swap(ref int a, ref int b)
        {
            int temp = a;
            a = b;
            b = temp;
        }

        static void SetColorIf(Material mat, string property, Color value)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetColor(property, value);
        }

        static void SetFloatIf(Material mat, string property, float value)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetFloat(property, value);
        }

        static void SetTextureIf(Material mat, string property, Texture texture)
        {
            if (mat != null && texture != null && mat.HasProperty(property))
                mat.SetTexture(property, texture);
        }

        static void SetTextureScaleIf(Material mat, string property, Vector2 scale)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetTextureScale(property, scale);
        }

        static void SetTextureOffsetIf(Material mat, string property, Vector2 offset)
        {
            if (mat != null && mat.HasProperty(property))
                mat.SetTextureOffset(property, offset);
        }
    }
}
