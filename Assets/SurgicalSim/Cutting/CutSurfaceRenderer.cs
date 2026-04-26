// CutSurfaceRenderer.cs
// 切面内部组织渲染器
// 在切割处生成一个独立的 Mesh 来渲染内部组织表面
// 支持：累积多次切割、跟随物理变形

using System.Collections.Generic;
using UnityEngine;

namespace SurgicalSim.Cutting
{
    [RequireComponent(typeof(MeshFilter))]
    [RequireComponent(typeof(MeshRenderer))]
    public class CutSurfaceRenderer : MonoBehaviour
    {
        [Header("切面材质")]
        public Color cutSurfaceColor = new Color(0.85f, 0.45f, 0.45f, 1f);
        public float smoothness = 0.3f;
        public float metallic = 0.0f;

        MeshFilter _meshFilter;
        MeshRenderer _meshRenderer;
        Mesh _cutMesh;

        // 累积的切面几何数据
        List<Vector3> _allVerts = new List<Vector3>();
        List<int>     _allTris  = new List<int>();
        List<Vector3> _allNormals = new List<Vector3>();
        List<int>     _particleIndices = new List<int>();
        Dictionary<int, int> _particleToVert = new Dictionary<int, int>();

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
            var shader = Shader.Find("SurgicalSim/TwoSidedLiver");
            if (shader == null)
                shader = Shader.Find("Universal Render Pipeline/Lit");

            if (shader != null)
            {
                var mat = new Material(shader);
                mat.SetColor("_Color", cutSurfaceColor);
                mat.SetColor("_InteriorColor", cutSurfaceColor);
                if (mat.HasProperty("_Smoothness"))
                    mat.SetFloat("_Smoothness", smoothness);
                if (mat.HasProperty("_Metallic"))
                    mat.SetFloat("_Metallic", metallic);

                // 关闭背面剔除
                mat.SetFloat("_Cull", 0); // Off
                _meshRenderer.material = mat;
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

                if (_particleToVert.TryGetValue(pid, out int existingIndex))
                {
                    patchIndices[i] = existingIndex;
                    _allNormals[existingIndex] += patchNormal;
                    continue;
                }

                int newIndex = _allVerts.Count;
                _particleToVert[pid] = newIndex;
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
                _cutMesh.SetVertices(_allVerts);
                _cutMesh.RecalculateNormals();
                _cutMesh.RecalculateBounds();
            }
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
            _particleToVert.Clear();
            _embedInfos.Clear();

            if (_cutMesh != null)
            {
                _cutMesh.Clear();
            }
        }

        void RebuildMesh()
        {
            _cutMesh.Clear();

            if (_allVerts.Count == 0) return;

            _cutMesh.SetVertices(_allVerts);

            if (_allNormals.Count == _allVerts.Count)
            {
                for (int i = 0; i < _allNormals.Count; i++)
                {
                    if (_allNormals[i].sqrMagnitude < 1e-10f)
                        _allNormals[i] = Vector3.up;
                    else
                        _allNormals[i].Normalize();
                }
                _cutMesh.SetNormals(_allNormals);
            }

            _cutMesh.SetTriangles(_allTris, 0);

            if (_allNormals.Count != _allVerts.Count)
                _cutMesh.RecalculateNormals();

            _cutMesh.RecalculateBounds();
        }

        public void RebuildNow()
        {
            RebuildMesh();
        }

        /// <summary>
        /// 切面顶点数
        /// </summary>
        public int VertexCount => _allVerts.Count;

        /// <summary>
        /// 切面三角形数
        /// </summary>
        public int TriangleCount => _allTris.Count / 3;
    }
}
