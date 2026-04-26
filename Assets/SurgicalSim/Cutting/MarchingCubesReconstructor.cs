// MarchingCubesReconstructor.cs
// GPU 加速的 MC 重建器
// DensitySplat.compute → MarchingCubes.compute → Mesh
// 全部在 GPU 上完成，CPU 只负责 readback 结果

using UnityEngine;
using UnityEngine.Rendering;
using SurgicalSim.Core;

namespace SurgicalSim.Cutting
{
    public class MarchingCubesReconstructor : System.IDisposable
    {
        // Compute Shaders
        ComputeShader _mcShader;
        ComputeShader _splatShader;
        int _kMarch, _kClear, _kSplat, _kConvert;

        // GPU Buffers
        ComputeBuffer _pointsBuffer;    // float4: xyz=pos, w=density
        ComputeBuffer _triangleBuffer;  // MC 输出三角形
        ComputeBuffer _triCountBuffer;  // MC 输出三角形数量
        ComputeBuffer _vertPosBuffer;   // 顶点位置 (float3)
        ComputeBuffer _vertActiveBuffer;// 顶点活跃标记 (int)
        ComputeBuffer _densityIntBuffer;// 整数密度场（原子加法用）

        int _resolution;
        Mesh _mesh;
        bool _initialized = false;
        bool _dirty = true;
        float _isoLevel = 0.5f;
        float _kernelRadius;

        Vector3 _origin;
        float _cellSize;

        int _numParticles;

        public Mesh OutputMesh => _mesh;
        public bool IsInitialized => _initialized;

        /// <summary>
        /// 初始化（需要两个 compute shader）
        /// </summary>
        public void Init(ComputeShader mcShader, ComputeShader splatShader,
                         int resolution, TetMeshData data)
        {
            _mcShader = mcShader;
            _splatShader = splatShader;
            _resolution = resolution;
            _numParticles = data.NumParticles;

            // 找 kernel
            _kMarch = _mcShader.FindKernel("March");
            _kClear = _splatShader.FindKernel("ClearDensity");
            _kSplat = _splatShader.FindKernel("SplatVertices");
            _kConvert = _splatShader.FindKernel("ConvertToFloat4");

            _mesh = new Mesh();
            _mesh.indexFormat = IndexFormat.UInt32;

            // 计算网格参数
            ComputeGridParams(data);

            // 估算核半径
            _kernelRadius = EstimateKernelRadius(data) * 1.8f;

            // 创建 GPU buffers
            int numPoints = resolution * resolution * resolution;
            int numVoxels = (resolution - 1) * (resolution - 1) * (resolution - 1);
            int maxTriangles = numVoxels * 5;

            _pointsBuffer = new ComputeBuffer(numPoints, sizeof(float) * 4);
            _triangleBuffer = new ComputeBuffer(maxTriangles, sizeof(float) * 3 * 3, ComputeBufferType.Append);
            _triCountBuffer = new ComputeBuffer(1, sizeof(int), ComputeBufferType.Raw);
            _vertPosBuffer = new ComputeBuffer(_numParticles, sizeof(float) * 3);
            _vertActiveBuffer = new ComputeBuffer(_numParticles, sizeof(int));
            _densityIntBuffer = new ComputeBuffer(numPoints, sizeof(int));

            _initialized = true;
            _dirty = true;

            Debug.Log($"[MCReconstructor] GPU 初始化 | 分辨率: {resolution}³ | " +
                      $"核半径: {_kernelRadius:F4} | isoLevel: {_isoLevel}");
        }

        public void MarkDirtyRegion(Vector3 center, float radius)
        {
            _dirty = true;
        }

        public void MarkFullRebuild()
        {
            _dirty = true;
        }

        /// <summary>
        /// GPU 重建：DensitySplat → MarchingCubes → Mesh
        /// </summary>
        public void Rebuild(TetMeshData data)
        {
            if (!_initialized || !_dirty) return;

            var sw = System.Diagnostics.Stopwatch.StartNew();

            // 0. 更新网格参数（位置可能已变化）
            ComputeGridParams(data);

            // 1. 上传顶点位置和活跃状态到 GPU
            UploadVertexData(data);

            // 2. GPU: 清空整数密度场
            SetSplatUniforms();
            _splatShader.SetBuffer(_kClear, "_DensityInt", _densityIntBuffer);
            int clearGroups = Mathf.CeilToInt(_resolution / 8f);
            _splatShader.Dispatch(_kClear, clearGroups, clearGroups, clearGroups);

            // 3. GPU: 顶点密度 splatting（原子整数加法）
            _splatShader.SetBuffer(_kSplat, "_DensityInt", _densityIntBuffer);
            _splatShader.SetBuffer(_kSplat, "_Vertices", _vertPosBuffer);
            _splatShader.SetBuffer(_kSplat, "_VertexActive", _vertActiveBuffer);
            _splatShader.SetInt("_NumVertices", _numParticles);
            int splatGroups = Mathf.CeilToInt(_numParticles / 256f);
            _splatShader.Dispatch(_kSplat, splatGroups, 1, 1);

            // 4. GPU: 整数密度 → float4 转换
            _splatShader.SetBuffer(_kConvert, "_DensityInt", _densityIntBuffer);
            _splatShader.SetBuffer(_kConvert, "_Points", _pointsBuffer);
            _splatShader.Dispatch(_kConvert, clearGroups, clearGroups, clearGroups);

            // 5. GPU: Marching Cubes
            _triangleBuffer.SetCounterValue(0);
            _mcShader.SetBuffer(_kMarch, "points", _pointsBuffer);
            _mcShader.SetBuffer(_kMarch, "triangles", _triangleBuffer);
            _mcShader.SetInt("numPointsPerAxis", _resolution);
            _mcShader.SetFloat("isoLevel", _isoLevel);
            int mcGroups = Mathf.CeilToInt((_resolution - 1) / 8f);
            _mcShader.Dispatch(_kMarch, mcGroups, mcGroups, mcGroups);

            // 5. Readback 三角形数量
            ComputeBuffer.CopyCount(_triangleBuffer, _triCountBuffer, 0);
            int[] triCountArray = { 0 };
            _triCountBuffer.GetData(triCountArray);
            int numTris = triCountArray[0];

            if (numTris == 0)
            {
                _mesh.Clear();
                _dirty = false;
                return;
            }

            // 6. Readback 三角形数据
            var triData = new MCTriangle[numTris];
            _triangleBuffer.GetData(triData, 0, 0, numTris);

            // 7. 构建 Unity Mesh
            BuildMesh(triData, numTris);

            sw.Stop();
            _dirty = false;

            if (sw.ElapsedMilliseconds > 5)
                Debug.Log($"[MCReconstructor] GPU 重建: {numTris} 三角形 | {sw.ElapsedMilliseconds}ms");
        }

        // ── 内部方法 ─────────────────────────────────────────

        void ComputeGridParams(TetMeshData data)
        {
            Vector3 min = Vector3.one * float.MaxValue;
            Vector3 max = Vector3.one * float.MinValue;

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;
                for (int j = 0; j < 4; j++)
                {
                    Vector3 p = data.Positions[data.TetIds[t * 4 + j]];
                    min = Vector3.Min(min, p);
                    max = Vector3.Max(max, p);
                }
            }

            Vector3 size = max - min;
            float maxDim = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
            float padding = maxDim * 0.15f;
            _origin = min - Vector3.one * padding;
            size += Vector3.one * padding * 2f;
            _cellSize = Mathf.Max(size.x, Mathf.Max(size.y, size.z)) / (_resolution - 1);
        }

        void UploadVertexData(TetMeshData data)
        {
            // 标记活跃顶点
            var active = new int[_numParticles];
            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;
                for (int j = 0; j < 4; j++)
                    active[data.TetIds[t * 4 + j]] = 1;
            }

            var positions = new Vector3[_numParticles];
            System.Array.Copy(data.Positions, positions, _numParticles);

            _vertPosBuffer.SetData(positions);
            _vertActiveBuffer.SetData(active);
        }

        void SetSplatUniforms()
        {
            _splatShader.SetInt("_NumPointsPerAxis", _resolution);
            _splatShader.SetFloat("_CellSize", _cellSize);
            // Unity SetFloats 对 float3 有 padding 问题，用 SetVector 更安全
            _splatShader.SetVector("_Origin", new Vector4(_origin.x, _origin.y, _origin.z, 0));
            _splatShader.SetFloat("_KernelRadius", _kernelRadius);
        }

        void BuildMesh(MCTriangle[] triData, int count)
        {
            var vertices = new Vector3[count * 3];
            var triangles = new int[count * 3];

            for (int i = 0; i < count; i++)
            {
                vertices[i * 3 + 0] = triData[i].vertexA;
                vertices[i * 3 + 1] = triData[i].vertexB;
                vertices[i * 3 + 2] = triData[i].vertexC;
                triangles[i * 3 + 0] = i * 3 + 0;
                triangles[i * 3 + 1] = i * 3 + 1;
                triangles[i * 3 + 2] = i * 3 + 2;
            }

            _mesh.Clear();
            _mesh.vertices = vertices;
            _mesh.triangles = triangles;
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
        }

        float EstimateKernelRadius(TetMeshData data)
        {
            float totalLen = 0f;
            int count = 0;
            for (int t = 0; t < data.NumTets && count < 3000; t++)
            {
                if (!data.TetActive[t]) continue;
                Vector3 p0 = data.Positions[data.TetIds[t*4]];
                Vector3 p1 = data.Positions[data.TetIds[t*4+1]];
                Vector3 p2 = data.Positions[data.TetIds[t*4+2]];
                Vector3 p3 = data.Positions[data.TetIds[t*4+3]];
                totalLen += (p1-p0).magnitude + (p2-p0).magnitude + (p3-p0).magnitude
                          + (p2-p1).magnitude + (p3-p1).magnitude + (p3-p2).magnitude;
                count += 6;
            }
            return (count > 0) ? totalLen / count : 0.01f;
        }

        public void Dispose()
        {
            _pointsBuffer?.Release();
            _triangleBuffer?.Release();
            _triCountBuffer?.Release();
            _vertPosBuffer?.Release();
            _vertActiveBuffer?.Release();
            _densityIntBuffer?.Release();
        }

        struct MCTriangle
        {
            public Vector3 vertexC;
            public Vector3 vertexB;
            public Vector3 vertexA;
        }
    }
}
