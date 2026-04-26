// SDFGenerator.cs
// 从活跃四面体生成平滑密度场（用于 Marching Cubes）
// 使用 metaball 风格的顶点 splatting：每个活跃顶点贡献平滑密度

using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim.Cutting
{
    public class SDFGenerator
    {
        int _res;
        float[] _density;     // 密度场 [res^3]（> 0 = 内部, < 0 = 外部）
        Vector3 _origin;      // 网格原点
        float _cellSize;      // 体素尺寸
        float _kernelRadius;  // splatting 核半径

        // 缓存活跃顶点标记
        bool[] _vertActive;

        public int Resolution => _res;
        public float[] Density => _density;
        public Vector3 Origin => _origin;
        public float CellSize => _cellSize;

        public SDFGenerator(int resolution)
        {
            _res = resolution;
        }

        /// <summary>
        /// 构建平滑密度场（完整重建）
        /// </summary>
        public void BuildFromActiveTets(TetMeshData data)
        {
            // 1. 计算活跃 tet 的 AABB
            Vector3 min = Vector3.one * float.MaxValue;
            Vector3 max = Vector3.one * float.MinValue;

            // 标记哪些顶点属于活跃 tet
            _vertActive = new bool[data.NumParticles];
            int activeCount = 0;

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;
                activeCount++;
                for (int j = 0; j < 4; j++)
                {
                    int vi = data.TetIds[t * 4 + j];
                    _vertActive[vi] = true;
                    Vector3 p = data.Positions[vi];
                    min = Vector3.Min(min, p);
                    max = Vector3.Max(max, p);
                }
            }

            if (activeCount == 0) return;

            // 2. AABB + padding
            Vector3 size = max - min;
            float maxDim = Mathf.Max(size.x, Mathf.Max(size.y, size.z));
            float padding = maxDim * 0.15f;
            min -= Vector3.one * padding;
            max += Vector3.one * padding;
            size = max - min;

            _origin = min;
            _cellSize = Mathf.Max(size.x, Mathf.Max(size.y, size.z)) / (_res - 1);

            // 核半径 = 基于平均边长，让相邻顶点的核能重叠
            _kernelRadius = EstimateKernelRadius(data) * 1.8f;

            // 3. 清空密度场
            int total = _res * _res * _res;
            _density = new float[total];
            // 初始全部为负（外部）
            for (int i = 0; i < total; i++) _density[i] = -1f;

            // 4. 顶点 splatting
            SplatActiveVertices(data);
        }

        /// <summary>
        /// 增量更新：只更新切割区域
        /// </summary>
        public void UpdateRegion(TetMeshData data, Vector3 center, float radius)
        {
            if (_density == null) { BuildFromActiveTets(data); return; }

            // 重新标记活跃顶点
            _vertActive = new bool[data.NumParticles];
            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;
                for (int j = 0; j < 4; j++)
                    _vertActive[data.TetIds[t * 4 + j]] = true;
            }

            // 扩展范围
            float r = Mathf.Max(radius * 4f, _kernelRadius * 2f);

            int ix0 = Mathf.Max(0, Mathf.FloorToInt((center.x - r - _origin.x) / _cellSize));
            int iy0 = Mathf.Max(0, Mathf.FloorToInt((center.y - r - _origin.y) / _cellSize));
            int iz0 = Mathf.Max(0, Mathf.FloorToInt((center.z - r - _origin.z) / _cellSize));
            int ix1 = Mathf.Min(_res - 1, Mathf.CeilToInt((center.x + r - _origin.x) / _cellSize));
            int iy1 = Mathf.Min(_res - 1, Mathf.CeilToInt((center.y + r - _origin.y) / _cellSize));
            int iz1 = Mathf.Min(_res - 1, Mathf.CeilToInt((center.z + r - _origin.z) / _cellSize));

            // 重置区域为外部
            for (int iz = iz0; iz <= iz1; iz++)
                for (int iy = iy0; iy <= iy1; iy++)
                    for (int ix = ix0; ix <= ix1; ix++)
                        _density[iz * _res * _res + iy * _res + ix] = -1f;

            // 重新 splat 区域内的活跃顶点
            SplatActiveVerticesInRegion(data, ix0, iy0, iz0, ix1, iy1, iz1);
        }

        /// <summary>
        /// 打包为 MC shader 需要的 float4（xyz=pos, w=density）
        /// </summary>
        public Vector4[] PackForMC()
        {
            int total = _res * _res * _res;
            var pts = new Vector4[total];
            for (int iz = 0; iz < _res; iz++)
                for (int iy = 0; iy < _res; iy++)
                    for (int ix = 0; ix < _res; ix++)
                    {
                        int idx = iz * _res * _res + iy * _res + ix;
                        Vector3 pos = _origin + new Vector3(ix, iy, iz) * _cellSize;
                        pts[idx] = new Vector4(pos.x, pos.y, pos.z, _density[idx]);
                    }
            return pts;
        }

        // ── 内部方法 ─────────────────────────────────────────

        void SplatActiveVertices(TetMeshData data)
        {
            float r = _kernelRadius;
            float r2 = r * r;
            int voxelRadius = Mathf.CeilToInt(r / _cellSize) + 1;

            for (int vi = 0; vi < data.NumParticles; vi++)
            {
                if (!_vertActive[vi]) continue;

                Vector3 p = data.Positions[vi];

                // 该顶点对应的中心体素
                int cx = Mathf.RoundToInt((p.x - _origin.x) / _cellSize);
                int cy = Mathf.RoundToInt((p.y - _origin.y) / _cellSize);
                int cz = Mathf.RoundToInt((p.z - _origin.z) / _cellSize);

                // 遍历核半径内的体素
                int x0 = Mathf.Max(0, cx - voxelRadius);
                int x1 = Mathf.Min(_res - 1, cx + voxelRadius);
                int y0 = Mathf.Max(0, cy - voxelRadius);
                int y1 = Mathf.Min(_res - 1, cy + voxelRadius);
                int z0 = Mathf.Max(0, cz - voxelRadius);
                int z1 = Mathf.Min(_res - 1, cz + voxelRadius);

                for (int iz = z0; iz <= z1; iz++)
                    for (int iy = y0; iy <= y1; iy++)
                        for (int ix = x0; ix <= x1; ix++)
                        {
                            Vector3 vp = _origin + new Vector3(ix, iy, iz) * _cellSize;
                            float dist2 = (vp - p).sqrMagnitude;
                            if (dist2 > r2) continue;

                            // 平滑核：Wyvill (1-d²/r²)³
                            float s = 1f - dist2 / r2;
                            float contribution = s * s * s;

                            int idx = iz * _res * _res + iy * _res + ix;
                            _density[idx] += contribution;
                        }
            }
        }

        void SplatActiveVerticesInRegion(TetMeshData data,
            int rx0, int ry0, int rz0, int rx1, int ry1, int rz1)
        {
            float r = _kernelRadius;
            float r2 = r * r;
            int voxelRadius = Mathf.CeilToInt(r / _cellSize) + 1;

            // 区域对应的世界坐标范围
            Vector3 regionMin = _origin + new Vector3(rx0, ry0, rz0) * _cellSize - Vector3.one * r;
            Vector3 regionMax = _origin + new Vector3(rx1, ry1, rz1) * _cellSize + Vector3.one * r;

            for (int vi = 0; vi < data.NumParticles; vi++)
            {
                if (!_vertActive[vi]) continue;

                Vector3 p = data.Positions[vi];

                // 检查顶点是否在扩展区域内
                if (p.x < regionMin.x || p.x > regionMax.x ||
                    p.y < regionMin.y || p.y > regionMax.y ||
                    p.z < regionMin.z || p.z > regionMax.z) continue;

                int cx = Mathf.RoundToInt((p.x - _origin.x) / _cellSize);
                int cy = Mathf.RoundToInt((p.y - _origin.y) / _cellSize);
                int cz = Mathf.RoundToInt((p.z - _origin.z) / _cellSize);

                int x0 = Mathf.Max(rx0, Mathf.Max(0, cx - voxelRadius));
                int x1 = Mathf.Min(rx1, Mathf.Min(_res - 1, cx + voxelRadius));
                int y0 = Mathf.Max(ry0, Mathf.Max(0, cy - voxelRadius));
                int y1 = Mathf.Min(ry1, Mathf.Min(_res - 1, cy + voxelRadius));
                int z0 = Mathf.Max(rz0, Mathf.Max(0, cz - voxelRadius));
                int z1 = Mathf.Min(rz1, Mathf.Min(_res - 1, cz + voxelRadius));

                for (int iz = z0; iz <= z1; iz++)
                    for (int iy = y0; iy <= y1; iy++)
                        for (int ix = x0; ix <= x1; ix++)
                        {
                            Vector3 vp = _origin + new Vector3(ix, iy, iz) * _cellSize;
                            float dist2 = (vp - p).sqrMagnitude;
                            if (dist2 > r2) continue;

                            float s = 1f - dist2 / r2;
                            float contribution = s * s * s;

                            int idx = iz * _res * _res + iy * _res + ix;
                            _density[idx] += contribution;
                        }
            }
        }

        /// <summary>
        /// 估算核半径：基于所有边的平均长度
        /// </summary>
        float EstimateKernelRadius(TetMeshData data)
        {
            float totalLen = 0f;
            int count = 0;

            // 采样前 500 个活跃 tet 的边
            for (int t = 0; t < data.NumTets && count < 3000; t++)
            {
                if (!data.TetActive[t]) continue;

                int i0 = data.TetIds[t * 4 + 0];
                int i1 = data.TetIds[t * 4 + 1];
                int i2 = data.TetIds[t * 4 + 2];
                int i3 = data.TetIds[t * 4 + 3];

                Vector3 p0 = data.Positions[i0];
                Vector3 p1 = data.Positions[i1];
                Vector3 p2 = data.Positions[i2];
                Vector3 p3 = data.Positions[i3];

                totalLen += (p1 - p0).magnitude;
                totalLen += (p2 - p0).magnitude;
                totalLen += (p3 - p0).magnitude;
                totalLen += (p2 - p1).magnitude;
                totalLen += (p3 - p1).magnitude;
                totalLen += (p3 - p2).magnitude;
                count += 6;
            }

            float avgEdge = (count > 0) ? totalLen / count : 0.01f;
            return avgEdge;
        }
    }
}
