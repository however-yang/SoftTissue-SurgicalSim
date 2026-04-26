// VNACutter.cs
// Virtual Node Algorithm 主算法
// 流程: computeIntersections → split → merge → subdivide
// 参考: Fatalite/CudaMeshCutting (Cutting.cuh)
// 移植为纯 C# CPU 版，GPU 交叉检测在 VNACutCompute.compute 中

using System;
using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;

namespace SurgicalSim.Cutting
{
    public class VNACutter
    {
        TetMeshData _data;

        // 切割平面定义（由 CuttingTool 提供）
        Vector3 _planePoint;
        Vector3 _planeNormal;

        // 内部状态
        Dictionary<IntersectionKey, IntersectionWeight> _intersections;
        Dictionary<IntersectionKey, List<int>> _boundary2TetIds;

        // 累积的切面三角形
        List<Vector3> _cutSurfaceVerts = new List<Vector3>();
        List<int>     _cutSurfaceTris  = new List<int>();
        List<Vector3> _cutSurfaceNormals = new List<Vector3>();

        // ════════════════════════════════════════════════════════
        // 初始化
        // ════════════════════════════════════════════════════════
        public void Init(TetMeshData data)
        {
            _data = data;
        }

        // ════════════════════════════════════════════════════════
        // 主入口：沿切割平面执行 VNA 切割
        // planePoint: 切割平面上的一点
        // planeNormal: 切割平面法线
        // affectedTets: 预筛选的受影响 tet 列表（由 CuttingTool 提供）
        // ════════════════════════════════════════════════════════
        public VNACutResult ExecuteCut(Vector3 planePoint, Vector3 planeNormal, List<int> affectedTets)
        {
            var sw = System.Diagnostics.Stopwatch.StartNew();
            var result = new VNACutResult();

            if (affectedTets == null || affectedTets.Count == 0)
                return result;

            _planePoint = planePoint;
            _planeNormal = planeNormal.normalized;

            // ── Step 1: 计算交叉点 ────────────────────────────
            _intersections = new Dictionary<IntersectionKey, IntersectionWeight>();
            _boundary2TetIds = new Dictionary<IntersectionKey, List<int>>();

            ComputeIntersections(affectedTets);
            result.IntersectionCount = _intersections.Count;

            if (_intersections.Count == 0)
                return result;

            // ── Step 2: Split — 找出被切的 tet 的连通分量 ──────
            var cutTets = new HashSet<int>();
            var cutElements = Split(affectedTets, cutTets);
            result.CutTetCount = cutTets.Count;
            result.CutElementCount = cutElements.Count;

            if (cutElements.Count == 0)
                return result;

            // ── Step 3: Merge — 用 UnionFind 合并共享顶点 ─────
            var newPositions = new List<Vector3>();
            var newTetIds = new List<int>(); // flat, 每 4 个一组

            Merge(cutElements, cutTets, newPositions, newTetIds);

            // ── Step 4: 生成切面三角形 ────────────────────────
            GenerateCutSurface(cutElements, cutTets, result);

            // ── 打包结果 ──────────────────────────────────────
            result.NewPositions = newPositions.ToArray();
            result.NewTetIds = newTetIds.ToArray();
            result.NewNumParticles = newPositions.Count;
            result.NewNumTets = newTetIds.Count / 4;

            sw.Stop();
            result.ElapsedMs = (float)sw.Elapsed.TotalMilliseconds;

            Debug.Log($"[VNACutter] 切割完成 | 交叉点: {result.IntersectionCount} | " +
                      $"切割 Tet: {result.CutTetCount} | 分区: {result.CutElementCount} | " +
                      $"新顶点: {result.NewNumParticles} | 新 Tet: {result.NewNumTets} | " +
                      $"耗时: {result.ElapsedMs:F1}ms");

            return result;
        }

        // ════════════════════════════════════════════════════════
        // Step 1: 计算交叉点
        // 对每个受影响 tet 的边，判断是否被切割平面穿过
        // ════════════════════════════════════════════════════════
        void ComputeIntersections(List<int> affectedTets)
        {
            var processedEdges = new HashSet<long>();

            foreach (int t in affectedTets)
            {
                if (!_data.TetActive[t]) continue;

                int tetBase = t * 4;

                // 建立 tet → boundary 映射
                var tetKey = IntersectionKey.FromTet(
                    _data.TetIds[tetBase], _data.TetIds[tetBase+1],
                    _data.TetIds[tetBase+2], _data.TetIds[tetBase+3]);
                AddBoundaryMapping(tetKey, t);

                // 检测 6 条边
                for (int e = 0; e < 6; e++)
                {
                    TetTopology.GetEdge(_data.TetIds, tetBase, e, out int a, out int b);
                    long edgeKey = TetTopology.EdgeKey(a, b);

                    if (processedEdges.Contains(edgeKey)) continue;
                    processedEdges.Add(edgeKey);

                    // 判断这条边是否被切割平面穿过
                    Vector3 posA = _data.Positions[a];
                    Vector3 posB = _data.Positions[b];

                    float dA = Vector3.Dot(posA - _planePoint, _planeNormal);
                    float dB = Vector3.Dot(posB - _planePoint, _planeNormal);

                    // 两端在平面不同侧 → 有交叉
                    if (dA * dB < 0)
                    {
                        // 计算交叉点的重心坐标
                        float absA = Mathf.Abs(dA);
                        float absB = Mathf.Abs(dB);
                        float wA = absB / (absA + absB); // 靠近 A 的权重
                        float wB = 1f - wA;

                        var key = IntersectionKey.FromEdge(a, b);
                        var weight = new IntersectionWeight { w0 = wA, w1 = wB };
                        _intersections[key] = weight;

                        // 建立边 → tet 的映射
                        AddBoundaryMapping(key, t);
                    }
                }

                // 检测 4 个面（面-边交叉）
                for (int f = 0; f < 4; f++)
                {
                    TetTopology.GetFace(_data.TetIds, tetBase, f, out int fa, out int fb, out int fc);
                    var faceKey = IntersectionKey.FromFace(fa, fb, fc);
                    AddBoundaryMapping(faceKey, t);
                }
            }
        }

        void AddBoundaryMapping(IntersectionKey key, int tetIdx)
        {
            if (!_boundary2TetIds.ContainsKey(key))
                _boundary2TetIds[key] = new List<int>();
            var list = _boundary2TetIds[key];
            if (!list.Contains(tetIdx))
                list.Add(tetIdx);
        }

        // ════════════════════════════════════════════════════════
        // Step 2: Split — 生成 CutElement
        // 对每个被切 tet，通过 BFS 找连通分量
        // 参考: CudaMeshCutting Cutting.cuh::split()
        // ════════════════════════════════════════════════════════
        List<CutElement> Split(List<int> affectedTets, HashSet<int> cutTets)
        {
            cutTets.Clear();

            // 找出所有被切的 tet（其边界上有交叉点的 tet）
            foreach (var kv in _boundary2TetIds)
            {
                if (_intersections.ContainsKey(kv.Key))
                {
                    foreach (int t in kv.Value)
                        cutTets.Add(t);
                }
            }

            var cutElements = new List<CutElement>();

            foreach (int t in cutTets)
            {
                if (!_data.TetActive[t]) continue;

                int tetBase = t * 4;
                bool[] added = new bool[4];
                int[] tetVerts = {
                    _data.TetIds[tetBase], _data.TetIds[tetBase+1],
                    _data.TetIds[tetBase+2], _data.TetIds[tetBase+3]
                };

                for (int j = 0; j < 4; j++)
                {
                    if (added[j]) continue;

                    // BFS：找所有通过未被切断的边连接的顶点
                    var ce = new CutElement(t, false);
                    var stack = new Stack<int>();
                    stack.Push(j);

                    while (stack.Count > 0)
                    {
                        int top = stack.Pop();
                        if (added[top]) continue;

                        ce.SetSub(top, true);
                        added[top] = true;

                        // 检查所有邻居
                        for (int k = 0; k < 4; k++)
                        {
                            if (added[k]) continue;

                            // 检查 edge (top, k) 是否有交叉点
                            int va = tetVerts[top], vb = tetVerts[k];
                            var edgeKey = IntersectionKey.FromEdge(va, vb);

                            // 如果这条边没有被切断，则两端连通
                            if (!_intersections.ContainsKey(edgeKey))
                                stack.Push(k);
                        }
                    }

                    cutElements.Add(ce);
                }
            }

            return cutElements;
        }

        // ════════════════════════════════════════════════════════
        // Step 3: Merge — 用 UnionFind 合并共享顶点
        // 同侧的顶点共享同一个新顶点，异侧的使用不同副本
        // 参考: CudaMeshCutting Cutting.cuh::merge()
        // ════════════════════════════════════════════════════════
        void Merge(List<CutElement> cutElements, HashSet<int> cutTets,
                   List<Vector3> newPositions, List<int> newTetIds)
        {
            int origNumVerts = _data.NumParticles;
            int totalVirtualNodes = origNumVerts + 4 * cutElements.Count;

            var uf = new UnionFind(totalVirtualNodes);

            // Face-Face merging: 确保同侧的顶点合并
            // 参考 CudaMeshCutting 的 faceNode2NewNode 逻辑
            // key = (face排序索引[3], materialNode, node) → newNodeId
            var faceNodeMap = new Dictionary<(int,int,int,int,int), int>();

            int virtualBase = origNumVerts;
            for (int ceIdx = 0; ceIdx < cutElements.Count; ceIdx++)
            {
                var ce = cutElements[ceIdx];
                int tetBase = ce.parentTetIndex * 4;
                int[] tetVerts = {
                    _data.TetIds[tetBase], _data.TetIds[tetBase+1],
                    _data.TetIds[tetBase+2], _data.TetIds[tetBase+3]
                };

                int vBase = virtualBase + ceIdx * 4;

                for (int f = 0; f < 4; f++) // 遍历 4 个面
                {
                    int fi0 = TetTopology.FaceIndices[f, 0];
                    int fi1 = TetTopology.FaceIndices[f, 1];
                    int fi2 = TetTopology.FaceIndices[f, 2];

                    // 排序面的全局顶点索引
                    int fa = tetVerts[fi0], fb = tetVerts[fi1], fc = tetVerts[fi2];
                    int fMin, fMid, fMax;
                    SortThree(fa, fb, fc, out fMin, out fMid, out fMax);

                    // 对面上的每个顶点做 merge
                    int[] faceLocalIndices = { fi0, fi1, fi2 };
                    foreach (int fij in faceLocalIndices)
                    {
                        if (!ce.GetSub(fij)) continue; // 这个子元素不包含这个顶点

                        int materialNode = tetVerts[fij];
                        int newId = vBase + fij;

                        // 将虚拟节点与原始节点合并
                        uf.Merge(newId, materialNode);

                        // 与相邻 tet 的对应顶点合并
                        foreach (int fik in faceLocalIndices)
                        {
                            int nodeId = tetVerts[fik];
                            int virtualId = vBase + fik;

                            // 生成唯一的 face-material-node key
                            var key = (fMin, fMid, fMax, materialNode, nodeId);

                            if (faceNodeMap.TryGetValue(key, out int existingId))
                            {
                                uf.Merge(existingId, virtualId);
                            }
                            else
                            {
                                faceNodeMap[key] = virtualId;
                            }
                        }
                    }
                }
            }

            // 生成新的网格
            var nodeMapping = new Dictionary<int, int>(); // uf.Find(virtualId) → newPositions index

            // 处理被切的 tet（使用虚拟节点）
            virtualBase = origNumVerts;
            for (int ceIdx = 0; ceIdx < cutElements.Count; ceIdx++)
            {
                var ce = cutElements[ceIdx];
                int tetBase = ce.parentTetIndex * 4;
                int vBase = virtualBase + ceIdx * 4;

                int[] newTet = new int[4];
                for (int i = 0; i < 4; i++)
                {
                    int virtualId = vBase + i;
                    int rootId = uf.Find(virtualId);

                    if (nodeMapping.TryGetValue(rootId, out int mappedIdx))
                    {
                        newTet[i] = mappedIdx;
                    }
                    else
                    {
                        int newIdx = newPositions.Count;
                        nodeMapping[rootId] = newIdx;
                        newPositions.Add(_data.Positions[_data.TetIds[tetBase + i]]);
                        newTet[i] = newIdx;
                    }
                }
                newTetIds.AddRange(newTet);
            }

            // 处理未被切的 tet（保持原样，但使用新的顶点索引）
            for (int t = 0; t < _data.NumTets; t++)
            {
                if (!_data.TetActive[t]) continue;
                if (cutTets.Contains(t)) continue;

                int tetBase = t * 4;
                int[] newTet = new int[4];
                for (int i = 0; i < 4; i++)
                {
                    int origVert = _data.TetIds[tetBase + i];
                    int rootId = uf.Find(origVert);

                    if (nodeMapping.TryGetValue(rootId, out int mappedIdx))
                    {
                        newTet[i] = mappedIdx;
                    }
                    else
                    {
                        int newIdx = newPositions.Count;
                        nodeMapping[rootId] = newIdx;
                        newPositions.Add(_data.Positions[origVert]);
                        newTet[i] = newIdx;
                    }
                }
                newTetIds.AddRange(newTet);
            }
        }

        // ════════════════════════════════════════════════════════
        // Step 4: 生成切面三角形
        // 在被切断的 tet 面上插值出切面几何
        // ════════════════════════════════════════════════════════
        void GenerateCutSurface(List<CutElement> cutElements, HashSet<int> cutTets,
                                VNACutResult result)
        {
            foreach (int t in cutTets)
            {
                if (!_data.TetActive[t]) continue;

                int tetBase = t * 4;
                Vector3[] tetPos = new Vector3[4];
                for (int i = 0; i < 4; i++)
                    tetPos[i] = _data.Positions[_data.TetIds[tetBase + i]];

                // 找到这个 tet 中所有被切断的边的交叉点
                var cutPoints = new List<Vector3>();
                for (int e = 0; e < 6; e++)
                {
                    TetTopology.GetEdge(_data.TetIds, tetBase, e, out int a, out int b);
                    var edgeKey = IntersectionKey.FromEdge(a, b);

                    if (_intersections.TryGetValue(edgeKey, out var weight))
                    {
                        // 插值位置
                        Vector3 posA = _data.Positions[a];
                        Vector3 posB = _data.Positions[b];
                        Vector3 cutPos;

                        // 根据 edge key 的排序方向决定权重
                        if (a < b)
                            cutPos = posA * weight.w0 + posB * weight.w1;
                        else
                            cutPos = posA * weight.w1 + posB * weight.w0;

                        cutPoints.Add(cutPos);
                    }
                }

                // 根据交叉点数量生成三角形
                if (cutPoints.Count >= 3)
                {
                    int baseIdx = result.CutSurfaceVerts.Count;
                    result.CutSurfaceVerts.AddRange(cutPoints);

                    // 计算切面法线（使用切割平面法线）
                    Vector3 normal = _planeNormal;

                    if (cutPoints.Count == 3)
                    {
                        // 一个三角形
                        result.CutSurfaceTris.Add(baseIdx);
                        result.CutSurfaceTris.Add(baseIdx + 1);
                        result.CutSurfaceTris.Add(baseIdx + 2);
                        result.CutSurfaceNormals.Add(normal);
                        result.CutSurfaceNormals.Add(normal);
                        result.CutSurfaceNormals.Add(normal);
                    }
                    else if (cutPoints.Count == 4)
                    {
                        // 两个三角形（四边形切面）
                        // 需要正确排序顶点以形成凸四边形
                        OrderQuadVerts(cutPoints, normal);

                        result.CutSurfaceTris.Add(baseIdx);
                        result.CutSurfaceTris.Add(baseIdx + 1);
                        result.CutSurfaceTris.Add(baseIdx + 2);

                        result.CutSurfaceTris.Add(baseIdx);
                        result.CutSurfaceTris.Add(baseIdx + 2);
                        result.CutSurfaceTris.Add(baseIdx + 3);

                        for (int i = 0; i < 4; i++)
                            result.CutSurfaceNormals.Add(normal);
                    }
                }
            }
        }

        /// <summary>
        /// 对四边形的 4 个顶点按逆时针排序
        /// </summary>
        void OrderQuadVerts(List<Vector3> verts, Vector3 normal)
        {
            if (verts.Count != 4) return;

            Vector3 center = (verts[0] + verts[1] + verts[2] + verts[3]) * 0.25f;

            // 建立局部坐标系
            Vector3 u = (verts[0] - center).normalized;
            Vector3 v = Vector3.Cross(normal, u).normalized;

            // 计算每个顶点的角度
            float[] angles = new float[4];
            for (int i = 0; i < 4; i++)
            {
                Vector3 d = verts[i] - center;
                angles[i] = Mathf.Atan2(Vector3.Dot(d, v), Vector3.Dot(d, u));
            }

            // 按角度排序
            for (int i = 0; i < 3; i++)
                for (int j = i + 1; j < 4; j++)
                    if (angles[j] < angles[i])
                    {
                        float ta = angles[i]; angles[i] = angles[j]; angles[j] = ta;
                        Vector3 tv = verts[i]; verts[i] = verts[j]; verts[j] = tv;
                    }
        }

        // ════════════════════════════════════════════════════════
        // 工具方法
        // ════════════════════════════════════════════════════════
        static void SortThree(int a, int b, int c, out int min, out int mid, out int max)
        {
            if (a > b) { int t = a; a = b; b = t; }
            if (b > c) { int t = b; b = c; c = t; }
            if (a > b) { int t = a; a = b; b = t; }
            min = a; mid = b; max = c;
        }

        // ════════════════════════════════════════════════════════
        // 公开属性
        // ════════════════════════════════════════════════════════
        public List<Vector3> CutSurfaceVerts => _cutSurfaceVerts;
        public List<int> CutSurfaceTris => _cutSurfaceTris;
    }
}
