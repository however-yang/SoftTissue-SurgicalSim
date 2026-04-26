// VNACutData.cs
// Virtual Node Algorithm 核心数据结构
// 参考: Fatalite/CudaMeshCutting - Adaptive Virtual Node Algorithm
// 参考论文: Sifakis et al. 2007 "Adaptive Virtual Node Algorithm"

using System;
using System.Collections.Generic;
using UnityEngine;

namespace SurgicalSim.Cutting
{
    // ════════════════════════════════════════════════════════
    // CutElement：被切割平面穿过的 tet 的材料分区信息
    // subElements[i] = true 表示该 tet 的第 i 个顶点属于当前分区
    // 一个 tet 被切后会生成 2+ 个 CutElement（每个分区一个）
    // ════════════════════════════════════════════════════════
    public struct CutElement
    {
        public int parentTetIndex;       // 原始 tet 索引
        public bool sub0, sub1, sub2, sub3; // 4 个子元素的存活状态

        public CutElement(int parentIdx, bool fill = true)
        {
            parentTetIndex = parentIdx;
            sub0 = sub1 = sub2 = sub3 = fill;
        }

        public bool GetSub(int i)
        {
            switch (i) { case 0: return sub0; case 1: return sub1; case 2: return sub2; case 3: return sub3; }
            return false;
        }
        public void SetSub(int i, bool val)
        {
            switch (i) { case 0: sub0 = val; break; case 1: sub1 = val; break; case 2: sub2 = val; break; case 3: sub3 = val; break; }
        }

        public int NumPieces => (sub0 ? 1 : 0) + (sub1 ? 1 : 0) + (sub2 ? 1 : 0) + (sub3 ? 1 : 0);
    }

    // ════════════════════════════════════════════════════════
    // UnionFind：用于 VNA merge 阶段的顶点合并
    // ════════════════════════════════════════════════════════
    public class UnionFind
    {
        int[] _parent;
        int[] _rank;

        public UnionFind(int size)
        {
            _parent = new int[size];
            _rank = new int[size];
            for (int i = 0; i < size; i++)
                _parent[i] = i;
        }

        public int Find(int x)
        {
            while (_parent[x] != x)
            {
                _parent[x] = _parent[_parent[x]]; // path compression
                x = _parent[x];
            }
            return x;
        }

        public void Merge(int a, int b)
        {
            int ra = Find(a), rb = Find(b);
            if (ra == rb) return;
            if (_rank[ra] < _rank[rb]) { int t = ra; ra = rb; rb = t; }
            _parent[rb] = ra;
            if (_rank[ra] == _rank[rb]) _rank[ra]++;
        }
    }

    // ════════════════════════════════════════════════════════
    // 交叉点信息：边/面/tet 上的交叉点及其重心坐标
    // key = 排序后的顶点索引（2/3/4个），value = 重心坐标权重
    // ════════════════════════════════════════════════════════
    public struct IntersectionKey : IEquatable<IntersectionKey>, IComparable<IntersectionKey>
    {
        public int i0, i1, i2, i3; // 排序后的索引，未使用的设为 -1

        public static IntersectionKey FromEdge(int a, int b)
        {
            if (a > b) { int t = a; a = b; b = t; }
            return new IntersectionKey { i0 = a, i1 = b, i2 = -1, i3 = -1 };
        }

        public static IntersectionKey FromFace(int a, int b, int c)
        {
            // 排序 3 个索引
            if (a > b) { int t = a; a = b; b = t; }
            if (b > c) { int t = b; b = c; c = t; }
            if (a > b) { int t = a; a = b; b = t; }
            return new IntersectionKey { i0 = a, i1 = b, i2 = c, i3 = -1 };
        }

        public static IntersectionKey FromTet(int a, int b, int c, int d)
        {
            // 排序 4 个索引
            if (a > b) { int t = a; a = b; b = t; }
            if (c > d) { int t = c; c = d; d = t; }
            if (a > c) { int t = a; a = c; c = t; }
            if (b > d) { int t = b; b = d; d = t; }
            if (b > c) { int t = b; b = c; c = t; }
            return new IntersectionKey { i0 = a, i1 = b, i2 = c, i3 = d };
        }

        public bool Equals(IntersectionKey o) => i0 == o.i0 && i1 == o.i1 && i2 == o.i2 && i3 == o.i3;
        public override bool Equals(object obj) => obj is IntersectionKey k && Equals(k);
        public override int GetHashCode() => i0 * 73856093 ^ i1 * 19349663 ^ i2 * 83492791 ^ i3 * 49979687;
        public int CompareTo(IntersectionKey o)
        {
            int c = i0.CompareTo(o.i0); if (c != 0) return c;
            c = i1.CompareTo(o.i1); if (c != 0) return c;
            c = i2.CompareTo(o.i2); if (c != 0) return c;
            return i3.CompareTo(o.i3);
        }
    }

    public struct IntersectionWeight
    {
        public float w0, w1, w2, w3; // 重心坐标权重

        public float Get(int i) { switch (i) { case 0: return w0; case 1: return w1; case 2: return w2; case 3: return w3; } return 0; }
        public void Set(int i, float v) { switch (i) { case 0: w0 = v; break; case 1: w1 = v; break; case 2: w2 = v; break; case 3: w3 = v; break; } }
    }

    // ════════════════════════════════════════════════════════
    // VNA 切割结果
    // ════════════════════════════════════════════════════════
    public class VNACutResult
    {
        // 新的顶点和 tet 索引（完整替换原有网格）
        public Vector3[] NewPositions;
        public int[]     NewTetIds;    // flat array, 每 4 个一组
        public int       NewNumParticles;
        public int       NewNumTets;

        // 切面三角形（用于渲染内部组织）
        public List<Vector3> CutSurfaceVerts = new List<Vector3>();
        public List<int>     CutSurfaceTris  = new List<int>();
        public List<Vector3> CutSurfaceNormals = new List<Vector3>();

        // 被切断的边约束
        public HashSet<long> DisabledEdges = new HashSet<long>();

        // 诊断
        public int IntersectionCount;
        public int CutTetCount;
        public int CutElementCount;
        public float ElapsedMs;
    }

    // ════════════════════════════════════════════════════════
    // Tet 拓扑工具
    // ════════════════════════════════════════════════════════
    public static class TetTopology
    {
        // tet 的 4 个面的局部顶点索引
        // FaceIndices[faceIdx] = {v0, v1, v2}（面的 3 个顶点在 tet 中的局部索引）
        public static readonly int[,] FaceIndices = {
            {0, 2, 1},  // face 0: 对面顶点 3
            {0, 1, 3},  // face 1: 对面顶点 2
            {0, 3, 2},  // face 2: 对面顶点 1
            {1, 2, 3}   // face 3: 对面顶点 0
        };

        // FaceOpposite[faceIdx] = 该面对面的顶点局部索引
        public static readonly int[] FaceOpposite = { 3, 2, 1, 0 };

        // tet 的 6 条边的局部顶点索引
        public static readonly int[,] EdgePairs = {
            {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}
        };

        // 每个面包含的 3 条边在 EdgePairs 中的索引
        // Face i 由 3 个顶点构成，对应 3 条边
        public static readonly int[,] FaceEdges = {
            {1, 3, 0},  // face 0: edges (0,2),(1,2),(0,1)
            {0, 4, 2},  // face 1: edges (0,1),(1,3),(0,3)
            {2, 5, 1},  // face 2: edges (0,3),(2,3),(0,2)
            {3, 5, 4}   // face 3: edges (1,2),(2,3),(1,3)
        };

        /// <summary>
        /// 获取 tet 第 faceIdx 个面的 3 个全局顶点索引
        /// </summary>
        public static void GetFace(int[] tetIds, int tetBase, int faceIdx,
            out int a, out int b, out int c)
        {
            a = tetIds[tetBase + FaceIndices[faceIdx, 0]];
            b = tetIds[tetBase + FaceIndices[faceIdx, 1]];
            c = tetIds[tetBase + FaceIndices[faceIdx, 2]];
        }

        /// <summary>
        /// 获取 tet 第 edgeIdx 条边的 2 个全局顶点索引
        /// </summary>
        public static void GetEdge(int[] tetIds, int tetBase, int edgeIdx,
            out int a, out int b)
        {
            a = tetIds[tetBase + EdgePairs[edgeIdx, 0]];
            b = tetIds[tetBase + EdgePairs[edgeIdx, 1]];
        }

        /// <summary>
        /// 生成排序后的边 key
        /// </summary>
        public static long EdgeKey(int a, int b)
        {
            if (a > b) { int t = a; a = b; b = t; }
            return (long)a * 200000L + b;
        }
    }
}
