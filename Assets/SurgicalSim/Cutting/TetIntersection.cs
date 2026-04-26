// TetIntersection.cs
// 四面体-几何体相交检测（用于切割）
// 支持：球-四面体、射线-四面体、空间加速网格

using System.Collections.Generic;
using UnityEngine;

namespace SurgicalSim.Cutting
{
    public static class TetIntersection
    {
        // ═══════════════════════════════════════════════════
        // 球与四面体相交检测（最常用 — 切割工具 tip）
        // 原理：如果球心到四面体的最近距离 < 半径，则相交
        // 简化实现：球心到 tet 中心距离 < (半径 + tet 外接球半径)
        // ═══════════════════════════════════════════════════
        public static bool SphereTet(Vector3 center, float radius,
            Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            // 快速检测：球心到 tet 中心距离
            Vector3 tetCenter = (p0 + p1 + p2 + p3) * 0.25f;
            float tetRadius = Mathf.Max(
                (p0 - tetCenter).magnitude,
                Mathf.Max((p1 - tetCenter).magnitude,
                Mathf.Max((p2 - tetCenter).magnitude,
                          (p3 - tetCenter).magnitude)));

            float dist = (center - tetCenter).magnitude;
            if (dist > radius + tetRadius) return false;

            // 精确检测：球心是否在 tet 内部，或距离 tet 任意面 < radius
            if (PointInTet(center, p0, p1, p2, p3)) return true;

            // 距离四个三角面
            if (PointTriDist(center, p0, p1, p2) < radius) return true;
            if (PointTriDist(center, p0, p1, p3) < radius) return true;
            if (PointTriDist(center, p0, p2, p3) < radius) return true;
            if (PointTriDist(center, p1, p2, p3) < radius) return true;

            return false;
        }

        // ═══════════════════════════════════════════════════
        // 点是否在四面体内部（用符号体积法）
        // ═══════════════════════════════════════════════════
        public static bool PointInTet(Vector3 p,
            Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            // 如果点在四面体内，则对 4 个子四面体的符号体积应该同号
            float d0 = SignedVolume(p, b, c, d);
            float d1 = SignedVolume(a, p, c, d);
            float d2 = SignedVolume(a, b, p, d);
            float d3 = SignedVolume(a, b, c, p);

            bool hasPos = d0 > 0 || d1 > 0 || d2 > 0 || d3 > 0;
            bool hasNeg = d0 < 0 || d1 < 0 || d2 < 0 || d3 < 0;
            return !(hasPos && hasNeg);
        }

        static float SignedVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            return Vector3.Dot(Vector3.Cross(b - a, c - a), d - a) / 6f;
        }

        // ═══════════════════════════════════════════════════
        // 点到三角形的最近距离
        // ═══════════════════════════════════════════════════
        static float PointTriDist(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
        {
            Vector3 ab = b - a, ac = c - a, ap = p - a;
            float d1 = Vector3.Dot(ab, ap), d2 = Vector3.Dot(ac, ap);
            if (d1 <= 0f && d2 <= 0f) return (p - a).magnitude;

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp), d4 = Vector3.Dot(ac, bp);
            if (d3 >= 0f && d4 <= d3) return (p - b).magnitude;

            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v2 = d1 / (d1 - d3);
                return (p - (a + v2 * ab)).magnitude;
            }

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp), d6 = Vector3.Dot(ac, cp);
            if (d6 >= 0f && d5 <= d6) return (p - c).magnitude;

            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w2 = d2 / (d2 - d6);
                return (p - (a + w2 * ac)).magnitude;
            }

            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w2 = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return (p - (b + w2 * (c - b))).magnitude;
            }

            float denom = 1f / (va + vb + vc);
            float v = vb * denom, w = vc * denom;
            Vector3 closest = a + ab * v + ac * w;
            return (p - closest).magnitude;
        }

        // ═══════════════════════════════════════════════════
        // 线段与四面体相交（连续切割用）
        // 检测线段是否穿过 tet 的任意三角面
        // ═══════════════════════════════════════════════════
        public static bool SegmentTet(Vector3 segA, Vector3 segB,
            Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
        {
            // tet 的 4 个面
            if (SegmentTriangle(segA, segB, p0, p1, p2)) return true;
            if (SegmentTriangle(segA, segB, p0, p1, p3)) return true;
            if (SegmentTriangle(segA, segB, p0, p2, p3)) return true;
            if (SegmentTriangle(segA, segB, p1, p2, p3)) return true;

            // 线段端点在 tet 内部
            if (PointInTet(segA, p0, p1, p2, p3)) return true;

            return false;
        }

        // Möller–Trumbore 线段-三角形相交
        static bool SegmentTriangle(Vector3 orig, Vector3 end,
            Vector3 v0, Vector3 v1, Vector3 v2)
        {
            Vector3 dir = end - orig;
            float segLen = dir.magnitude;
            if (segLen < 1e-8f) return false;
            dir /= segLen;

            Vector3 e1 = v1 - v0, e2 = v2 - v0;
            Vector3 h = Vector3.Cross(dir, e2);
            float a = Vector3.Dot(e1, h);
            if (a > -1e-6f && a < 1e-6f) return false;

            float f = 1f / a;
            Vector3 s = orig - v0;
            float u = f * Vector3.Dot(s, h);
            if (u < 0f || u > 1f) return false;

            Vector3 q = Vector3.Cross(s, e1);
            float v = f * Vector3.Dot(dir, q);
            if (v < 0f || u + v > 1f) return false;

            float t = f * Vector3.Dot(e2, q);
            return t > 0f && t < segLen;
        }

        // ═══════════════════════════════════════════════════
        // 批量查找：与球体/线段相交的所有活跃 tet
        // ═══════════════════════════════════════════════════
        public static List<int> FindTetsInSphere(Core.TetMeshData data,
            Vector3 center, float radius)
        {
            var result = new List<int>();
            float r2 = (radius + 0.1f); // 粗筛容差
            r2 *= r2;

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                // 粗筛：tet 中心到球心距离
                Vector3 tc = data.TetCenter(t);
                if ((tc - center).sqrMagnitude > r2 * 4f) continue;

                // 精确检测
                Vector3 p0 = data.Positions[data.TetIds[t*4]];
                Vector3 p1 = data.Positions[data.TetIds[t*4+1]];
                Vector3 p2 = data.Positions[data.TetIds[t*4+2]];
                Vector3 p3 = data.Positions[data.TetIds[t*4+3]];

                if (SphereTet(center, radius, p0, p1, p2, p3))
                    result.Add(t);
            }
            return result;
        }

        public static List<int> FindTetsAlongSegment(Core.TetMeshData data,
            Vector3 segA, Vector3 segB, float radius)
        {
            var result = new List<int>();

            // 线段 AABB 扩展
            Vector3 mn = Vector3.Min(segA, segB) - Vector3.one * radius;
            Vector3 mx = Vector3.Max(segA, segB) + Vector3.one * radius;

            for (int t = 0; t < data.NumTets; t++)
            {
                if (!data.TetActive[t]) continue;

                Vector3 p0 = data.Positions[data.TetIds[t*4]];
                Vector3 p1 = data.Positions[data.TetIds[t*4+1]];
                Vector3 p2 = data.Positions[data.TetIds[t*4+2]];
                Vector3 p3 = data.Positions[data.TetIds[t*4+3]];

                Vector3 tetMin = Vector3.Min(Vector3.Min(p0, p1), Vector3.Min(p2, p3));
                Vector3 tetMax = Vector3.Max(Vector3.Max(p0, p1), Vector3.Max(p2, p3));

                // 用 tet 的真实 AABB 做粗筛，避免中心点粗筛漏掉被刀轨扫中的大 tet。
                if (tetMax.x < mn.x || tetMin.x > mx.x ||
                    tetMax.y < mn.y || tetMin.y > mx.y ||
                    tetMax.z < mn.z || tetMin.z > mx.z)
                {
                    continue;
                }

                // 线段穿过 tet 或 tet 在线段半径内
                if (SegmentTet(segA, segB, p0, p1, p2, p3) ||
                    SphereTet(segA, radius, p0, p1, p2, p3) ||
                    SphereTet(segB, radius, p0, p1, p2, p3))
                {
                    result.Add(t);
                }
            }
            return result;
        }
    }
}
