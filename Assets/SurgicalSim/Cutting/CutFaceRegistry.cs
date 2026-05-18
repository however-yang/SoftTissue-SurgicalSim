using System.Collections.Generic;

namespace SurgicalSim.Cutting
{
    public enum SurfaceFaceClass
    {
        HiddenInternal = 0,
        OriginalSurface = 1,
        CutSurface = 2
    }

    public sealed class CutFaceRegistry
    {
        readonly HashSet<long> _cutFaceKeys = new HashSet<long>();
        readonly HashSet<long> _cutEdgeKeys = new HashSet<long>();
        public int CutFaceCount => _cutFaceKeys.Count;
        public int CutEdgeCount => _cutEdgeKeys.Count;

        public void Clear()
        {
            _cutFaceKeys.Clear();
            _cutEdgeKeys.Clear();
        }

        public void AddCutFace(int a, int b, int c, int strokeId = -1, int patchId = -1)
        {
            long key = SurfaceReconstructor.FaceKey(a, b, c);
            AddCutFaceKey(key, strokeId, patchId);
            AddCutEdge(a, b);
            AddCutEdge(b, c);
            AddCutEdge(c, a);
        }

        public void AddCutFaceKey(long key, int strokeId = -1, int patchId = -1)
        {
            _cutFaceKeys.Add(key);
        }

        public void AddCutEdge(int a, int b)
        {
            AddCutEdgeKey(SurfaceReconstructor.EdgeKey(a, b));
        }

        public void AddCutEdgeKey(long key)
        {
            _cutEdgeKeys.Add(key);
        }

        public bool IsCutFace(long key)
        {
            return _cutFaceKeys.Contains(key);
        }

        public bool IsCutEdge(long key)
        {
            return _cutEdgeKeys.Contains(key);
        }

        public SurfaceFaceClass ClassifyFace(long key)
        {
            return _cutFaceKeys.Contains(key)
                ? SurfaceFaceClass.CutSurface
                : SurfaceFaceClass.HiddenInternal;
        }
    }
}
