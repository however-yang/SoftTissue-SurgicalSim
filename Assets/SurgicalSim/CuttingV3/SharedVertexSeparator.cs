using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Cutting;

namespace SurgicalSim.CuttingV3
{
    /// <summary>
    /// Splits residual point-only connections left by tetrahedral subdivision.
    ///
    /// The old separator used a positive/negative side tag and then guessed
    /// where uncut neighbour tets belonged by centroid distance. That is not
    /// strong enough for continuous cutting: after several segments, the
    /// one-ring around a vertex can contain more than two material components,
    /// and a point-only bridge keeps XPBD constraints coupled.
    ///
    /// This implementation follows the shared-vertex criterion directly.
    /// Around a touched vertex v, active owner tets remain in the same local
    /// component when their shared face/edge is not a registered cut barrier.
    /// This keeps edge-adjacent numerical slivers attached while still cutting
    /// true edge-only bridges along the explicit cut surface.
    /// If the owner set has multiple such connected components, all but the
    /// largest component are rerouted to clones of v.
    /// </summary>
    internal sealed class SharedVertexSeparator
    {
        readonly TetMeshData _data;
        readonly Dictionary<int, List<int>> _vertexToTets;
        readonly HashSet<long> _cutFaceKeys;
        readonly HashSet<long> _cutEdgeKeys;
        readonly HashSet<int> _cutSurfaceVerts;

        readonly HashSet<int> _touched = new HashSet<int>(256);
        readonly Dictionary<int, int> _childSide = new Dictionary<int, int>(4096);

        readonly List<int> _owners = new List<int>(64);
        readonly List<List<int>> _components = new List<List<int>>(8);
        readonly List<int> _queue = new List<int>(64);
        bool _hasFramePlane;
        Vector3 _framePlaneNormal;
        float _framePlaneD;

        public int TouchedVertexCount { get; private set; }
        public int SeparatedVertexCount { get; private set; }
        public int RemainingSharedVertexCount { get; private set; }
        public int ChildTetCount { get; private set; }

        public int StrokeChildSideEntries => _childSide.Count;
        public int StrokeSeparatedVertexCount { get; private set; }
        public int StrokeCrossFramePivotHits { get; private set; }
        public int StrokeZombieChildSkipped { get; private set; }

        public SharedVertexSeparator(
            TetMeshData data,
            Dictionary<int, List<int>> vertexToTets,
            HashSet<long> cutFaceKeys,
            HashSet<long> cutEdgeKeys,
            HashSet<int> cutSurfaceVerts = null)
        {
            _data = data;
            _vertexToTets = vertexToTets;
            _cutFaceKeys = cutFaceKeys;
            _cutEdgeKeys = cutEdgeKeys;
            _cutSurfaceVerts = cutSurfaceVerts;
        }

        public void ResetStroke()
        {
            _touched.Clear();
            _childSide.Clear();
            TouchedVertexCount = 0;
            SeparatedVertexCount = 0;
            RemainingSharedVertexCount = 0;
            ChildTetCount = 0;
            StrokeSeparatedVertexCount = 0;
            StrokeCrossFramePivotHits = 0;
            StrokeZombieChildSkipped = 0;
        }

        public void BeginFrame()
        {
            _touched.Clear();
            TouchedVertexCount = 0;
            SeparatedVertexCount = 0;
            RemainingSharedVertexCount = 0;
            ChildTetCount = 0;
            _hasFramePlane = false;
            _framePlaneNormal = Vector3.zero;
            _framePlaneD = 0f;
        }

        public void SetFramePlane(Vector3 normal, float d)
        {
            if (normal.sqrMagnitude < 1e-10f)
            {
                _hasFramePlane = false;
                return;
            }

            _framePlaneNormal = normal.normalized;
            _framePlaneD = d;
            _hasFramePlane = true;
        }

        public void NotifyTetCut(int parentTet)
        {
            if (_data == null) return;
            if (parentTet < 0 || parentTet >= _data.NumTets) return;
            int b = 4 * parentTet;
            _touched.Add(_data.TetIds[b + 0]);
            _touched.Add(_data.TetIds[b + 1]);
            _touched.Add(_data.TetIds[b + 2]);
            _touched.Add(_data.TetIds[b + 3]);
        }

        public void NotifyChildSide(int childTet, int side)
        {
            if (childTet < 0) return;
            if (side == 0) return;
            _childSide[childTet] = side > 0 ? +1 : -1;
            ChildTetCount++;

            if (_data == null || childTet >= _data.NumTets) return;

            int b = 4 * childTet;
            _touched.Add(_data.TetIds[b + 0]);
            _touched.Add(_data.TetIds[b + 1]);
            _touched.Add(_data.TetIds[b + 2]);
            _touched.Add(_data.TetIds[b + 3]);
        }

        public void SeparateAlongCut(
            System.Func<int, int> cloneVertex,
            System.Action<int, int, int> replaceVertexInTet)
        {
            TouchedVertexCount = _touched.Count;
            if (TouchedVertexCount == 0) return;
            if (_data == null || cloneVertex == null || replaceVertexInTet == null) return;

            foreach (int v in _touched)
            {
                if (v < 0) continue;
                if (!_vertexToTets.TryGetValue(v, out var ownerRefs) || ownerRefs.Count <= 1)
                    continue;

                SnapshotActiveOwners(v, ownerRefs);
                if (_owners.Count <= 1) continue;

                BuildComponents(v);
                if (_components.Count <= 1)
                {
                    RemainingSharedVertexCount++;
                    continue;
                }

                int keep = LargestComponentIndex();
                for (int ci = 0; ci < _components.Count; ci++)
                {
                    if (ci == keep) continue;

                    int vClone = cloneVertex(v);
                    if (vClone < 0) continue;

                    var comp = _components[ci];
                    for (int i = 0; i < comp.Count; i++)
                        replaceVertexInTet(comp[i], v, vClone);

                    SeparatedVertexCount++;
                    StrokeSeparatedVertexCount++;
                }
            }

            _touched.Clear();
        }

        void SnapshotActiveOwners(int v, List<int> ownerRefs)
        {
            _owners.Clear();
            for (int i = 0; i < ownerRefs.Count; i++)
            {
                int t = ownerRefs[i];
                if (t < 0 || t >= _data.NumTets) continue;
                if (!_data.TetActive[t])
                {
                    if (_childSide.ContainsKey(t)) StrokeZombieChildSkipped++;
                    continue;
                }
                if (!TetContainsVertex(t, v)) continue;
                _owners.Add(t);
            }
        }

        void BuildComponents(int pivot)
        {
            ClearComponents();
            int n = _owners.Count;
            int[] compOf = new int[n];
            for (int i = 0; i < n; i++) compOf[i] = -1;

            for (int start = 0; start < n; start++)
            {
                if (compOf[start] >= 0) continue;

                int compIndex = _components.Count;
                var comp = GetComponentList(compIndex);
                comp.Clear();
                int compSide = OwnerSide(_owners[start]);

                _queue.Clear();
                _queue.Add(start);
                compOf[start] = compIndex;

                for (int qi = 0; qi < _queue.Count; qi++)
                {
                    int ownerIndex = _queue[qi];
                    int ownerTet = _owners[ownerIndex];
                    comp.Add(ownerTet);

                    for (int j = 0; j < n; j++)
                    {
                        if (compOf[j] >= 0) continue;
                        if (!CanShareMaterialConnection(ownerTet, _owners[j], pivot)) continue;

                        int candidateSide = OwnerSide(_owners[j]);
                        if (compSide != 0 && candidateSide != 0 && compSide != candidateSide)
                            continue;

                        compOf[j] = compIndex;
                        if (compSide == 0 && candidateSide != 0)
                            compSide = candidateSide;
                        _queue.Add(j);
                    }
                }
            }
        }

        int OwnerSide(int tet)
        {
            if (_childSide.TryGetValue(tet, out int side))
                return side;
            return 0;
        }

        void ClearComponents()
        {
            for (int i = 0; i < _components.Count; i++) _components[i].Clear();
            _components.Clear();
        }

        List<int> GetComponentList(int index)
        {
            while (_components.Count <= index) _components.Add(new List<int>(16));
            return _components[index];
        }

        int LargestComponentIndex()
        {
            int best = 0;
            int bestCount = _components[0].Count;
            for (int i = 1; i < _components.Count; i++)
            {
                if (_components[i].Count <= bestCount) continue;
                best = i;
                bestCount = _components[i].Count;
            }
            return best;
        }

        bool TetContainsVertex(int t, int v)
        {
            int b = 4 * t;
            return _data.TetIds[b + 0] == v ||
                   _data.TetIds[b + 1] == v ||
                   _data.TetIds[b + 2] == v ||
                   _data.TetIds[b + 3] == v;
        }

        bool CanShareMaterialConnection(int ta, int tb, int pivot)
        {
            int ba = 4 * ta;
            int bb = 4 * tb;
            int s0 = -1;
            int s1 = -1;
            int s2 = -1;
            int sharedCount = 0;

            for (int ia = 0; ia < 4; ia++)
            {
                int va = _data.TetIds[ba + ia];
                for (int ib = 0; ib < 4; ib++)
                {
                    if (va != _data.TetIds[bb + ib]) continue;
                    if (sharedCount == 0) s0 = va;
                    else if (sharedCount == 1) s1 = va;
                    else if (sharedCount == 2) s2 = va;
                    sharedCount++;
                    break;
                }
            }

            if (sharedCount >= 3)
            {
                long faceKey = SurfaceReconstructor.FaceKey(s0, s1, s2);
                if (_cutFaceKeys != null && _cutFaceKeys.Contains(faceKey))
                    return false;
                if (AllCutSurfaceVertices(s0, s1, s2) && ShouldSeparateAcrossCut(ta, tb))
                    return false;
                return true;
            }

            if (sharedCount == 2)
            {
                if (s0 != pivot && s1 != pivot)
                    return true;

                long edgeKey = SurfaceReconstructor.EdgeKey(s0, s1);
                if (_cutEdgeKeys != null && _cutEdgeKeys.Contains(edgeKey))
                    return false;
                if (BothCutSurfaceVertices(s0, s1) && ShouldSeparateAcrossCut(ta, tb))
                    return false;
                return true;
            }

            return false;
        }

        bool BothCutSurfaceVertices(int a, int b)
        {
            return _cutSurfaceVerts != null &&
                   _cutSurfaceVerts.Contains(a) &&
                   _cutSurfaceVerts.Contains(b);
        }

        bool AllCutSurfaceVertices(int a, int b, int c)
        {
            return _cutSurfaceVerts != null &&
                   _cutSurfaceVerts.Contains(a) &&
                   _cutSurfaceVerts.Contains(b) &&
                   _cutSurfaceVerts.Contains(c);
        }

        bool ShouldSeparateAcrossCut(int ta, int tb)
        {
            int sideA = OwnerSide(ta);
            int sideB = OwnerSide(tb);
            if (sideA != 0 && sideB != 0)
                return sideA != sideB;

            if (!_hasFramePlane) return false;

            float da = Vector3.Dot(_framePlaneNormal, TetCenter(ta)) + _framePlaneD;
            float db = Vector3.Dot(_framePlaneNormal, TetCenter(tb)) + _framePlaneD;
            const float eps = 1e-5f;
            if (da > eps && db < -eps)
            {
                StrokeCrossFramePivotHits++;
                return true;
            }
            if (da < -eps && db > eps)
            {
                StrokeCrossFramePivotHits++;
                return true;
            }

            return false;
        }

        Vector3 TetCenter(int t)
        {
            int b = 4 * t;
            return 0.25f * (
                _data.Positions[_data.TetIds[b + 0]] +
                _data.Positions[_data.TetIds[b + 1]] +
                _data.Positions[_data.TetIds[b + 2]] +
                _data.Positions[_data.TetIds[b + 3]]);
        }
    }
}
