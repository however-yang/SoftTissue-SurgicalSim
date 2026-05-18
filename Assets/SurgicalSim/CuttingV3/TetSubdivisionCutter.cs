using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Cutting;
using SurgicalSim.Physics;

namespace SurgicalSim.CuttingV3
{
    public class TetSubdivisionCutter
    {
        const float EPS = 1e-8f;
        const float SNAP_DIST = 1e-4f;
        const float MIN_TET_VOLUME = 1e-12f;
        const float SWEPT_MARGIN = 0.015f;
        const float RIBBON_INTERSECTION_MARGIN = 0.006f;
        const float NEW_CUT_VERTEX_VELOCITY_DAMPING = 0.35f;
        const float SMALL_COMPONENT_VELOCITY_DAMPING = 0.18f;
        const float SMALL_COMPONENT_VOLUME_FRACTION = 0.015f;
        const int SMALL_COMPONENT_TET_THRESHOLD = 48;
        const float MAX_STABILIZED_VERTEX_SPEED = 0.35f;

        // Yu 2025's trajectory correction is meant to collapse only
        // near-vertex intersections. A 20% edge-length snap visibly drags a
        // flat cut surface onto unrelated mesh vertices, which is one of the
        // main sources of raised triangular spikes during straight cuts.
        const float SNAP_EDGE_FRAC = 0f;
        const float BOUNDARY_EDGE_SNAP_FRAC = 0f;
        const float GENERATED_EDGE_SNAP_FRAC = 0f;

        // A straight stroke should be one geometric plane. Keep that plane
        // locked while the measured sweep is still a near-linear continuation;
        // start a fresh patch on real turns so future S/angle cuts still have
        // a clean extension point instead of one global frozen plane.
        const float PATCH_MAX_NORMAL_ANGLE_DEG = 12f;
        const float PATCH_MAX_MOVE_ANGLE_DEG = 30f;
        const float PATCH_MAX_OFFSET = 0.02f;

        TetMeshData _data;
        XPBDSolverGPU _solver;
        bool _dirty;
        int _originalParticleCount;

        readonly Dictionary<long, (int posV, int negV)> _frameSplitCache = new Dictionary<long, (int posV, int negV)>();
        readonly Dictionary<(int patch, long edge), (int posV, int negV)> _strokeSplitCache =
            new Dictionary<(int patch, long edge), (int posV, int negV)>();
        readonly Dictionary<int, int> _frameSnapCache = new Dictionary<int, int>();
        readonly Dictionary<int, List<int>> _vertexToTets = new Dictionary<int, List<int>>();
        readonly HashSet<int> _alreadyCutTets = new HashSet<int>();
        readonly List<int> _candidateTets = new List<int>(2048);
        readonly List<int> _contactCandidates = new List<int>(1024);
        readonly Vector2[] _ribbonCutPolygon = new Vector2[6];
        readonly SpatialTetIndex _spatialIndex = new SpatialTetIndex();
        readonly List<CutSurfacePatch> _pendingCutSurfacePatches = new List<CutSurfacePatch>(256);
        readonly List<CutSurfacePatch> _splitCutSurfacePatches = new List<CutSurfacePatch>(4);
        readonly List<int> _splitCreatedTets = new List<int>(8);
        readonly List<int> _splitCutSurfaceVertexIds = new List<int>(8);
        readonly HashSet<long> _splitCutSurfaceFaceKeys = new HashSet<long>();
        readonly HashSet<long> _splitCutSurfaceEdgeKeys = new HashSet<long>();
        readonly HashSet<int> _cutSurfaceVertexIds = new HashSet<int>();
        readonly HashSet<long> _cutSurfaceFaceKeys = new HashSet<long>();
        readonly HashSet<long> _cutSurfaceEdgeKeys = new HashSet<long>();
        readonly CutFaceRegistry _cutFaceRegistry = new CutFaceRegistry();
        readonly HashSet<int> _originalSurfaceVertexIds = new HashSet<int>();
        readonly HashSet<long> _originalSurfaceFaceKeys = new HashSet<long>();
        readonly HashSet<long> _originalSurfaceEdgeKeys = new HashSet<long>();
        readonly List<int> _surfaceRootIds = new List<int>();
        readonly List<int> _surfaceSupport0 = new List<int>();
        readonly List<int> _surfaceSupport1 = new List<int>();
        readonly List<int> _surfaceSupport2 = new List<int>();
        readonly List<Vector3> _cutSurfaceNormals = new List<Vector3>(128);
        readonly List<int> _tetBirthStroke = new List<int>();
        readonly List<int> _tetBirthPatch = new List<int>();
        readonly HashSet<int> _stabilizeVertexIds = new HashSet<int>();
        readonly List<int> _componentQueue = new List<int>(256);
        readonly List<int> _componentTets = new List<int>(256);
        readonly HashSet<int> _componentVerts = new HashSet<int>();

        SharedVertexSeparator _separator;

        int _strokeSerial;
        int _strokeStartVertCount = -1;
        Vector3 _lastValidNormal;
        Vector3 _currentCutNormal;
        Vector3 _patchNormal;
        Vector3 _patchMoveDir;
        float _patchPlaneD;
        int _patchId;
        bool _hasPatchPlane;
        bool _indexDirty = true;
        float _initialActiveRestVolume = MIN_TET_VOLUME;
        int[] _componentVisited;
        int _componentVisitStamp;

        // Diagnostic logging: print the separator state every N cut
        // frames that produced a non-trivial result. Set to 0 to
        // disable. A value around 30 keeps the console readable while
        // still showing the trend during a single stroke.
        const int LOG_EVERY_N_CUT_FRAMES = 30;
        int _logTicker;
        int _prevCrossFrameHits;

        // Step 4: per-SplitTet child counters incremented by AT(). When
        // every child of a single parent tet is rejected (typically
        // because all of them are slivers below MIN_TET_VOLUME), SplitTet
        // rolls back the parent's deactivation so the cut surface does
        // not develop empty holes.
        int _splitAttempts;
        int _splitSuccesses;
        int _currentSplitTet = -1;
        // Stroke-level diagnostic for the HUD / console.
        public int StrokeSplitRollbackCount { get; private set; }

        public float LastMoveDistance { get; private set; }
        public int LastCandidateTetCount { get; private set; }
        public int LastIntersectedTetCount { get; private set; }
        public string LastRejectReason { get; private set; } = "idle";
        public int LastSeparatedVertexCount { get; private set; }
        public int LastRemainingSharedVertexCount { get; private set; }
        // Stroke-level diagnostic counters exposed for the HUD / logs.
        public int StrokeSeparatedVertexCount => _separator?.StrokeSeparatedVertexCount ?? 0;
        public int StrokeCrossFramePivotHits => _separator?.StrokeCrossFramePivotHits ?? 0;
        public int StrokeChildSideEntries => _separator?.StrokeChildSideEntries ?? 0;
        public int StrokeZombieChildSkipped => _separator?.StrokeZombieChildSkipped ?? 0;
        public float LastQueryMs { get; private set; }
        public float LastSplitMs { get; private set; }
        public float LastSeparateMs { get; private set; }

        public int TotalNewVerts { get; private set; }
        public int TotalNewTets { get; private set; }
        public int TotalCutTets { get; private set; }
        public bool ProtectStrokeBornTets { get; set; } = true;
        public bool IsDirty => _dirty;
        public HashSet<int> CutSurfaceVertexIds => _cutSurfaceVertexIds;
        public HashSet<long> CutSurfaceFaceKeys => _cutSurfaceFaceKeys;
        public HashSet<long> CutSurfaceEdgeKeys => _cutSurfaceEdgeKeys;
        public CutFaceRegistry CutFaceRegistry => _cutFaceRegistry;
        public HashSet<int> OriginalSurfaceVertexIds => _originalSurfaceVertexIds;
        public HashSet<long> OriginalSurfaceFaceKeys => _originalSurfaceFaceKeys;
        public HashSet<long> OriginalSurfaceEdgeKeys => _originalSurfaceEdgeKeys;
        public List<int> SurfaceRootIds => _surfaceRootIds;
        public List<int> SurfaceSupport0 => _surfaceSupport0;
        public List<int> SurfaceSupport1 => _surfaceSupport1;
        public List<int> SurfaceSupport2 => _surfaceSupport2;
        public int OriginalParticleCount => _originalParticleCount;
        public List<Vector3> CutSurfaceNormals => _cutSurfaceNormals;
        public int PendingCutSurfacePatchCount => _pendingCutSurfacePatches.Count;

        public struct CutSurfacePatch
        {
            public int a;
            public int b;
            public int c;
            public int d;
            public int count;
            public Vector3 normal;
            public bool reverseWinding;

            public int IdAt(int index)
            {
                switch (index)
                {
                    case 0: return a;
                    case 1: return b;
                    case 2: return c;
                    case 3: return d;
                    default: return -1;
                }
            }
        }

        public void DrainCutSurfacePatches(List<CutSurfacePatch> output)
        {
            if (output == null) return;
            output.AddRange(_pendingCutSurfacePatches);
            _pendingCutSurfacePatches.Clear();
        }

        public void Init(TetMeshData data, XPBDSolverGPU solver)
        {
            _data = data;
            _solver = solver;
            _dirty = false;
            _originalParticleCount = _data != null ? _data.NumParticles : 0;
            TotalNewVerts = 0;
            TotalNewTets = 0;
            TotalCutTets = 0;
            _strokeStartVertCount = -1;
            _lastValidNormal = Vector3.zero;
            _currentCutNormal = Vector3.zero;
            _pendingCutSurfacePatches.Clear();
            _splitCutSurfacePatches.Clear();
            _splitCreatedTets.Clear();
            _splitCutSurfaceVertexIds.Clear();
            _splitCutSurfaceFaceKeys.Clear();
            _splitCutSurfaceEdgeKeys.Clear();
            _stabilizeVertexIds.Clear();
            ComputeInitialVolumeStats();
            _cutSurfaceVertexIds.Clear();
            _cutSurfaceFaceKeys.Clear();
            _cutSurfaceEdgeKeys.Clear();
            _cutFaceRegistry.Clear();
            BuildOriginalSurfaceVertexSet();
            _cutSurfaceNormals.Clear();
            _tetBirthStroke.Clear();
            _tetBirthPatch.Clear();
            EnsureTetMetadataSlots(_data != null ? _data.NumTets : 0);
            RebuildVertexAdjacency();
            _separator = _data != null
                ? new SharedVertexSeparator(_data, _vertexToTets, _cutSurfaceFaceKeys, _cutSurfaceEdgeKeys, _cutSurfaceVertexIds)
                : null;
            RebuildSpatialIndex(_data != null ? _data.NumTets : 0);
        }

        void ComputeInitialVolumeStats()
        {
            _initialActiveRestVolume = MIN_TET_VOLUME;
            if (_data == null) return;

            float sum = 0f;
            for (int t = 0; t < _data.NumTets; t++)
            {
                if (!_data.TetActive[t]) continue;
                float vol = 0f;
                if (_data.RestVolumes != null && t < _data.RestVolumes.Length)
                    vol = _data.RestVolumes[t];
                if (vol <= 0f)
                {
                    int b = 4 * t;
                    vol = Mathf.Abs(TetVolume(
                        _data.RestPositions[_data.TetIds[b + 0]],
                        _data.RestPositions[_data.TetIds[b + 1]],
                        _data.RestPositions[_data.TetIds[b + 2]],
                        _data.RestPositions[_data.TetIds[b + 3]]));
                }
                sum += Mathf.Max(0f, vol);
            }

            _initialActiveRestVolume = Mathf.Max(MIN_TET_VOLUME, sum);
        }

        void BuildOriginalSurfaceVertexSet()
        {
            _originalSurfaceVertexIds.Clear();
            _originalSurfaceFaceKeys.Clear();
            _originalSurfaceEdgeKeys.Clear();
            _surfaceRootIds.Clear();
            _surfaceSupport0.Clear();
            _surfaceSupport1.Clear();
            _surfaceSupport2.Clear();
            if (_data == null) return;

            for (int v = 0; v < _data.NumParticles; v++)
            {
                _surfaceRootIds.Add(-1);
                _surfaceSupport0.Add(-1);
                _surfaceSupport1.Add(-1);
                _surfaceSupport2.Add(-1);
            }

            if (_data.SurfaceTriIds == null) return;

            for (int i = 0; i < _data.SurfaceTriIds.Length; i++)
            {
                int v = _data.SurfaceTriIds[i];
                if (v >= 0 && v < _originalParticleCount)
                    _originalSurfaceVertexIds.Add(v);
            }

            foreach (int v in _originalSurfaceVertexIds)
                SetSurfaceSupport(v, v);

            for (int i = 0; i + 2 < _data.SurfaceTriIds.Length; i += 3)
            {
                int a = _data.SurfaceTriIds[i];
                int b = _data.SurfaceTriIds[i + 1];
                int c = _data.SurfaceTriIds[i + 2];
                if (a < 0 || b < 0 || c < 0) continue;
                if (a >= _originalParticleCount ||
                    b >= _originalParticleCount ||
                    c >= _originalParticleCount) continue;
                _originalSurfaceFaceKeys.Add(SurfaceReconstructor.FaceKey(a, b, c));
                _originalSurfaceEdgeKeys.Add(SurfaceReconstructor.EdgeKey(a, b));
                _originalSurfaceEdgeKeys.Add(SurfaceReconstructor.EdgeKey(b, c));
                _originalSurfaceEdgeKeys.Add(SurfaceReconstructor.EdgeKey(c, a));
            }
        }

        public void ResetStroke()
        {
            _strokeSerial++;
            _alreadyCutTets.Clear();
            _strokeSplitCache.Clear();
            _frameSplitCache.Clear();
            _frameSnapCache.Clear();
            _strokeStartVertCount = _data != null ? _data.NumParticles : -1;
            _lastValidNormal = Vector3.zero;
            _currentCutNormal = Vector3.zero;
            _patchNormal = Vector3.zero;
            _patchMoveDir = Vector3.zero;
            _patchPlaneD = 0f;
            _patchId = 0;
            _hasPatchPlane = false;
            _pendingCutSurfacePatches.Clear();
            _splitCutSurfacePatches.Clear();
            _splitCreatedTets.Clear();
            _splitCutSurfaceVertexIds.Clear();
            _splitCutSurfaceFaceKeys.Clear();
            _splitCutSurfaceEdgeKeys.Clear();
            RebuildSpatialIndex(_data != null ? _data.NumTets : 0);
            // CRITICAL: the separator's _childSide table is stroke
            // scoped so the lotus-root detection sees side info from
            // every frame of THIS stroke. Without this reset the table
            // would leak across strokes and grow without bound.
            _separator?.ResetStroke();
            _logTicker = 0;
            _prevCrossFrameHits = 0;
            StrokeSplitRollbackCount = 0;
        }

        public struct CutResult
        {
            public int newVerts;
            public int newTets;
            public int cutTets;
            public float elapsedMs;
        }

        public bool IsBladeNearMesh(Vector3 bladeA, Vector3 bladeB, float radius)
        {
            if (_data == null || _data.NumTets == 0) return false;

            if (_indexDirty) RebuildSpatialIndex(_data.NumTets);

            Vector3 lo = Vector3.Min(bladeA, bladeB) - Vector3.one * radius;
            Vector3 hi = Vector3.Max(bladeA, bladeB) + Vector3.one * radius;

            _spatialIndex.QueryAabb(lo, hi, _data.NumTets, _contactCandidates);
            LastCandidateTetCount = _contactCandidates.Count;

            float r2 = radius * radius;
            for (int i = 0; i < _contactCandidates.Count; i++)
            {
                int t = _contactCandidates[i];
                if (t < 0 || t >= _data.NumTets || !_data.TetActive[t]) continue;

                int i0 = _data.TetIds[4 * t + 0];
                int i1 = _data.TetIds[4 * t + 1];
                int i2 = _data.TetIds[4 * t + 2];
                int i3 = _data.TetIds[4 * t + 3];

                if (PointSegmentDistanceSq(_data.Positions[i0], bladeA, bladeB) <= r2) return true;
                if (PointSegmentDistanceSq(_data.Positions[i1], bladeA, bladeB) <= r2) return true;
                if (PointSegmentDistanceSq(_data.Positions[i2], bladeA, bladeB) <= r2) return true;
                if (PointSegmentDistanceSq(_data.Positions[i3], bladeA, bladeB) <= r2) return true;

                if (SegmentAabbDistanceSq(bladeA, bladeB, t) <= r2) return true;
            }

            return false;
        }

        public CutResult Cut(Vector3 A0, Vector3 B0, Vector3 A1, Vector3 B1)
        {
            CutResult result = new CutResult();
            if (_data == null) return result;

            var swTotal = System.Diagnostics.Stopwatch.StartNew();
            var sw = System.Diagnostics.Stopwatch.StartNew();
            _frameSplitCache.Clear();
            _frameSnapCache.Clear();
            _separator?.BeginFrame();
            LastSeparatedVertexCount = 0;
            LastRemainingSharedVertexCount = 0;
            LastQueryMs = 0f;
            LastSplitMs = 0f;
            LastSeparateMs = 0f;

            Vector3 bladeDir = B1 - A1;
            float bladeLen = bladeDir.magnitude;
            Vector3 move = 0.5f * ((A1 + B1) - (A0 + B0));
            LastMoveDistance = move.magnitude;
            Vector3 moveDir = LastMoveDistance > 1e-6f ? move / LastMoveDistance : Vector3.zero;

            if (bladeLen < 1e-6f)
            {
                LastRejectReason = "blade_too_short";
                result.elapsedMs = (float)swTotal.Elapsed.TotalMilliseconds;
                return result;
            }

            bladeDir /= bladeLen;
            Vector3 planeNormal = Vector3.Cross(bladeDir, move);
            if (planeNormal.sqrMagnitude < 1e-10f)
            {
                planeNormal = _lastValidNormal.sqrMagnitude > 1e-10f
                    ? _lastValidNormal
                    : StableNormal(bladeDir);
            }
            else
            {
                planeNormal.Normalize();
            }

            if (_lastValidNormal.sqrMagnitude > 1e-10f)
            {
                // Keep plane orientation consistent across frames: a cross
                // product sign flip would otherwise force the stroke cache
                // to hand out fresh EP pairs and produce duplicate "scale
                // flake" geometry on the cut surface.
                if (Vector3.Dot(planeNormal, _lastValidNormal) < 0f) planeNormal = -planeNormal;
            }

            Vector3 planePoint = 0.25f * (A0 + B0 + A1 + B1);
            float planeD = -Vector3.Dot(planeNormal, planePoint);
            StabilizePatchPlane(moveDir, ref planeNormal, ref planeD);
            _currentCutNormal = planeNormal;
            _lastValidNormal = planeNormal;
            _separator?.SetFramePlane(planeNormal, planeD);

            Vector3 lo = Vector3.Min(Vector3.Min(A0, B0), Vector3.Min(A1, B1)) - Vector3.one * SWEPT_MARGIN;
            Vector3 hi = Vector3.Max(Vector3.Max(A0, B0), Vector3.Max(A1, B1)) + Vector3.one * SWEPT_MARGIN;

            if (_indexDirty) RebuildSpatialIndex(_data.NumTets);
            int segmentTetLimit = _data.NumTets;
            _spatialIndex.QueryAabb(lo, hi, segmentTetLimit, _candidateTets);
            LastQueryMs = (float)sw.Elapsed.TotalMilliseconds;
            LastCandidateTetCount = _candidateTets.Count;

            if (_candidateTets.Count == 0)
            {
                LastRejectReason = _spatialIndex.OverlapsBounds(lo, hi) ? "no_candidate" : "no_contact";
                result.elapsedMs = (float)swTotal.Elapsed.TotalMilliseconds;
                return result;
            }

            sw.Restart();
            int newVerts0 = _data.NumParticles;
            int newTets0 = _data.NumTets;
            int cutCount = 0;
            int intersected = 0;

            for (int ci = 0; ci < _candidateTets.Count; ci++)
            {
                int t = _candidateTets[ci];
                if (t < 0 || t >= _data.NumTets) continue;
                if (t >= segmentTetLimit) continue;
                if (!_data.TetActive[t]) continue;
                if (_alreadyCutTets.Contains(t)) continue;
                if (WasBornInCurrentStroke(t)) continue;
                if (!TetAabbOverlap(t, lo, hi)) continue;

                int i0 = _data.TetIds[4 * t + 0];
                int i1 = _data.TetIds[4 * t + 1];
                int i2 = _data.TetIds[4 * t + 2];
                int i3 = _data.TetIds[4 * t + 3];

                float d0 = Vector3.Dot(planeNormal, _data.Positions[i0]) + planeD;
                float d1 = Vector3.Dot(planeNormal, _data.Positions[i1]) + planeD;
                float d2 = Vector3.Dot(planeNormal, _data.Positions[i2]) + planeD;
                float d3 = Vector3.Dot(planeNormal, _data.Positions[i3]) + planeD;

                EnforceConsistentSide(i0, ref d0);
                EnforceConsistentSide(i1, ref d1);
                EnforceConsistentSide(i2, ref d2);
                EnforceConsistentSide(i3, ref d3);

                float rawD0 = d0;
                float rawD1 = d1;
                float rawD2 = d2;
                float rawD3 = d3;

                d0 = SnapSignedDistance(d0);
                d1 = SnapSignedDistance(d1);
                d2 = SnapSignedDistance(d2);
                d3 = SnapSignedDistance(d3);

                int pos = CountPositive(d0, d1, d2, d3);
                if (pos == 0 || pos == 4)
                {
                    continue;
                }

                if (!TetPlaneIntersectsRibbon(t, rawD0, rawD1, rawD2, rawD3,
                    planePoint, bladeDir, moveDir, bladeLen, LastMoveDistance))
                {
                    continue;
                }

                intersected++;
                if (SplitTet(t, d0, d1, d2, d3))
                {
                    cutCount++;
                }
            }

            LastSplitMs = (float)sw.Elapsed.TotalMilliseconds;
            LastIntersectedTetCount = intersected;

            // Stage E (CuttingV3.1 Step 1): shared-vertex separation. Walks the
            // local adjacency around every vertex touched by a cut tet and
            // duplicates pivots whose remaining neighbourhood is disconnected,
            // eliminating the residual coupling between the two sides of the
            // cut path (the "藕断丝连" failure mode).
            sw.Restart();
            if (_separator != null && intersected > 0)
            {
                _separator.SeparateAlongCut(CloneVertex, ReplaceVertexInTet);
                LastSeparatedVertexCount = _separator.SeparatedVertexCount;
                LastRemainingSharedVertexCount = _separator.RemainingSharedVertexCount;

                // Low-frequency diagnostic. Fires only when the stroke
                // actually produced separations OR a fresh cross-frame
                // bridge pivot was detected, so it stays quiet during
                // idle frames but tells us immediately how often the
                // stroke-scoped _childSide is paying off.
                if (LastSeparatedVertexCount > 0 ||
                    _separator.StrokeCrossFramePivotHits != _prevCrossFrameHits)
                {
                    _logTicker++;
                    if (_logTicker >= LOG_EVERY_N_CUT_FRAMES)
                    {
                        _logTicker = 0;
                        UnityEngine.Debug.Log(
                            $"[SepDiag] sep={LastSeparatedVertexCount} " +
                            $"rem={LastRemainingSharedVertexCount} " +
                            $"strokeSep={_separator.StrokeSeparatedVertexCount} " +
                            $"crossFrame={_separator.StrokeCrossFramePivotHits} " +
                            $"childSide={_separator.StrokeChildSideEntries} " +
                            $"zombie={_separator.StrokeZombieChildSkipped} " +
                            $"rollback={StrokeSplitRollbackCount} " +
                            $"P/T={_data.NumParticles}/{_data.NumTets}");
                    }
                    _prevCrossFrameHits = _separator.StrokeCrossFramePivotHits;
                }
            }
            LastSeparateMs = (float)sw.Elapsed.TotalMilliseconds;

            result.newVerts = _data.NumParticles - newVerts0;
            result.newTets = _data.NumTets - newTets0;
            result.cutTets = cutCount;
            result.elapsedMs = (float)swTotal.Elapsed.TotalMilliseconds;

            if (cutCount > 0)
            {
                _dirty = true;
                // Keep the stroke index stable. Rebuilding it after every split
                // dominates frame time; inactive source tets are filtered at query time.
                TotalNewVerts += result.newVerts;
                TotalNewTets += result.newTets;
                TotalCutTets += result.cutTets;
                LastRejectReason = "cut";
            }
            else
            {
                LastRejectReason = intersected == 0 ? "no_intersection" : "degenerate";
            }

            return result;
        }

        public void FlushToGPU()
        {
            if (_solver == null || _data == null || !_dirty) return;

            _solver.ReadbackAll(_data);
            StabilizeCutRegionForGpu();
            _solver.Dispose();
            _solver.Init(_data);
            RebuildSpatialIndex(_data.NumTets);
            _dirty = false;
        }

        void StabilizeCutRegionForGpu()
        {
            if (_data == null || _stabilizeVertexIds.Count == 0) return;

            float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);
            DampenVertices(_stabilizeVertexIds, NEW_CUT_VERTEX_VELOCITY_DAMPING, dt);
            StabilizeSmallCutComponents(dt);
            _stabilizeVertexIds.Clear();
        }

        void StabilizeSmallCutComponents(float dt)
        {
            if (_vertexToTets.Count == 0) return;
            EnsureComponentVisited();

            if (_componentVisitStamp == int.MaxValue)
            {
                System.Array.Clear(_componentVisited, 0, _componentVisited.Length);
                _componentVisitStamp = 0;
            }
            _componentVisitStamp++;
            int stamp = _componentVisitStamp;
            float smallVolumeLimit = Mathf.Max(MIN_TET_VOLUME, _initialActiveRestVolume * SMALL_COMPONENT_VOLUME_FRACTION);

            foreach (int seedVertex in _stabilizeVertexIds)
            {
                if (!_vertexToTets.TryGetValue(seedVertex, out var seedTets)) continue;
                for (int i = 0; i < seedTets.Count; i++)
                {
                    int seedTet = seedTets[i];
                    if (seedTet < 0 || seedTet >= _data.NumTets) continue;
                    if (!_data.TetActive[seedTet]) continue;
                    if (_componentVisited[seedTet] == stamp) continue;

                    float restVolume = CollectActiveTetComponent(seedTet, stamp);
                    if (_componentTets.Count == 0) continue;

                    bool tinyByCount = _componentTets.Count <= SMALL_COMPONENT_TET_THRESHOLD;
                    bool tinyByVolume = restVolume <= smallVolumeLimit;
                    if (!tinyByCount && !tinyByVolume) continue;

                    DampenVertices(_componentVerts, SMALL_COMPONENT_VELOCITY_DAMPING, dt);
                }
            }
        }

        float CollectActiveTetComponent(int startTet, int stamp)
        {
            _componentQueue.Clear();
            _componentTets.Clear();
            _componentVerts.Clear();

            _componentVisited[startTet] = stamp;
            _componentQueue.Add(startTet);
            float restVolume = 0f;

            for (int qi = 0; qi < _componentQueue.Count; qi++)
            {
                int t = _componentQueue[qi];
                _componentTets.Add(t);
                restVolume += ActiveRestVolume(t);

                int b = 4 * t;
                for (int k = 0; k < 4; k++)
                {
                    int v = _data.TetIds[b + k];
                    _componentVerts.Add(v);
                    if (!_vertexToTets.TryGetValue(v, out var incident)) continue;

                    for (int j = 0; j < incident.Count; j++)
                    {
                        int next = incident[j];
                        if (next < 0 || next >= _data.NumTets) continue;
                        if (!_data.TetActive[next]) continue;
                        if (_componentVisited[next] == stamp) continue;

                        _componentVisited[next] = stamp;
                        _componentQueue.Add(next);
                    }
                }
            }

            return restVolume;
        }

        float ActiveRestVolume(int t)
        {
            if (_data.RestVolumes != null && t >= 0 && t < _data.RestVolumes.Length)
                return Mathf.Max(0f, _data.RestVolumes[t]);

            int b = 4 * t;
            return Mathf.Abs(TetVolume(
                _data.RestPositions[_data.TetIds[b + 0]],
                _data.RestPositions[_data.TetIds[b + 1]],
                _data.RestPositions[_data.TetIds[b + 2]],
                _data.RestPositions[_data.TetIds[b + 3]]));
        }

        void EnsureComponentVisited()
        {
            if (_componentVisited == null || _componentVisited.Length < _data.NumTets)
                _componentVisited = new int[_data.NumTets];
        }

        void DampenVertices(IEnumerable<int> vertexIds, float damping, float dt)
        {
            damping = Mathf.Clamp01(damping);
            foreach (int v in vertexIds)
            {
                if (v < 0 || v >= _data.NumParticles) continue;
                if (_data.InvMass[v] == 0f) continue;

                Vector3 vel = _data.Velocities[v];
                if (!IsFinite(vel))
                    vel = Vector3.zero;
                else
                    vel *= damping;

                float speed = vel.magnitude;
                if (speed > MAX_STABILIZED_VERTEX_SPEED)
                    vel *= MAX_STABILIZED_VERTEX_SPEED / speed;

                _data.Velocities[v] = vel;
                _data.PrevPositions[v] = _data.Positions[v] - vel * dt;
            }
        }

        static bool IsFinite(Vector3 v)
        {
            return !(float.IsNaN(v.x) || float.IsNaN(v.y) || float.IsNaN(v.z) ||
                     float.IsInfinity(v.x) || float.IsInfinity(v.y) || float.IsInfinity(v.z));
        }

        bool SplitTet(int t, float d0, float d1, float d2, float d3)
        {
            int[] ids =
            {
                _data.TetIds[4 * t + 0],
                _data.TetIds[4 * t + 1],
                _data.TetIds[4 * t + 2],
                _data.TetIds[4 * t + 3]
            };
            float[] sd = { d0, d1, d2, d3 };

            for (int i = 0; i < 4; i++) sd[i] = SnapSignedDistance(sd[i]);
            int pos = CountPositive(sd[0], sd[1], sd[2], sd[3]);
            if (pos == 0 || pos == 4) return false;

            _separator?.NotifyTetCut(t);

            // Deactivate the parent. If the case-split below produces
            // zero valid children (all rejected as sub-MIN_TET_VOLUME
            // slivers) we must put the parent back so the cut surface
            // does not develop an empty hole at this tet's footprint.
            _data.TetActive[t] = false;
            RemoveTetFromAdjacency(t, ids);
            _alreadyCutTets.Add(t);

            _splitAttempts = 0;
            _splitSuccesses = 0;
            _splitCutSurfacePatches.Clear();
            _splitCreatedTets.Clear();
            _splitCutSurfaceVertexIds.Clear();
            _splitCutSurfaceFaceKeys.Clear();
            _splitCutSurfaceEdgeKeys.Clear();

            System.Array.Sort(ids, sd);
            _currentSplitTet = t;
            if (pos == 1 || pos == 3) Case1_Split(ids, sd, pos);
            else Case2_Split(ids, sd);
            _currentSplitTet = -1;

            if (_splitSuccesses == 0)
            {
                RollbackSplit(t);
                return false;
            }

            CommitQueuedCutSurfacePatches();
            return true;
        }

        void RollbackSplit(int parentTet)
        {
            _splitCutSurfacePatches.Clear();
            _splitCutSurfaceVertexIds.Clear();
            _splitCutSurfaceFaceKeys.Clear();
            _splitCutSurfaceEdgeKeys.Clear();

            for (int i = 0; i < _splitCreatedTets.Count; i++)
            {
                int child = _splitCreatedTets[i];
                if (child < 0 || child >= _data.NumTets) continue;
                if (!_data.TetActive[child]) continue;
                _data.TetActive[child] = false;
                RemoveTetFromAdjacency(child,
                    new[]
                    {
                        _data.TetIds[4 * child + 0],
                        _data.TetIds[4 * child + 1],
                        _data.TetIds[4 * child + 2],
                        _data.TetIds[4 * child + 3]
                    });
            }
            _splitCreatedTets.Clear();

            _data.TetActive[parentTet] = true;
            AddTetToAdjacency(parentTet,
                _data.TetIds[4 * parentTet + 0],
                _data.TetIds[4 * parentTet + 1],
                _data.TetIds[4 * parentTet + 2],
                _data.TetIds[4 * parentTet + 3]);
            _alreadyCutTets.Remove(parentTet);
            StrokeSplitRollbackCount++;
        }

        void Case1_Split(int[] v, float[] d, int posCount)
        {
            int isoSide = posCount == 1 ? 1 : -1;
            int isoIdx = -1;
            int[] otherIdx = new int[3];
            int oi = 0;

            for (int i = 0; i < 4; i++)
            {
                if ((d[i] >= 0f ? 1 : -1) == isoSide) isoIdx = i;
                else otherIdx[oi++] = i;
            }

            int A = v[isoIdx];
            int B = v[otherIdx[0]];
            int C = v[otherIdx[1]];
            int D = v[otherIdx[2]];
            bool isoIsPos = isoSide > 0;

            int b1p, b1n, c1p, c1n, d1p, d1n;
            EP(A, B, d[isoIdx], d[otherIdx[0]], out b1p, out b1n);
            EP(A, C, d[isoIdx], d[otherIdx[1]], out c1p, out c1n);
            EP(A, D, d[isoIdx], d[otherIdx[2]], out d1p, out d1n);

            int B1_iso = isoIsPos ? b1p : b1n;
            int C1_iso = isoIsPos ? c1p : c1n;
            int D1_iso = isoIsPos ? d1p : d1n;
            int B1_oth = isoIsPos ? b1n : b1p;
            int C1_oth = isoIsPos ? c1n : c1p;
            int D1_oth = isoIsPos ? d1n : d1p;

            int sIso = isoIsPos ? +1 : -1;
            int sOth = -sIso;

            MarkCutSurfaceVertices(B1_iso, C1_iso, D1_iso);
            MarkCutSurfaceVertices(B1_oth, C1_oth, D1_oth);
            MarkCutSurfaceFace(B1_iso, C1_iso, D1_iso, -1);
            MarkCutSurfaceFace(B1_oth, C1_oth, D1_oth, -1);
            QueueCutSurfacePatch(B1_iso, C1_iso, D1_iso, -1, false);
            QueueCutSurfacePatch(B1_oth, C1_oth, D1_oth, -1, true);

            RegisterChild(AT(A, B1_iso, C1_iso, D1_iso), sIso);
            RegisterChild(AT(B, B1_oth, C1_oth, C), sOth);
            RegisterChild(AT(B1_oth, C1_oth, D1_oth, C), sOth);
            RegisterChild(AT(B1_oth, C, D1_oth, D), sOth);
        }

        void Case2_Split(int[] v, float[] d)
        {
            int A = -1, D = -1, B = -1, C = -1;
            float dA = 0f, dD = 0f, dB = 0f, dC = 0f;
            int pi = 0, ni = 0;

            for (int i = 0; i < 4; i++)
            {
                if (d[i] >= 0f)
                {
                    if (pi == 0) { A = v[i]; dA = d[i]; }
                    else { D = v[i]; dD = d[i]; }
                    pi++;
                }
                else
                {
                    if (ni == 0) { B = v[i]; dB = d[i]; }
                    else { C = v[i]; dC = d[i]; }
                    ni++;
                }
            }

            int e1p, e1n, f1p, f1n, g1p, g1n, h1p, h1n;
            EP(A, B, dA, dB, out e1p, out e1n);
            EP(A, C, dA, dC, out f1p, out f1n);
            EP(D, C, dD, dC, out g1p, out g1n);
            EP(D, B, dD, dB, out h1p, out h1n);

            MarkCutSurfaceVertices(e1p, f1p, g1p, h1p);
            MarkCutSurfaceVertices(e1n, f1n, g1n, h1n);
            MarkCutSurfaceFace(e1p, f1p, g1p, h1p);
            MarkCutSurfaceFace(e1n, f1n, g1n, h1n);
            QueueCutSurfacePatch(e1p, f1p, g1p, h1p, false);
            QueueCutSurfacePatch(e1n, f1n, g1n, h1n, true);

            // A and D are on the positive side, B and C on the negative.
            RegisterChild(AT(A, e1p, f1p, h1p), +1);
            RegisterChild(AT(A, D, f1p, h1p), +1);
            RegisterChild(AT(D, f1p, g1p, h1p), +1);

            RegisterChild(AT(B, e1n, f1n, h1n), -1);
            RegisterChild(AT(B, C, f1n, h1n), -1);
            RegisterChild(AT(C, f1n, g1n, h1n), -1);
        }

        void RegisterChild(int childTet, int side)
        {
            if (childTet < 0) return;
            _separator?.NotifyChildSide(childTet, side);
        }

        void QueueCutSurfacePatch(int a, int b, int c, int d, bool reverseWinding)
        {
            if (a < 0 || b < 0 || c < 0) return;
            if (d >= 0)
            {
                if (a == b || a == c || a == d || b == c || b == d || c == d) return;
            }
            else
            {
                if (a == b || a == c || b == c) return;
            }

            Vector3 n = _currentCutNormal.sqrMagnitude > 1e-10f
                ? _currentCutNormal.normalized
                : Vector3.up;

            int count = d >= 0 ? 4 : 3;
            int[] ids = count == 4
                ? new[] { a, b, c, d }
                : new[] { a, b, c };
            SortPatchIdsInPlane(ids, count, n);
            if (!PatchHasArea(ids, count)) return;

            _splitCutSurfacePatches.Add(new CutSurfacePatch
            {
                a = ids[0],
                b = ids[1],
                c = ids[2],
                d = count == 4 ? ids[3] : -1,
                count = count,
                normal = n,
                reverseWinding = reverseWinding
            });
        }

        void MarkCutSurfaceVertices(params int[] ids)
        {
            for (int i = 0; i < ids.Length; i++)
            {
                int id = ids[i];
                if (id < 0) continue;
                if (id < _originalParticleCount) continue;
                if (!_splitCutSurfaceVertexIds.Contains(id))
                    _splitCutSurfaceVertexIds.Add(id);
            }
        }

        void MarkCutSurfaceFace(int a, int b, int c, int d)
        {
            if (a < 0 || b < 0 || c < 0) return;
            if (d < 0)
            {
                _splitCutSurfaceFaceKeys.Add(SurfaceReconstructor.FaceKey(a, b, c));
                MarkCutSurfaceEdge(a, b);
                MarkCutSurfaceEdge(b, c);
                MarkCutSurfaceEdge(c, a);
                return;
            }

            // Case2's subdivision template exposes the cut quad as these two
            // tet-boundary triangles. Marking all four possible triples hides
            // legal rim faces and visibly damages the cut edge.
            _splitCutSurfaceFaceKeys.Add(SurfaceReconstructor.FaceKey(a, b, d));
            _splitCutSurfaceFaceKeys.Add(SurfaceReconstructor.FaceKey(b, c, d));
            MarkCutSurfaceEdge(a, b);
            MarkCutSurfaceEdge(b, d);
            MarkCutSurfaceEdge(d, a);
            MarkCutSurfaceEdge(b, c);
            MarkCutSurfaceEdge(c, d);
            MarkCutSurfaceEdge(d, b);
        }

        void MarkCutSurfaceEdge(int a, int b)
        {
            if (a < 0 || b < 0 || a == b) return;
            _splitCutSurfaceEdgeKeys.Add(SurfaceReconstructor.EdgeKey(a, b));
        }

        void CommitQueuedCutSurfacePatches()
        {
            for (int i = 0; i < _splitCutSurfaceVertexIds.Count; i++)
                _cutSurfaceVertexIds.Add(_splitCutSurfaceVertexIds[i]);

            foreach (long key in _splitCutSurfaceFaceKeys)
            {
                _cutSurfaceFaceKeys.Add(key);
                _cutFaceRegistry.AddCutFaceKey(key, _strokeSerial, _patchId);
            }

            foreach (long key in _splitCutSurfaceEdgeKeys)
            {
                _cutSurfaceEdgeKeys.Add(key);
                _cutFaceRegistry.AddCutEdgeKey(key);
            }

            for (int i = 0; i < _splitCutSurfacePatches.Count; i++)
            {
                var patch = _splitCutSurfacePatches[i];
                _pendingCutSurfacePatches.Add(patch);
                RegisterCutSurfaceNormal(patch.normal);
                for (int j = 0; j < patch.count; j++)
                {
                    int id = patch.IdAt(j);
                    if (id >= _originalParticleCount) _cutSurfaceVertexIds.Add(id);
                }
            }

            _splitCutSurfacePatches.Clear();
            _splitCreatedTets.Clear();
            _splitCutSurfaceVertexIds.Clear();
            _splitCutSurfaceFaceKeys.Clear();
            _splitCutSurfaceEdgeKeys.Clear();
        }

        void RegisterCutSurfaceNormal(Vector3 normal)
        {
            if (normal.sqrMagnitude < 1e-10f) return;
            normal.Normalize();
            for (int i = 0; i < _cutSurfaceNormals.Count; i++)
            {
                if (Mathf.Abs(Vector3.Dot(_cutSurfaceNormals[i], normal)) > 0.995f)
                    return;
            }
            _cutSurfaceNormals.Add(normal);
        }

        void SortPatchIdsInPlane(int[] ids, int count, Vector3 normal)
        {
            if (ids == null || count < 3) return;

            Vector3 center = Vector3.zero;
            for (int i = 0; i < count; i++)
                center += _data.Positions[ids[i]];
            center /= count;

            Vector3 refAxis = Mathf.Abs(Vector3.Dot(normal, Vector3.up)) < 0.8f
                ? Vector3.up
                : Vector3.right;
            Vector3 u = Vector3.Cross(refAxis, normal);
            if (u.sqrMagnitude < 1e-10f) u = Vector3.right;
            u.Normalize();
            Vector3 v = Vector3.Cross(normal, u).normalized;

            for (int i = 1; i < count; i++)
            {
                int id = ids[i];
                float angle = PatchAngle(_data.Positions[id], center, u, v);
                int j = i - 1;
                while (j >= 0)
                {
                    float other = PatchAngle(_data.Positions[ids[j]], center, u, v);
                    if (other <= angle) break;
                    ids[j + 1] = ids[j];
                    j--;
                }
                ids[j + 1] = id;
            }

            Vector3 p0 = _data.Positions[ids[0]];
            Vector3 p1 = _data.Positions[ids[1]];
            Vector3 p2 = _data.Positions[ids[2]];
            if (Vector3.Dot(Vector3.Cross(p1 - p0, p2 - p0), normal) < 0f)
            {
                int tmp = ids[1];
                ids[1] = ids[count - 1];
                ids[count - 1] = tmp;
            }
        }

        static float PatchAngle(Vector3 p, Vector3 center, Vector3 u, Vector3 v)
        {
            Vector3 d = p - center;
            return Mathf.Atan2(Vector3.Dot(d, v), Vector3.Dot(d, u));
        }

        bool PatchHasArea(int[] ids, int count)
        {
            if (ids == null || count < 3) return false;

            Vector3 p0 = _data.Positions[ids[0]];
            Vector3 p1 = _data.Positions[ids[1]];
            Vector3 p2 = _data.Positions[ids[2]];
            if (Vector3.Cross(p1 - p0, p2 - p0).sqrMagnitude > 1e-12f)
                return true;

            if (count == 4)
            {
                Vector3 p3 = _data.Positions[ids[3]];
                if (Vector3.Cross(p2 - p0, p3 - p0).sqrMagnitude > 1e-12f)
                    return true;
            }

            return false;
        }

        void EP(int a, int b, float da, float db, out int posV, out int negV)
        {
            int lo = Mathf.Min(a, b);
            int hi = Mathf.Max(a, b);
            long key = (((long)lo) << 32) | (uint)hi;

            if (_frameSplitCache.TryGetValue(key, out var cached))
            {
                posV = cached.posV;
                negV = cached.negV;
                return;
            }

            bool isOriginalEdge = a < _strokeStartVertCount && b < _strokeStartVertCount;
            var strokeKey = (_patchId, key);
            if (isOriginalEdge && _strokeSplitCache.TryGetValue(strokeKey, out cached))
            {
                _frameSplitCache[key] = cached;
                posV = cached.posV;
                negV = cached.negV;
                return;
            }

            float denom = da - db;
            float s = Mathf.Abs(denom) < EPS ? 0.5f : da / denom;
            s = Mathf.Clamp01(s);

            // Step 2 trajectory correction: snap when the intersection lies
            // close to either endpoint of the edge. Using the parametric
            // distance (SNAP_EDGE_FRAC of the edge) gives a scale-invariant
            // criterion that actually fires on the dominant sliver case,
            // unlike the previous absolute 1e-4 threshold.
            (int posV, int negV) pair;
            float snapFrac = EdgeTouchesGeneratedCutSurface(a, b)
                ? GENERATED_EDGE_SNAP_FRAC
                : EdgeIsOnActiveBoundary(a, b)
                    ? BOUNDARY_EDGE_SNAP_FRAC
                    : SNAP_EDGE_FRAC;

            if (s <= snapFrac)
            {
                if (!_frameSnapCache.TryGetValue(a, out int clone))
                {
                    clone = CloneVertex(a);
                    _frameSnapCache[a] = clone;
                }
                pair = da >= 0f ? (a, clone) : (clone, a);
            }
            else if (s >= 1f - snapFrac)
            {
                if (!_frameSnapCache.TryGetValue(b, out int clone))
                {
                    clone = CloneVertex(b);
                    _frameSnapCache[b] = clone;
                }
                pair = db >= 0f ? (b, clone) : (clone, b);
            }
            else
            {
                Vector3 p = Vector3.Lerp(_data.Positions[a], _data.Positions[b], s);
                Vector3 r = Vector3.Lerp(_data.RestPositions[a], _data.RestPositions[b], s);
                Vector3 pv = Vector3.Lerp(_data.PrevPositions[a], _data.PrevPositions[b], s);
                Vector3 vel = Vector3.Lerp(_data.Velocities[a], _data.Velocities[b], s);
                float ia = _data.InvMass[a];
                float ib = _data.InvMass[b];
                float inv = (ia == 0f || ib == 0f) ? 0f : Mathf.Lerp(ia, ib, s);

                int vPos = AddVertex(p, r, pv, vel, inv);
                int vNeg = AddVertex(p, r, pv, vel, inv);
                AssignGeneratedSurfaceSupport(vPos, a, b);
                AssignGeneratedSurfaceSupport(vNeg, a, b);
                pair = (vPos, vNeg);
            }

            EnsureVertexSlot(pair.posV);
            EnsureVertexSlot(pair.negV);

            posV = pair.posV;
            negV = pair.negV;
            _frameSplitCache[key] = pair;
            if (isOriginalEdge) _strokeSplitCache[strokeKey] = pair;
        }

        // Returns the newly created tet id or -1 if the tet was rejected
        // (degenerate / sub-MIN_TET_VOLUME). Callers in Case1_Split and
        // Case2_Split use the returned id to register the child tet with
        // the SharedVertexSeparator so that the side it belongs to is
        // recorded for the per-frame shared-vertex separation pass.
        //
        // Tracks per-SplitTet success/attempt counters that SplitTet
        // inspects after Case[12]_Split returns: if *every* child was
        // rejected we roll back the parent's deactivation, preventing
        // the cut surface from developing an empty hole where a tet
        // used to be.
        int AT(int a, int b, int c, int d)
        {
            if (a == b || a == c || a == d || b == c || b == d || c == d) return -1;

            float signedVol = TetVolume(
                _data.RestPositions[a],
                _data.RestPositions[b],
                _data.RestPositions[c],
                _data.RestPositions[d]);
            if (Mathf.Abs(signedVol) < MIN_TET_VOLUME) return -1;

            _splitAttempts++;
            int tet;
            if (signedVol < 0f)
            {
                tet = _data.AddTet(a, c, b, d);
                AddTetToAdjacency(tet, a, c, b, d);
            }
            else
            {
                tet = _data.AddTet(a, b, c, d);
                AddTetToAdjacency(tet, a, b, c, d);
            }

            MarkTetBornInCurrentPatch(tet);
            _splitCreatedTets.Add(tet);
            _spatialIndex.AddTet(_data, tet);
            _splitSuccesses++;
            return tet;
        }

        int CloneVertex(int v)
        {
            int clone = AddVertex(
                _data.Positions[v],
                _data.RestPositions[v],
                _data.PrevPositions[v],
                _data.Velocities[v],
                _data.InvMass[v]);

            EnsureSurfaceRootSlot(clone);
            CopySurfaceSupport(clone, v);
            return clone;
        }

        int AddVertex(Vector3 pos, Vector3 rest, Vector3 prev, Vector3 velocity, float invMass)
        {
            int v = _data.AddParticle(pos, velocity, invMass);
            _data.RestPositions[v] = rest;
            _data.PrevPositions[v] = prev;
            EnsureSurfaceRootSlot(v);
            EnsureSurfaceSupportSlot(v);
            _surfaceRootIds[v] = -1;
            _surfaceSupport0[v] = -1;
            _surfaceSupport1[v] = -1;
            _surfaceSupport2[v] = -1;
            _stabilizeVertexIds.Add(v);
            return v;
        }

        void EnsureSurfaceRootSlot(int v)
        {
            while (_surfaceRootIds.Count <= v)
                _surfaceRootIds.Add(-1);
        }

        void EnsureSurfaceSupportSlot(int v)
        {
            while (_surfaceSupport0.Count <= v)
            {
                _surfaceSupport0.Add(-1);
                _surfaceSupport1.Add(-1);
                _surfaceSupport2.Add(-1);
            }
        }

        void SetSurfaceSupport(int v, int s0, int s1 = -1, int s2 = -1)
        {
            EnsureSurfaceRootSlot(v);
            EnsureSurfaceSupportSlot(v);
            _surfaceSupport0[v] = s0;
            _surfaceSupport1[v] = s1;
            _surfaceSupport2[v] = s2;
            _surfaceRootIds[v] = s1 < 0 && s2 < 0 ? s0 : -1;
        }

        void CopySurfaceSupport(int dst, int src)
        {
            EnsureSurfaceRootSlot(dst);
            EnsureSurfaceSupportSlot(dst);
            if (src < 0 || src >= _surfaceSupport0.Count)
            {
                SetSurfaceSupport(dst, -1);
                return;
            }

            _surfaceSupport0[dst] = _surfaceSupport0[src];
            _surfaceSupport1[dst] = _surfaceSupport1[src];
            _surfaceSupport2[dst] = _surfaceSupport2[src];
            _surfaceRootIds[dst] = src >= 0 && src < _surfaceRootIds.Count
                ? _surfaceRootIds[src]
                : -1;
        }

        void AssignGeneratedSurfaceSupport(int v, int a, int b)
        {
            int r0 = -1;
            int r1 = -1;
            int r2 = -1;
            int count = 0;
            if (!CollectVertexSurfaceSupport(a, ref r0, ref r1, ref r2, ref count) ||
                !CollectVertexSurfaceSupport(b, ref r0, ref r1, ref r2, ref count))
            {
                return;
            }

            if (count == 1)
            {
                SetSurfaceSupport(v, r0);
            }
            else if (count == 2)
            {
                if (_originalSurfaceEdgeKeys.Contains(SurfaceReconstructor.EdgeKey(r0, r1)))
                    SetSurfaceSupport(v, r0, r1);
            }
            else if (count == 3)
            {
                if (_originalSurfaceFaceKeys.Contains(SurfaceReconstructor.FaceKey(r0, r1, r2)))
                    SetSurfaceSupport(v, r0, r1, r2);
            }
        }

        bool CollectVertexSurfaceSupport(int vertex, ref int r0, ref int r1, ref int r2, ref int count)
        {
            if (vertex < 0 || vertex >= _surfaceSupport0.Count) return false;
            bool any = false;
            if (!AppendSurfaceSupport(_surfaceSupport0[vertex], ref r0, ref r1, ref r2, ref count, ref any))
                return false;
            if (!AppendSurfaceSupport(_surfaceSupport1[vertex], ref r0, ref r1, ref r2, ref count, ref any))
                return false;
            if (!AppendSurfaceSupport(_surfaceSupport2[vertex], ref r0, ref r1, ref r2, ref count, ref any))
                return false;
            return any;
        }

        bool AppendSurfaceSupport(int root, ref int r0, ref int r1, ref int r2, ref int count, ref bool any)
        {
            if (root < 0) return true;
            any = true;
            if ((count > 0 && r0 == root) ||
                (count > 1 && r1 == root) ||
                (count > 2 && r2 == root))
            {
                return true;
            }

            if (count >= 3) return false;
            if (count == 0) r0 = root;
            else if (count == 1) r1 = root;
            else r2 = root;
            count++;
            return true;
        }

        bool EdgeTouchesGeneratedCutSurface(int a, int b)
        {
            return a >= _strokeStartVertCount ||
                   b >= _strokeStartVertCount;
        }

        bool EdgeIsOnActiveBoundary(int a, int b)
        {
            if (_data == null) return false;
            if (_currentSplitTet >= 0 && TetEdgeHasBoundaryFace(_currentSplitTet, a, b))
                return true;

            if (!_vertexToTets.TryGetValue(a, out var tets)) return false;

            for (int i = 0; i < tets.Count; i++)
            {
                int t = tets[i];
                if (t < 0 || t >= _data.NumTets || !_data.TetActive[t]) continue;

                int baseIdx = 4 * t;
                int i0 = _data.TetIds[baseIdx + 0];
                int i1 = _data.TetIds[baseIdx + 1];
                int i2 = _data.TetIds[baseIdx + 2];
                int i3 = _data.TetIds[baseIdx + 3];
                if (i0 != b && i1 != b && i2 != b && i3 != b) continue;

                if (FaceContainsEdge(i0, i2, i1, a, b) && ActiveFaceIncidentCount(i0, i2, i1) == 1) return true;
                if (FaceContainsEdge(i0, i1, i3, a, b) && ActiveFaceIncidentCount(i0, i1, i3) == 1) return true;
                if (FaceContainsEdge(i0, i3, i2, a, b) && ActiveFaceIncidentCount(i0, i3, i2) == 1) return true;
                if (FaceContainsEdge(i1, i2, i3, a, b) && ActiveFaceIncidentCount(i1, i2, i3) == 1) return true;
            }

            return false;
        }

        bool TetEdgeHasBoundaryFace(int t, int a, int b)
        {
            if (t < 0 || t >= _data.NumTets) return false;

            int baseIdx = 4 * t;
            int i0 = _data.TetIds[baseIdx + 0];
            int i1 = _data.TetIds[baseIdx + 1];
            int i2 = _data.TetIds[baseIdx + 2];
            int i3 = _data.TetIds[baseIdx + 3];

            if (FaceContainsEdge(i0, i2, i1, a, b) && ActiveFaceIncidentCount(i0, i2, i1) == 1) return true;
            if (FaceContainsEdge(i0, i1, i3, a, b) && ActiveFaceIncidentCount(i0, i1, i3) == 1) return true;
            if (FaceContainsEdge(i0, i3, i2, a, b) && ActiveFaceIncidentCount(i0, i3, i2) == 1) return true;
            if (FaceContainsEdge(i1, i2, i3, a, b) && ActiveFaceIncidentCount(i1, i2, i3) == 1) return true;

            return false;
        }

        int ActiveFaceIncidentCount(int a, int b, int c)
        {
            int count = 0;
            if (_currentSplitTet >= 0 && TetContainsFace(_currentSplitTet, a, b, c))
                count++;

            if (_vertexToTets.TryGetValue(a, out var tets))
            {
                for (int i = 0; i < tets.Count; i++)
                {
                    int t = tets[i];
                    if (t < 0 || t >= _data.NumTets || !_data.TetActive[t]) continue;
                    if (!TetContainsFace(t, a, b, c)) continue;

                    count++;
                    if (count > 1) return count;
                }
            }

            return count;
        }

        bool TetContainsFace(int t, int a, int b, int c)
        {
            if (t < 0 || t >= _data.NumTets) return false;

            int baseIdx = 4 * t;
            bool hasA = false;
            bool hasB = false;
            bool hasC = false;
            for (int k = 0; k < 4; k++)
            {
                int v = _data.TetIds[baseIdx + k];
                if (v == a) hasA = true;
                else if (v == b) hasB = true;
                else if (v == c) hasC = true;
            }

            return hasA && hasB && hasC;
        }

        static bool FaceContainsEdge(int x, int y, int z, int a, int b)
        {
            bool hasA = x == a || y == a || z == a;
            bool hasB = x == b || y == b || z == b;
            return hasA && hasB;
        }

        void StabilizePatchPlane(Vector3 moveDir, ref Vector3 planeNormal, ref float planeD)
        {
            if (!_hasPatchPlane)
            {
                BeginPatchPlane(moveDir, planeNormal, planeD);
                return;
            }

            if (Vector3.Dot(planeNormal, _patchNormal) < 0f)
            {
                planeNormal = -planeNormal;
                planeD = -planeD;
            }

            float normalMin = Mathf.Cos(PATCH_MAX_NORMAL_ANGLE_DEG * Mathf.Deg2Rad);
            float moveMin = Mathf.Cos(PATCH_MAX_MOVE_ANGLE_DEG * Mathf.Deg2Rad);
            bool normalStillStraight = Vector3.Dot(planeNormal, _patchNormal) >= normalMin;
            bool moveStillStraight =
                moveDir.sqrMagnitude < 1e-10f ||
                _patchMoveDir.sqrMagnitude < 1e-10f ||
                Mathf.Abs(Vector3.Dot(moveDir, _patchMoveDir)) >= moveMin;
            bool offsetStillStraight = Mathf.Abs(planeD - _patchPlaneD) <= PATCH_MAX_OFFSET;

            if (normalStillStraight && moveStillStraight && offsetStillStraight)
            {
                planeNormal = _patchNormal;
                planeD = _patchPlaneD;
                return;
            }

            BeginPatchPlane(moveDir, planeNormal, planeD);
        }

        void BeginPatchPlane(Vector3 moveDir, Vector3 planeNormal, float planeD)
        {
            _patchNormal = planeNormal.normalized;
            _patchMoveDir = moveDir.sqrMagnitude > 1e-10f ? moveDir.normalized : Vector3.zero;
            _patchPlaneD = planeD;
            _patchId++;
            _hasPatchPlane = true;
        }

        bool TetPlaneIntersectsRibbon(
            int t,
            float d0, float d1, float d2, float d3,
            Vector3 planePoint,
            Vector3 bladeDir,
            Vector3 moveDir,
            float bladeLen,
            float moveLen)
        {
            if (moveLen < 1e-6f || moveDir.sqrMagnitude < 1e-10f) return true;

            int b = 4 * t;
            Vector3 p0 = _data.Positions[_data.TetIds[b + 0]];
            Vector3 p1 = _data.Positions[_data.TetIds[b + 1]];
            Vector3 p2 = _data.Positions[_data.TetIds[b + 2]];
            Vector3 p3 = _data.Positions[_data.TetIds[b + 3]];

            int count = 0;
            AddPlaneEdgePoint(p0, p1, d0, d1, planePoint, bladeDir, moveDir, ref count);
            AddPlaneEdgePoint(p0, p2, d0, d2, planePoint, bladeDir, moveDir, ref count);
            AddPlaneEdgePoint(p0, p3, d0, d3, planePoint, bladeDir, moveDir, ref count);
            AddPlaneEdgePoint(p1, p2, d1, d2, planePoint, bladeDir, moveDir, ref count);
            AddPlaneEdgePoint(p1, p3, d1, d3, planePoint, bladeDir, moveDir, ref count);
            AddPlaneEdgePoint(p2, p3, d2, d3, planePoint, bladeDir, moveDir, ref count);

            if (count == 0) return false;

            float halfU = bladeLen * 0.5f + RIBBON_INTERSECTION_MARGIN;
            float halfV = moveLen * 0.5f + RIBBON_INTERSECTION_MARGIN;
            for (int i = 0; i < count; i++)
            {
                if (PointInRibbonRect(_ribbonCutPolygon[i], halfU, halfV)) return true;
            }

            SortPolygon2D(_ribbonCutPolygon, count);

            if (count >= 3)
            {
                if (PointInConvexPolygon(new Vector2(-halfU, -halfV), _ribbonCutPolygon, count)) return true;
                if (PointInConvexPolygon(new Vector2( halfU, -halfV), _ribbonCutPolygon, count)) return true;
                if (PointInConvexPolygon(new Vector2( halfU,  halfV), _ribbonCutPolygon, count)) return true;
                if (PointInConvexPolygon(new Vector2(-halfU,  halfV), _ribbonCutPolygon, count)) return true;
            }

            for (int i = 0; i < count; i++)
            {
                Vector2 a = _ribbonCutPolygon[i];
                Vector2 c = _ribbonCutPolygon[(i + 1) % count];
                if (SegmentIntersectsRibbonRect(a, c, halfU, halfV)) return true;
            }

            return false;
        }

        void AddPlaneEdgePoint(
            Vector3 a, Vector3 b,
            float da, float db,
            Vector3 planePoint,
            Vector3 bladeDir,
            Vector3 moveDir,
            ref int count)
        {
            if (count >= _ribbonCutPolygon.Length) return;

            float denom = da - db;
            if (Mathf.Abs(denom) < EPS) return;
            if (da * db > 0f) return;

            float s = Mathf.Clamp01(da / denom);
            Vector3 p = Vector3.Lerp(a, b, s);
            Vector3 local = p - planePoint;
            Vector2 q = new Vector2(Vector3.Dot(local, bladeDir), Vector3.Dot(local, moveDir));

            for (int i = 0; i < count; i++)
            {
                if ((_ribbonCutPolygon[i] - q).sqrMagnitude < 1e-10f) return;
            }
            _ribbonCutPolygon[count++] = q;
        }

        static bool PointInRibbonRect(Vector2 p, float halfU, float halfV)
        {
            return Mathf.Abs(p.x) <= halfU && Mathf.Abs(p.y) <= halfV;
        }

        static void SortPolygon2D(Vector2[] points, int count)
        {
            if (count < 3) return;
            Vector2 center = Vector2.zero;
            for (int i = 0; i < count; i++) center += points[i];
            center /= count;

            for (int i = 1; i < count; i++)
            {
                Vector2 p = points[i];
                float angle = Mathf.Atan2(p.y - center.y, p.x - center.x);
                int j = i - 1;
                while (j >= 0)
                {
                    float other = Mathf.Atan2(points[j].y - center.y, points[j].x - center.x);
                    if (other <= angle) break;
                    points[j + 1] = points[j];
                    j--;
                }
                points[j + 1] = p;
            }
        }

        static bool PointInConvexPolygon(Vector2 p, Vector2[] polygon, int count)
        {
            bool hasPos = false;
            bool hasNeg = false;
            for (int i = 0; i < count; i++)
            {
                Vector2 a = polygon[i];
                Vector2 b = polygon[(i + 1) % count];
                float cross = Cross2D(b - a, p - a);
                if (cross > 1e-7f) hasPos = true;
                else if (cross < -1e-7f) hasNeg = true;
                if (hasPos && hasNeg) return false;
            }
            return true;
        }

        static bool SegmentIntersectsRibbonRect(Vector2 a, Vector2 b, float halfU, float halfV)
        {
            if (PointInRibbonRect(a, halfU, halfV) || PointInRibbonRect(b, halfU, halfV)) return true;

            Vector2 r0 = new Vector2(-halfU, -halfV);
            Vector2 r1 = new Vector2( halfU, -halfV);
            Vector2 r2 = new Vector2( halfU,  halfV);
            Vector2 r3 = new Vector2(-halfU,  halfV);

            return SegmentsIntersect(a, b, r0, r1) ||
                   SegmentsIntersect(a, b, r1, r2) ||
                   SegmentsIntersect(a, b, r2, r3) ||
                   SegmentsIntersect(a, b, r3, r0);
        }

        static bool SegmentsIntersect(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
        {
            float c1 = Cross2D(b - a, c - a);
            float c2 = Cross2D(b - a, d - a);
            float c3 = Cross2D(d - c, a - c);
            float c4 = Cross2D(d - c, b - c);

            if (((c1 > 0f && c2 < 0f) || (c1 < 0f && c2 > 0f)) &&
                ((c3 > 0f && c4 < 0f) || (c3 < 0f && c4 > 0f)))
            {
                return true;
            }

            const float e = 1e-7f;
            return Mathf.Abs(c1) <= e && PointOnSegment(c, a, b) ||
                   Mathf.Abs(c2) <= e && PointOnSegment(d, a, b) ||
                   Mathf.Abs(c3) <= e && PointOnSegment(a, c, d) ||
                   Mathf.Abs(c4) <= e && PointOnSegment(b, c, d);
        }

        static bool PointOnSegment(Vector2 p, Vector2 a, Vector2 b)
        {
            return p.x >= Mathf.Min(a.x, b.x) - 1e-7f &&
                   p.x <= Mathf.Max(a.x, b.x) + 1e-7f &&
                   p.y >= Mathf.Min(a.y, b.y) - 1e-7f &&
                   p.y <= Mathf.Max(a.y, b.y) + 1e-7f;
        }

        static float Cross2D(Vector2 a, Vector2 b)
        {
            return a.x * b.y - a.y * b.x;
        }

        void EnforceConsistentSide(int vertex, ref float d)
        {
            if (vertex < 0 || vertex >= _strokeStartVertCount) return;
            if (Mathf.Abs(d) > SNAP_DIST) return;
            d = d >= 0f ? SNAP_DIST : -SNAP_DIST;
        }

        static int CountPositive(float d0, float d1, float d2, float d3)
        {
            int pos = 0;
            if (d0 > 0f) pos++;
            if (d1 > 0f) pos++;
            if (d2 > 0f) pos++;
            if (d3 > 0f) pos++;
            return pos;
        }

        static float SnapSignedDistance(float d)
        {
            if (Mathf.Abs(d) < SNAP_DIST) return d >= 0f ? SNAP_DIST : -SNAP_DIST;
            return d;
        }

        bool TetAabbOverlap(int t, Vector3 lo, Vector3 hi)
        {
            GetTetAabb(t, out var a, out var b);
            return a.x <= hi.x && b.x >= lo.x &&
                   a.y <= hi.y && b.y >= lo.y &&
                   a.z <= hi.z && b.z >= lo.z;
        }

        void GetTetAabb(int t, out Vector3 lo, out Vector3 hi)
        {
            Vector3 p0 = _data.Positions[_data.TetIds[4 * t + 0]];
            Vector3 p1 = _data.Positions[_data.TetIds[4 * t + 1]];
            Vector3 p2 = _data.Positions[_data.TetIds[4 * t + 2]];
            Vector3 p3 = _data.Positions[_data.TetIds[4 * t + 3]];
            lo = Vector3.Min(Vector3.Min(p0, p1), Vector3.Min(p2, p3));
            hi = Vector3.Max(Vector3.Max(p0, p1), Vector3.Max(p2, p3));
        }

        float SegmentAabbDistanceSq(Vector3 a, Vector3 b, int tet)
        {
            GetTetAabb(tet, out var lo, out var hi);
            Vector3 mid = 0.5f * (a + b);
            float best = AabbPointDistanceSq(mid, lo, hi);
            best = Mathf.Min(best, AabbPointDistanceSq(a, lo, hi));
            best = Mathf.Min(best, AabbPointDistanceSq(b, lo, hi));
            return best;
        }

        static float AabbPointDistanceSq(Vector3 p, Vector3 lo, Vector3 hi)
        {
            float dx = p.x < lo.x ? lo.x - p.x : (p.x > hi.x ? p.x - hi.x : 0f);
            float dy = p.y < lo.y ? lo.y - p.y : (p.y > hi.y ? p.y - hi.y : 0f);
            float dz = p.z < lo.z ? lo.z - p.z : (p.z > hi.z ? p.z - hi.z : 0f);
            return dx * dx + dy * dy + dz * dz;
        }

        static float PointSegmentDistanceSq(Vector3 p, Vector3 a, Vector3 b)
        {
            Vector3 ab = b - a;
            float denom = Vector3.Dot(ab, ab);
            if (denom < EPS) return (p - a).sqrMagnitude;
            float t = Mathf.Clamp01(Vector3.Dot(p - a, ab) / denom);
            Vector3 q = a + ab * t;
            return (p - q).sqrMagnitude;
        }

        Vector3 TetCenter(int t)
        {
            return 0.25f * (
                _data.Positions[_data.TetIds[4 * t + 0]] +
                _data.Positions[_data.TetIds[4 * t + 1]] +
                _data.Positions[_data.TetIds[4 * t + 2]] +
                _data.Positions[_data.TetIds[4 * t + 3]]);
        }

        static Vector3 StableNormal(Vector3 bladeDir)
        {
            Vector3 axis = Mathf.Abs(Vector3.Dot(bladeDir, Vector3.up)) < 0.8f ? Vector3.up : Vector3.right;
            return Vector3.Cross(bladeDir, axis).normalized;
        }

        static float TetVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            return Vector3.Dot(b - a, Vector3.Cross(c - a, d - a)) / 6f;
        }

        void EnsureVertexSlot(int v)
        {
            if (!_vertexToTets.ContainsKey(v)) _vertexToTets[v] = new List<int>(8);
        }

        void RebuildVertexAdjacency()
        {
            _vertexToTets.Clear();
            if (_data == null) return;
            EnsureTetMetadataSlots(_data.NumTets);
            for (int v = 0; v < _data.NumParticles; v++) EnsureVertexSlot(v);
            for (int t = 0; t < _data.NumTets; t++)
            {
                if (!_data.TetActive[t]) continue;
                AddTetToAdjacency(t,
                    _data.TetIds[4 * t + 0],
                    _data.TetIds[4 * t + 1],
                    _data.TetIds[4 * t + 2],
                    _data.TetIds[4 * t + 3]);
            }
        }

        void AddTetToAdjacency(int t, int a, int b, int c, int d)
        {
            AddTetReference(a, t);
            AddTetReference(b, t);
            AddTetReference(c, t);
            AddTetReference(d, t);
        }

        void RemoveTetFromAdjacency(int t, int[] ids)
        {
            for (int i = 0; i < ids.Length; i++) RemoveTetReference(ids[i], t);
        }

        // Called by SharedVertexSeparator when a pivot vertex must be split
        // into a fresh clone for one of its disconnected neighbour components.
        // The tet is left otherwise untouched: positions, rest volumes, and
        // adjacency for the other three vertices stay valid because the clone
        // has the same world/rest position as the original.
        void ReplaceVertexInTet(int t, int oldV, int newV)
        {
            if (_data == null) return;
            if (t < 0 || t >= _data.NumTets) return;
            if (oldV == newV) return;

            int b = 4 * t;
            int[] oldIds =
            {
                _data.TetIds[b + 0],
                _data.TetIds[b + 1],
                _data.TetIds[b + 2],
                _data.TetIds[b + 3]
            };
            int slot = -1;
            for (int k = 0; k < 4; k++)
            {
                if (oldIds[k] == oldV) { slot = k; break; }
            }
            if (slot < 0) return;

            _data.TetIds[b + slot] = newV;
            RemoveTetReference(oldV, t);
            AddTetReference(newV, t);

            if (_cutSurfaceVertexIds.Contains(oldV))
            {
                _cutSurfaceVertexIds.Add(newV);
                CopyCommittedCutFacesForClone(oldIds, oldV, newV);
                CopyCommittedCutEdgesForClone(t, oldV, newV);
            }

            MarkSeparatedCutFaces(t);
        }

        void CopyCommittedCutFacesForClone(int[] oldIds, int oldV, int newV)
        {
            if (oldIds == null || oldIds.Length < 4) return;

            CopyCommittedCutFaceForClone(oldIds[0], oldIds[2], oldIds[1], oldV, newV);
            CopyCommittedCutFaceForClone(oldIds[0], oldIds[1], oldIds[3], oldV, newV);
            CopyCommittedCutFaceForClone(oldIds[0], oldIds[3], oldIds[2], oldV, newV);
            CopyCommittedCutFaceForClone(oldIds[1], oldIds[2], oldIds[3], oldV, newV);
        }

        void CopyCommittedCutFaceForClone(int a, int b, int c, int oldV, int newV)
        {
            if (a != oldV && b != oldV && c != oldV) return;
            if (!_cutSurfaceFaceKeys.Contains(SurfaceReconstructor.FaceKey(a, b, c))) return;

            if (a == oldV) a = newV;
            if (b == oldV) b = newV;
            if (c == oldV) c = newV;
            MarkCommittedCutSurfaceFace(a, b, c);
        }

        void CopyCommittedCutEdgesForClone(int t, int oldV, int newV)
        {
            int b = 4 * t;
            for (int k = 0; k < 4; k++)
            {
                int other = _data.TetIds[b + k];
                if (other < 0 || other == oldV || other == newV) continue;
                if (_cutSurfaceEdgeKeys.Contains(SurfaceReconstructor.EdgeKey(oldV, other)))
                {
                    _cutSurfaceEdgeKeys.Add(SurfaceReconstructor.EdgeKey(newV, other));
                    _cutFaceRegistry.AddCutEdge(newV, other);
                }
            }
        }

        void MarkSeparatedCutFaces(int t)
        {
            int b = 4 * t;
            int i0 = _data.TetIds[b + 0];
            int i1 = _data.TetIds[b + 1];
            int i2 = _data.TetIds[b + 2];
            int i3 = _data.TetIds[b + 3];

            TryMarkSeparatedCutFace(i0, i2, i1);
            TryMarkSeparatedCutFace(i0, i1, i3);
            TryMarkSeparatedCutFace(i0, i3, i2);
            TryMarkSeparatedCutFace(i1, i2, i3);
        }

        void TryMarkSeparatedCutFace(int a, int b, int c)
        {
            if (!_cutSurfaceVertexIds.Contains(a) ||
                !_cutSurfaceVertexIds.Contains(b) ||
                !_cutSurfaceVertexIds.Contains(c))
            {
                return;
            }

            if (!_cutSurfaceEdgeKeys.Contains(SurfaceReconstructor.EdgeKey(a, b)) ||
                !_cutSurfaceEdgeKeys.Contains(SurfaceReconstructor.EdgeKey(b, c)) ||
                !_cutSurfaceEdgeKeys.Contains(SurfaceReconstructor.EdgeKey(c, a)))
            {
                return;
            }

            Vector3 n = Vector3.Cross(
                _data.Positions[b] - _data.Positions[a],
                _data.Positions[c] - _data.Positions[a]);
            if (n.sqrMagnitude < 1e-12f)
            {
                MarkCommittedCutSurfaceFace(a, b, c);
                return;
            }
            n.Normalize();

            for (int i = 0; i < _cutSurfaceNormals.Count; i++)
            {
                Vector3 cn = _cutSurfaceNormals[i];
                if (cn.sqrMagnitude < 1e-10f) continue;
                cn.Normalize();
                if (Mathf.Abs(Vector3.Dot(n, cn)) >= 0.5f)
                {
                    MarkCommittedCutSurfaceFace(a, b, c);
                    return;
                }
            }
        }

        void MarkCommittedCutSurfaceFace(int a, int b, int c)
        {
            if (a < 0 || b < 0 || c < 0) return;
            _cutSurfaceFaceKeys.Add(SurfaceReconstructor.FaceKey(a, b, c));
            _cutFaceRegistry.AddCutFace(a, b, c, _strokeSerial, _patchId);
            MarkCommittedCutSurfaceEdge(a, b);
            MarkCommittedCutSurfaceEdge(b, c);
            MarkCommittedCutSurfaceEdge(c, a);
        }

        void MarkCommittedCutSurfaceEdge(int a, int b)
        {
            if (a < 0 || b < 0 || a == b) return;
            _cutSurfaceEdgeKeys.Add(SurfaceReconstructor.EdgeKey(a, b));
            _cutFaceRegistry.AddCutEdge(a, b);
        }

        bool WasBornInCurrentPatch(int tet)
        {
            if (tet < 0 || tet >= _tetBirthStroke.Count) return false;
            return _tetBirthStroke[tet] == _strokeSerial &&
                   _tetBirthPatch[tet] == _patchId;
        }

        bool WasBornInCurrentStroke(int tet)
        {
            if (tet < 0 || tet >= _tetBirthStroke.Count) return false;
            return _tetBirthStroke[tet] == _strokeSerial;
        }

        void MarkTetBornInCurrentPatch(int tet)
        {
            EnsureTetMetadataSlots(tet + 1);
            _tetBirthStroke[tet] = _strokeSerial;
            _tetBirthPatch[tet] = _patchId;
        }

        void EnsureTetMetadataSlots(int count)
        {
            while (_tetBirthStroke.Count < count)
            {
                _tetBirthStroke.Add(-1);
                _tetBirthPatch.Add(-1);
            }
        }

        void AddTetReference(int v, int t)
        {
            EnsureVertexSlot(v);
            List<int> list = _vertexToTets[v];
            if (!list.Contains(t)) list.Add(t);
        }

        void RemoveTetReference(int v, int t)
        {
            if (!_vertexToTets.TryGetValue(v, out var list)) return;
            for (int i = list.Count - 1; i >= 0; i--)
            {
                if (list[i] == t) list.RemoveAt(i);
            }
        }

        void RebuildSpatialIndex(int tetLimit)
        {
            if (_data == null) return;
            _spatialIndex.Build(_data, tetLimit);
            _indexDirty = false;
        }

        class SpatialTetIndex
        {
            const float CELL = 0.04f;

            readonly Dictionary<Vector3Int, List<int>> _cells = new Dictionary<Vector3Int, List<int>>(4096);
            readonly List<int> _stamp = new List<int>(4096);
            int _queryId = 1;
            Bounds _bounds;
            bool _hasBounds;
            TetMeshData _data;

            public void Build(TetMeshData data, int tetLimit)
            {
                _data = data;
                _cells.Clear();
                _hasBounds = false;
                if (data == null) return;

                int limit = Mathf.Clamp(tetLimit, 0, data.NumTets);
                EnsureStampSize(data.NumTets);
                for (int t = 0; t < limit; t++)
                {
                    if (!data.TetActive[t]) continue;
                    GetTetAabb(data, t, out var lo, out var hi);
                    AddBounds(lo, hi);

                    Vector3Int c0 = ToCell(lo);
                    Vector3Int c1 = ToCell(hi);
                    for (int x = c0.x; x <= c1.x; x++)
                    for (int y = c0.y; y <= c1.y; y++)
                    for (int z = c0.z; z <= c1.z; z++)
                    {
                        Vector3Int key = new Vector3Int(x, y, z);
                        if (!_cells.TryGetValue(key, out var list))
                        {
                            list = new List<int>(8);
                            _cells[key] = list;
                        }
                        list.Add(t);
                    }
                }
            }

            public void AddTet(TetMeshData data, int t)
            {
                if (data == null || t < 0 || t >= data.NumTets || !data.TetActive[t]) return;
                if (_data == null) _data = data;

                EnsureStampSize(data.NumTets);
                GetTetAabb(data, t, out var lo, out var hi);
                AddBounds(lo, hi);

                Vector3Int c0 = ToCell(lo);
                Vector3Int c1 = ToCell(hi);
                for (int x = c0.x; x <= c1.x; x++)
                for (int y = c0.y; y <= c1.y; y++)
                for (int z = c0.z; z <= c1.z; z++)
                {
                    Vector3Int key = new Vector3Int(x, y, z);
                    if (!_cells.TryGetValue(key, out var list))
                    {
                        list = new List<int>(8);
                        _cells[key] = list;
                    }
                    list.Add(t);
                }
            }

            public bool OverlapsBounds(Vector3 lo, Vector3 hi)
            {
                if (!_hasBounds) return false;
                return lo.x <= _bounds.max.x && hi.x >= _bounds.min.x &&
                       lo.y <= _bounds.max.y && hi.y >= _bounds.min.y &&
                       lo.z <= _bounds.max.z && hi.z >= _bounds.min.z;
            }

            public void QueryAabb(Vector3 lo, Vector3 hi, int maxTetExclusive, List<int> output)
            {
                output.Clear();
                if (_data == null || _cells.Count == 0) return;

                _queryId++;
                if (_queryId == int.MaxValue)
                {
                    _queryId = 1;
                    for (int i = 0; i < _stamp.Count; i++) _stamp[i] = 0;
                }

                Vector3Int c0 = ToCell(lo);
                Vector3Int c1 = ToCell(hi);
                int maxTet = Mathf.Min(maxTetExclusive, _data.NumTets);
                EnsureStampSize(_data.NumTets);

                for (int x = c0.x; x <= c1.x; x++)
                for (int y = c0.y; y <= c1.y; y++)
                for (int z = c0.z; z <= c1.z; z++)
                {
                    Vector3Int key = new Vector3Int(x, y, z);
                    if (!_cells.TryGetValue(key, out var list)) continue;
                    for (int i = 0; i < list.Count; i++)
                    {
                        int t = list[i];
                        if (t < 0 || t >= maxTet) continue;
                        if (_stamp[t] == _queryId) continue;
                        _stamp[t] = _queryId;
                        output.Add(t);
                    }
                }
            }

            void EnsureStampSize(int count)
            {
                while (_stamp.Count < count) _stamp.Add(0);
            }

            void AddBounds(Vector3 lo, Vector3 hi)
            {
                if (!_hasBounds)
                {
                    _bounds = new Bounds(0.5f * (lo + hi), hi - lo);
                    _hasBounds = true;
                    return;
                }
                _bounds.Encapsulate(lo);
                _bounds.Encapsulate(hi);
            }

            static Vector3Int ToCell(Vector3 p)
            {
                return new Vector3Int(
                    Mathf.FloorToInt(p.x / CELL),
                    Mathf.FloorToInt(p.y / CELL),
                    Mathf.FloorToInt(p.z / CELL));
            }

            static void GetTetAabb(TetMeshData data, int t, out Vector3 lo, out Vector3 hi)
            {
                Vector3 p0 = data.Positions[data.TetIds[4 * t + 0]];
                Vector3 p1 = data.Positions[data.TetIds[4 * t + 1]];
                Vector3 p2 = data.Positions[data.TetIds[4 * t + 2]];
                Vector3 p3 = data.Positions[data.TetIds[4 * t + 3]];
                lo = Vector3.Min(Vector3.Min(p0, p1), Vector3.Min(p2, p3));
                hi = Vector3.Max(Vector3.Max(p0, p1), Vector3.Max(p2, p3));
            }
        }
    }
}
