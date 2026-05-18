using System.Collections.Generic;
using UnityEngine;
using SurgicalSim.Core;
using SurgicalSim.Cutting;
using SurgicalSim.Physics;

namespace SurgicalSim.CuttingV3
{
    public class CuttingToolV3 : MonoBehaviour
    {
        [Header("Cutting")]
        public float cutRadius = 0.05f;

        [Header("Contact")]
        public float contactRadius = 0.03f;

        [Header("Continuous Stroke")]
        [Tooltip("Reference length used to derive the path sampler minimum-move "
               + "threshold. Smaller values keep more samples (smoother but more "
               + "cut work); larger values drop low-velocity jitter. Default 0.012 "
               + "is fine for ~0.4 m organ models.")]
        public float maxCutSegmentLength = 0.012f;

        [Range(1, 8)]
        [Tooltip("Catmull-Rom subdivisions per ribbon segment emitted by the "
               + "path sampler. 1-2 keeps the cut responsive; raise only if "
               + "the tool sampling rate is visibly too coarse.")]
        public int maxCutSubsteps = 2;

        [Tooltip("Force a solver/surface rebuild when the blade leaves the mesh.")]
        public bool forceFlushOnExit = true;

        [Tooltip("Prevent tets created by this stroke from being cut again before the blade exits. This keeps one continuous stroke from repeatedly tessellating the same cut sheet.")]
        public bool protectNewTetsForStroke = true;

        [Header("Input")]
        public bool useSurgicalTool = true;

        [Header("Performance")]
        [Tooltip("Rebuild the visible surface while the blade is still inside tissue.")]
        public bool rebuildSurfaceDuringStroke = true;

        [Range(1, 10)]
        public int surfaceUpdateInterval = 1;

        [Tooltip("Allow expensive GPU/surface rebuilds while the blade is still inside tissue.")]
        public bool flushDuringStroke = true;

        [Range(1, 30)]
        public int flushInterval = 3;

        [Header("Cut Surface")]
        [Tooltip("Render zero-width cut faces from explicit split polygons instead of raw tet boundary fragments.")]
        public bool renderExplicitCutSurface = true;

        [Tooltip("Rebuild cut-surface rendering from current active tets so old cut patches cannot be dragged across separated components.")]
        public bool rebuildCutSurfaceFromActiveTopology = true;

        [Tooltip("Optional visual guard for long stale/sliver cut triangles. 0 disables filtering. Active-topology cut-surface rebuilds do not need this guard.")]
        public float maxRenderedCutSurfaceEdge = 0f;

        [Header("Debug")]
        public bool showDebugRay = true;

        TetSubdivisionCutter _cutter;
        TetMeshData _data;
        TetMeshVisualizer _visualizer;
        XPBDSolverGPU _solver;
        SurgicalTool _surgicalTool;
        CutSurfaceRenderer _cutSurfaceRenderer;
        Camera _cam;

        bool _isCutting;
        bool _wasInsideMesh;
        int _framesSinceFlush;
        int _framesSinceSurfaceUpdate;
        bool _forceFlushRequested;
        int _surfaceSyncedNewVerts;
        int _surfaceSyncedNewTets;
        int _surfaceSyncedCutTets;
        float _lastSurfaceUpdateMs;
        float _lastGpuFlushMs;

        BladePathSampler _sampler;
        readonly List<TetSubdivisionCutter.CutSurfacePatch> _cutPatchDrain =
            new List<TetSubdivisionCutter.CutSurfacePatch>(256);
        readonly List<int> _patchParticleIds = new List<int>(4);
        readonly List<int> _renderSurfaceTris = new List<int>(8192);
        readonly List<int> _renderCutTris = new List<int>(4096);

        LineRenderer _debugLine;

        public void Init(TetMeshData data, XPBDSolverGPU solver, TetMeshVisualizer visualizer)
        {
            _data = data;
            _solver = solver;
            _visualizer = visualizer;
            _cam = Camera.main;

            _cutter = new TetSubdivisionCutter();
            _cutter.Init(data, solver);
            _cutter.ProtectStrokeBornTets = protectNewTetsForStroke;

            // The sampler dedup threshold is derived from maxCutSegmentLength
            // so it still tracks the inspector value, but clamped into a safe
            // band: a too-large minMove starves the spline of samples and
            // gives a coarser cut; too small wastes work on jitter that the
            // surface reconstructor would never resolve anyway.
            float minMove = Mathf.Clamp(maxCutSegmentLength * 0.15f, 0.0005f, 0.005f);
            _sampler = new BladePathSampler(
                substeps: Mathf.Clamp(maxCutSubsteps, 1, 8),
                minMove: minMove);

            _framesSinceFlush = 0;
            _framesSinceSurfaceUpdate = 0;
            _forceFlushRequested = false;
            _wasInsideMesh = false;
            _isCutting = false;
            MarkSurfaceSynced();

            if (useSurgicalTool) EnsureSurgicalTool();
            if (renderExplicitCutSurface && !rebuildCutSurfaceFromActiveTopology)
                EnsureCutSurfaceRenderer(clearExisting: true);
            else
                ClearLegacyCutSurfaceRenderer();
            if (showDebugRay) SetupDebugLine();

            Debug.Log($"[CuttingToolV3] Init P:{data.NumParticles} T:{data.NumTets}");
        }

        void Update()
        {
            if (_data == null) return;
            if (useSurgicalTool) AutoCutStep();
            else MouseCutStep();

            if (renderExplicitCutSurface && !rebuildCutSurfaceFromActiveTopology && _cutSurfaceRenderer != null)
                _cutSurfaceRenderer.UpdatePositions(_data);
        }

        void AutoCutStep()
        {
            if (_surgicalTool == null)
            {
                EnsureSurgicalTool();
                return;
            }

            Transform tf = _visualizer != null ? _visualizer.transform : transform;
            Vector3 bladeA = tf.InverseTransformPoint(_surgicalTool.BladeA);
            Vector3 bladeB = tf.InverseTransformPoint(_surgicalTool.BladeB);

            UpdateDebugLine(_surgicalTool.BladeA, _surgicalTool.BladeB);

            bool insideMesh = IsBladeNearMesh(bladeA, bladeB);

            if (insideMesh && !_wasInsideMesh)
            {
                // Stroke start: clean cutter caches; the path sampler is
                // reset so the new stroke does not inherit a tangent from
                // the previous stroke's exit motion.
                _isCutting = true;
                _cutter.ResetStroke();
                _sampler.Reset();
            }

            if (insideMesh && _isCutting)
            {
                // Step 2 path smoothing: feed the latest in-tissue pose to
                // the sampler. It dedups against the previous sample, runs
                // Catmull-Rom over the four-sample window, and enqueues
                // smoothed ribbon quads. We then drain them through the
                // cutter in the order they were emitted.
                _sampler.Push(bladeA, bladeB);
                DrainSampler();
            }

            if (!insideMesh && _wasInsideMesh)
            {
                // Stroke end: flush the last linear segment so the trailing
                // portion of the path inside the tissue still gets cut even
                // though we never received a "post" sample after exit. The
                // exit pose itself is intentionally NOT pushed -- doing so
                // would extend the cut surface outside the mesh.
                _sampler.Flush();
                DrainSampler();
                _isCutting = false;
                if (forceFlushOnExit) _forceFlushRequested = true;
            }

            _wasInsideMesh = insideMesh;
        }

        void DrainSampler()
        {
            if (_cutter == null || _sampler == null) return;
            bool hadEvents = false;
            while (_sampler.TryDequeue(out var quad))
            {
                _cutter.ProtectStrokeBornTets = protectNewTetsForStroke;
                _cutter.Cut(quad.A0, quad.B0, quad.A1, quad.B1);
                hadEvents = true;
            }

            if (hadEvents) DrainCutSurfacePatches();
        }

        bool IsBladeNearMesh(Vector3 bladeA, Vector3 bladeB)
        {
            return _cutter != null && _cutter.IsBladeNearMesh(bladeA, bladeB, contactRadius);
        }

        void MouseCutStep()
        {
            // Kept as the old V3 placeholder. The active cutting path uses SurgicalTool.
            if (_cam == null) _cam = Camera.main;
        }

        public void FlushCutToGPU()
        {
            FlushCutToGPU(false);
        }

        void FlushCutToGPU(bool force)
        {
            if (_data == null || _solver == null || _cutter == null) return;
            if (!_cutter.IsDirty) return;

            _framesSinceFlush++;
            _framesSinceSurfaceUpdate++;

            bool topologyChanged = SurfaceTopologyChanged();
            bool shouldUpdateSurface =
                topologyChanged &&
                (force || _forceFlushRequested || !_isCutting ||
                 (rebuildSurfaceDuringStroke &&
                  _framesSinceSurfaceUpdate >= Mathf.Max(1, surfaceUpdateInterval)));

            if (shouldUpdateSurface)
            {
                UpdateSurface();
                MarkSurfaceSynced();
                _framesSinceSurfaceUpdate = 0;
            }

            bool shouldFlush = force || _forceFlushRequested || !_isCutting || flushDuringStroke;
            if (!shouldFlush) return;
            if (_isCutting && flushDuringStroke && _framesSinceFlush < Mathf.Max(1, flushInterval)) return;

            var sw = System.Diagnostics.Stopwatch.StartNew();
            _cutter.FlushToGPU();
            sw.Stop();
            _lastGpuFlushMs = (float)sw.Elapsed.TotalMilliseconds;

            if (SurfaceTopologyChanged())
            {
                UpdateSurface();
                MarkSurfaceSynced();
                _framesSinceSurfaceUpdate = 0;
            }
            _framesSinceFlush = 0;
            _forceFlushRequested = false;
        }

        public int TotalSplitVerts => _cutter?.TotalNewVerts ?? 0;
        public int TotalCutEdges => _cutter?.TotalNewTets ?? 0;
        public int TotalCutTets => _cutter?.TotalCutTets ?? 0;
        public bool ToolCutPressed => _isCutting;
        public float LastToolMoveDistance => _cutter?.LastMoveDistance ?? 0f;
        public int LastCandidateTetCount => _cutter?.LastCandidateTetCount ?? 0;
        public int LastIntersectedTetCount => _cutter?.LastIntersectedTetCount ?? 0;
        public string LastCutRejectReason => _cutter?.LastRejectReason ?? "no_cutter";
        public int LastSeparatedVertexCount => _cutter?.LastSeparatedVertexCount ?? 0;
        public int LastRemainingSharedVertexCount => _cutter?.LastRemainingSharedVertexCount ?? 0;
        public float LastCutQueryMs => _cutter?.LastQueryMs ?? 0f;
        public float LastCutSplitMs => _cutter?.LastSplitMs ?? 0f;
        public float LastCutSeparateMs => _cutter?.LastSeparateMs ?? 0f;
        public float LastSurfaceUpdateMs => _lastSurfaceUpdateMs;
        public float LastGpuFlushMs => _lastGpuFlushMs;

        void UpdateSurface()
        {
            if (_visualizer == null) return;
            var sw = System.Diagnostics.Stopwatch.StartNew();

            if (_cutter != null &&
                renderExplicitCutSurface &&
                rebuildCutSurfaceFromActiveTopology &&
                _cutter.CutSurfaceVertexIds.Count > 0)
            {
                SurfaceReconstructor.RebuildBoundarySurfaceSplitByCut(
                    _data,
                    _cutter.CutFaceRegistry,
                    _cutter.CutSurfaceVertexIds,
                    _cutter.CutSurfaceEdgeKeys,
                    _renderSurfaceTris,
                    _renderCutTris,
                    _cutter.CutSurfaceNormals);
                _visualizer.SetNormalMergeExcludedVertices(_cutter.CutSurfaceVertexIds);
                _visualizer.RebuildTopology(_renderSurfaceTris, _renderCutTris);
                ClearLegacyCutSurfaceRenderer();
            }
            else
            {
                int[] tris = SurfaceReconstructor.RebuildSurface(_data);
                _visualizer.SetNormalMergeExcludedVertices(null);
                _visualizer.RebuildTopology(tris);
                if (renderExplicitCutSurface && rebuildCutSurfaceFromActiveTopology)
                    ClearLegacyCutSurfaceRenderer();
                else
                    RebuildExplicitCutSurfaceFromTopology();
            }

            sw.Stop();
            _lastSurfaceUpdateMs = (float)sw.Elapsed.TotalMilliseconds;
        }

        bool SurfaceTopologyChanged()
        {
            if (_cutter == null) return false;
            return _surfaceSyncedNewVerts != _cutter.TotalNewVerts ||
                   _surfaceSyncedNewTets != _cutter.TotalNewTets ||
                   _surfaceSyncedCutTets != _cutter.TotalCutTets;
        }

        void MarkSurfaceSynced()
        {
            if (_cutter == null)
            {
                _surfaceSyncedNewVerts = 0;
                _surfaceSyncedNewTets = 0;
                _surfaceSyncedCutTets = 0;
                return;
            }

            _surfaceSyncedNewVerts = _cutter.TotalNewVerts;
            _surfaceSyncedNewTets = _cutter.TotalNewTets;
            _surfaceSyncedCutTets = _cutter.TotalCutTets;
        }

        void DrainCutSurfacePatches()
        {
            if (_cutter == null) return;

            _cutPatchDrain.Clear();
            _cutter.DrainCutSurfacePatches(_cutPatchDrain);
            if (_cutPatchDrain.Count == 0) return;

            if (rebuildCutSurfaceFromActiveTopology) return;
            if (!renderExplicitCutSurface) return;
            EnsureCutSurfaceRenderer();
            if (_cutSurfaceRenderer == null) return;

            for (int i = 0; i < _cutPatchDrain.Count; i++)
            {
                var patch = _cutPatchDrain[i];
                _patchParticleIds.Clear();
                for (int j = 0; j < patch.count; j++)
                {
                    int id = patch.IdAt(j);
                    if (id >= 0) _patchParticleIds.Add(id);
                }

                _cutSurfaceRenderer.AddParticlePatch(
                    _data,
                    _patchParticleIds,
                    patch.normal,
                    patch.reverseWinding,
                    rebuildMesh: false);
            }

            _cutSurfaceRenderer.RebuildNow();
        }

        void RebuildExplicitCutSurfaceFromTopology()
        {
            if (!renderExplicitCutSurface || !rebuildCutSurfaceFromActiveTopology) return;
            if (_cutter == null || _data == null) return;

            EnsureCutSurfaceRenderer();
            if (_cutSurfaceRenderer == null) return;

            if (_cutter.CutFaceRegistry.CutFaceCount == 0)
            {
                _cutSurfaceRenderer.Clear();
                return;
            }

            int[] cutTris = SurfaceReconstructor.RebuildCutBoundarySurface(
                _data,
                _cutter.CutFaceRegistry);
            _cutSurfaceRenderer.SetParticleTriangles(
                _data,
                cutTris,
                EffectiveCutSurfaceEdgeLimit());
        }

        void EnsureCutSurfaceRenderer(bool clearExisting = false)
        {
            if (!renderExplicitCutSurface)
            {
                _cutSurfaceRenderer = null;
                return;
            }

            Transform parent = _visualizer != null ? _visualizer.transform : transform;
            Transform existing = parent.Find("CutSurfaceV3");
            GameObject go;
            if (existing != null)
            {
                go = existing.gameObject;
            }
            else
            {
                go = new GameObject("CutSurfaceV3");
                go.transform.SetParent(parent, false);
            }

            go.SetActive(true);
            go.transform.localPosition = Vector3.zero;
            go.transform.localRotation = Quaternion.identity;
            go.transform.localScale = Vector3.one;

            _cutSurfaceRenderer = go.GetComponent<CutSurfaceRenderer>();
            if (_cutSurfaceRenderer == null)
                _cutSurfaceRenderer = go.AddComponent<CutSurfaceRenderer>();

            _cutSurfaceRenderer.maxRuntimeEdgeLength = EffectiveCutSurfaceEdgeLimit();

            if (clearExisting)
                _cutSurfaceRenderer.Clear();
        }

        void ClearLegacyCutSurfaceRenderer()
        {
            if (_cutSurfaceRenderer != null)
            {
                _cutSurfaceRenderer.Clear();
                _cutSurfaceRenderer.gameObject.SetActive(false);
                _cutSurfaceRenderer = null;
                return;
            }

            Transform parent = _visualizer != null ? _visualizer.transform : transform;
            Transform existing = parent != null ? parent.Find("CutSurfaceV3") : null;
            if (existing == null) return;
            var renderer = existing.GetComponent<CutSurfaceRenderer>();
            if (renderer != null) renderer.Clear();
            existing.gameObject.SetActive(false);
        }

        float EffectiveCutSurfaceEdgeLimit()
        {
            if (rebuildCutSurfaceFromActiveTopology) return 0f;
            if (maxRenderedCutSurfaceEdge <= 0f) return 0f;
            return maxRenderedCutSurfaceEdge;
        }

        void EnsureSurgicalTool()
        {
            if (_surgicalTool != null) return;
            _surgicalTool = GetComponent<SurgicalTool>() ?? gameObject.AddComponent<SurgicalTool>();
        }

        void SetupDebugLine()
        {
            if (_debugLine != null) return;

            _debugLine = GetComponent<LineRenderer>();
            if (_debugLine == null) _debugLine = gameObject.AddComponent<LineRenderer>();

            _debugLine.startWidth = 0.003f;
            _debugLine.endWidth = 0.001f;
            _debugLine.material = new Material(Shader.Find("Sprites/Default"));
            _debugLine.startColor = Color.cyan;
            _debugLine.endColor = Color.yellow;
            _debugLine.positionCount = 2;
            _debugLine.enabled = false;
        }

        void UpdateDebugLine(Vector3 a, Vector3 b)
        {
            if (!showDebugRay || _debugLine == null) return;
            _debugLine.enabled = true;
            _debugLine.SetPosition(0, a);
            _debugLine.SetPosition(1, b);
        }

        void OnDestroy()
        {
            if (_debugLine != null && _debugLine.material != null) Destroy(_debugLine.material);
        }
    }
}
