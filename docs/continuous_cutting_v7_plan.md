# Continuous Cutting V7 Plan

> Audit note, 2026-05-17: V7 is directionally correct, but it is not strict enough for the updated ten requirements. The main gaps are recutting residual attachments, preserving thin detached pieces, and making contact detection non-rejecting at the cutting layer. The V8 replacement plan appended at the end is the plan that should drive the next implementation pass.

## Goal

V7 replaces the current ad-hoc continuous topology layer in `Assets/SurgicalSim/CuttingV3` with a cut-sheet driven topology pipeline.

The single-tetrahedron subdivision templates stay. The change is how a continuous blade path is represented, how the new cut faces are registered, how shared vertices are split, and how tiny numerical fragments are prevented without deleting tetrahedra.

The target behavior is:

- zero-width cutting;
- no tetrahedron deletion to create a gap;
- smooth visual cut surfaces;
- arbitrary blade trajectories, including straight, S-shaped, and right-angle cuts;
- no point-only or edge-only residual bridges after a through cut;
- no debris cloud from numerical sliver components;
- local work only, so cutting does not collapse to very low FPS.

## Why V6 Is Not Enough

The current V6 code has the right single-tet subdivision direction, but it still mixes several responsibilities:

- the blade path is processed as many local plane events, so a continuous cut surface can become a sequence of weakly related patches;
- the cut surface is partly explicit and partly inferred from boundary faces;
- vertex separation currently uses one local criterion around touched vertices;
- numerical slivers can be detached as real tissue components;
- rendering still depends on face extraction after topology mutation instead of a persistent face class.

The latest face-only separator made the main failure clearer: requiring full-face connectivity removes more residual bridges, but it also turns many local sliver fans into separate free components. That creates debris and rough cut edges. The correct solution is not "always face-only"; it is "face connectivity after removing only the faces blocked by the cut sheet, followed by fragment control."

## Research Basis

V7 uses the following ideas from the literature and adapts them to this Unity XPBD codebase:

- Virtual Node Algorithm and Adaptive VNA: duplicate nodes by material component, not by deleting volume. Adaptive VNA also treats intersections on tet vertices, edges, and faces as first-class cases rather than perturbing them away.
- State-machine tetrahedral cutting: each tetrahedron must move through consistent cut states so neighbouring tets do not disagree about topology.
- 2024 graph-based dissection / AFCC: complex cuts need connected-component reasoning and small-component control to avoid ghost forces and fragment artifacts.
- 2025 graph coloring XPBD: dynamic topology needs local graph/scheduler updates for performance; correctness should stay local so later GPU recoloring is possible.
- SOFA cutting / mesh refinement: affected tetrahedra and their neighbours must be updated coherently, not as independent isolated cuts.

## Requirements Mapping

1. Smooth cut surface: cut faces are rendered from explicit cut-sheet patches only. Main surface extraction never guesses a cut face from "all cut vertices" or normal similarity.
2. Arbitrary trajectory: the blade path becomes an ordered cut sheet made from bounded ribbon patches. Curves are integrated from short patches; right-angle turns create crease patches.
3. No lotus-root bridge: after splitting, material components are detected in a barrier-aware tet graph. A shared vertex is cloned when its incident tets belong to multiple material components.
4. Performance: all graph and surface work is restricted to the dirty region around touched tets and their local neighbours.
5. 2025 graph coloring compatibility: every topology mutation records touched constraints so later recoloring can be local.
6. High FPS target: do not rebuild global topology/render surfaces on every sample; batch and localize.
7. Zero-width cutting: generated positive and negative vertices share the same position; no cut-gap tet removal.
8. Single-tet split remains: V7 wraps the existing Case1/Case2 templates instead of replacing them.
9. Touch means cut: contact detection may filter candidates for performance, but the cutting layer must not reject a valid local intersection. Degenerate cases are snapped/classified, not skipped as a whole stroke.

## Plan Review Against The Nine Requirements

This plan satisfies the nine requirements under the following non-negotiable rules:

- Requirement 1 is satisfied only if cut-surface rendering is driven by explicit cut faces. A face may not become pink/black because of backface direction, normal similarity, or "all vertices are generated".
- Requirements 2 and 8 are satisfied by keeping the existing single-tet split templates and feeding them an ordered cut sheet. Straight cuts are one stable patch sequence; S-curves are integrated short patches; right-angle turns create new patch/corner ids.
- Requirement 3 is satisfied only by barrier-aware connected components. Raw face-only connectivity is too aggressive and creates debris; raw edge-aware connectivity is too weak and can leave bridges. V7 must use cut barriers, side labels, and local component analysis together.
- Requirements 4 and 6 require dirty-region processing. Any V7 step that loops the whole mesh every blade sample is a temporary diagnostic path, not the final algorithm.
- Requirement 5 is used as the solver/performance layer: topology changes must record touched constraints so graph coloring can be updated locally later.
- Requirement 7 forbids deleting tetrahedra to make a visible gap. Fragment control may attach or merge numerical slivers into a neighbouring material component, but must not remove tet volume as the cutting operation.
- Requirement 9 means broad-phase acceleration is allowed only to find candidates. Once a tet is geometrically intersected by the blade cut sheet, the cutting layer must process it or classify a precise degeneracy; it may not reject the cut for visual quality, performance, or arbitrary thresholds.

## V7 Pipeline

### 1. Blade Path To Cut Sheet

Create a stroke-level `CutSheet` object.

Each blade sample adds an ordered `CutPatch`:

- endpoints: previous blade segment and current blade segment;
- local blade direction;
- movement direction;
- patch normal;
- patch id;
- corner id for high-angle turns;
- a bounded AABB for candidate lookup.

Patch rules:

- straight motion keeps one stable patch plane;
- S-curves create many short ordered patches;
- a right-angle turn ends the current patch and starts a new crease patch;
- intersection caches are scoped to `(stroke id, patch id, original edge key)`.

The cut sheet is the only authority for continuous geometry. Individual tets should not invent their own unrelated cutting plane.

### 2. Candidate Tet Detection

Use the existing spatial tet index, but tighten semantics:

- candidate set = tets whose current AABB overlaps the bounded cut patch AABB;
- then test tet against the swept ribbon footprint;
- if the ribbon intersects the tet, the tet must be processed.

The cut layer must not reject based on visual heuristics, side history, or performance shortcuts. Only already-processed stroke rules can skip recutting a child tet, and that skip is valid only when the tet was already processed by the same patch id.

### 3. Robust Local Intersection

For every intersected tet:

- classify each vertex against the patch plane;
- if the plane passes close to a vertex, edge, or face, snap to that feature and record a degeneracy event;
- do not create near-zero sliver child tets just to avoid a degeneracy;
- reuse the same intersection pair for the same original edge within a patch.

This is the part closest to Adaptive VNA. The goal is to avoid both missed cuts and sliver debris.

### 4. Existing Single-Tet Split

Keep the existing Case1/Case2 templates:

- Case1 emits a triangle cut patch;
- Case2 emits a quad cut patch triangulated consistently;
- generated vertices are paired positive/negative at identical positions;
- every emitted cut face/edge gets a persistent `CutFaceId` / `CutEdgeId`.

No tet deletion is used to open a gap.

### 5. Same-Stroke Recut Policy

The current option `protectNewTetsForStroke` is conceptually right but too blunt.

V7 policy:

- a tet created by the current patch is not recut by the same patch;
- it can be recut by a later patch only if the later patch has a different patch id or corner id and the geometry actually intersects it;
- for a straight cut, this prevents repeated cutting and roughness;
- for an S-curve or right-angle turn, it still allows the path to continue through already-generated child tets when the blade direction meaningfully changes.

### 6. Barrier-Aware Connected Components

Replace the current `SharedVertexSeparator` criterion with a barrier-aware graph.

For each touched vertex `v`:

1. collect active incident tets in the dirty region;
2. build adjacency between incident tets only when:
   - they share a full face containing `v`;
   - the shared face is not a registered cut face;
   - the shared face is not intersected by the current cut sheet barrier;
   - their patch-side labels do not conflict;
3. find connected components;
4. clone `v` for every material component except the kept component;
5. rewrite only those tets to the clone.

This removes true topological bridges. It is different from the current face-only separator because it uses cut barriers and side labels, not just raw face sharing.

### 7. Fragment Control Without Tet Deletion

After component splitting, run a local fragment pass over dirty components.

A component is considered numerical debris if all are true:

- tet count or rest volume is below a small threshold;
- it was created in the current stroke/patch;
- it has no stable original surface anchor or only a tiny anchor;
- its cut boundary is not a closed meaningful cut boundary;
- it is adjacent to a larger component through non-cut boundary geometry.

Numerical debris is not deleted. Instead it is merged/attached topologically by choosing the larger neighbouring material component as its owner during vertex clone assignment. This keeps zero-width cutting and prevents loose sliver pieces.

Real small tissue pieces are preserved when they have a meaningful closed cut boundary or original surface support.

### 8. FaceClass Rendering

Every visible boundary face gets exactly one class:

- `OriginalSurface`: supported by an original surface triangle and not hidden by a cut;
- `CutSurface`: explicitly emitted by the split templates and present in the cut face registry;
- `HiddenInternal`: everything else.

Rendering rules:

- main liver mesh renders only `OriginalSurface`;
- cut surface mesh renders only `CutSurface`;
- no shader-side inference is allowed to decide whether a face is cut tissue;
- no "all generated vertices" heuristic is allowed for cut-surface classification when explicit face ids exist.

This is the main fix for wrong colors and flickering black/pink triangles.

### 9. Solver And Performance

Keep the first V7 implementation CPU-local, then optimize:

- dirty tet set = cut parents, new children, one-ring neighbours, and incident tets of split vertices;
- rebuild visible surfaces only for dirty face regions where possible;
- batch GPU solver rebuilds during a stroke;
- later: preallocate GPU buffers, append new particles/tets, and recolor changed XPBD constraints locally.

The 2025 graph-coloring paper is most relevant here: the topology algorithm should record the changed constraint graph neighbourhood so dynamic recoloring can be added without redesigning cutting.

## Implementation Phases

### Phase 0: Stabilize Current V3

Purpose: stop the latest regression before deeper refactor.

- Change `SharedVertexSeparator` back from raw full-face-only connectivity to the previous less destructive edge-aware local criterion.
- Keep the explicit cut-face renderer fix.
- Keep main liver shader surface-colored; cut color belongs to `CutSurfaceRenderer`.
- Add diagnostics for separated component count, small component volume, and emitted cut face count.

### Phase 1: Face Registry Cleanup

- Implemented: introduced `CutFaceRegistry` with explicit face and edge ids.
- Implemented: `SurfaceReconstructor` can now consume the registry for original-surface exclusion and explicit cut-surface extraction.
- Implemented: `CuttingToolV3` uses the explicit registry path for topology-derived cut-surface rendering instead of vertex/normal inference.

### Phase 2: CutSheet Representation

- Add `CutSheet`, `CutPatch`, and `CutPatchClassifier`.
- Move path plane locking out of `TetSubdivisionCutter` into the cut-sheet layer.
- Store patch/corner ids for right-angle and S-curve paths.

### Phase 3: Robust Tet Intersection

- Centralize tet-patch intersection into one module.
- Classify vertex/edge/face degeneracies.
- Reuse edge intersections per `(stroke, patch, original edge)`.
- Avoid sliver tet creation where snapping can represent the same zero-width topology.

### Phase 4: Barrier-Aware Vertex Split

- Implemented first pass: `SharedVertexSeparator` now builds local components with registered cut faces and cut edges as barriers.
- Remaining: promote this into a dedicated `BarrierComponentSeparator` with explicit cut-sheet side labels and a residual bridge detector.

### Phase 5: Fragment Control

- Add dirty-region component measurement.
- Attach numerical debris to the nearest larger component during clone assignment.
- Preserve real small pieces with closed cut boundaries.

### Phase 6: Performance Pass

- Local surface rebuild.
- Batched GPU flush during stroke.
- Optional local XPBD graph recoloring.

## Success Tests

1. Straight through cut:
   - one smooth cut surface;
   - no black/pink wrong surface patches;
   - no debris;
   - no residual point bridge.

2. S-curve cut:
   - continuous curved cut surface;
   - no repeated same-patch recuts;
   - no dangling thread.

3. Right-angle turn:
   - visible crease at the corner;
   - no diagonal shortcut plane;
   - no debris cloud at the corner.

4. Thin lobe cut:
   - small real piece remains coherent;
   - numerical slivers do not explode as free pieces.

5. Performance:
   - candidate count and dirty graph size stay local;
   - cutting frame should not collapse to the 5-15 FPS range on the current liver test scene.

## V8 Audit Result And Replacement Plan

This section supersedes the V7 implementation phases above. V7 should be treated as a stabilization checkpoint, not the final design.

### Audit verdict

| Requirement | V7 status | Required V8 correction |
| --- | --- | --- |
| Smooth cut surface | Partially satisfies | Make the cut sheet the only source of cut-face geometry; remove remaining heuristic rendering and stale patch paths |
| Arbitrary tool trajectory | Partially satisfies | Add explicit `CutSheet`, `CutPatch`, `cornerId`, and ordered patch processing instead of ad-hoc plane stabilization |
| No residual bridge | Partially satisfies | Add barrier-aware material components plus a residual bridge detector after every cut batch |
| No severe frame drop | Partially satisfies | Make all detection, splitting, component analysis, surface rebuild, and solver recoloring dirty-region based |
| Compatible with 2025 graph coloring | Partially satisfies | Record touched constraints and recolor only changed XPBD graph neighborhoods |
| Low computation cost | Partially satisfies | Use spatial index, local patch cache, batched GPU flush, and local render mesh updates |
| Zero-width cut, no tet deletion | Satisfies as a rule | Keep positive/negative duplicate vertices at identical positions; fragment handling may attach/merge ownership but must not delete cut volume |
| Keep single-tet split templates | Satisfies as a rule | Continue to use the current Case1/Case2 subdivision templates, but feed them better continuous topology state |
| Tool contact must cut | Not strict enough | Candidate filters may accelerate only; if a tet intersects the cut sheet, the cut layer must process it or record a precise degeneracy |
| Thin/small detached piece keeps shape | Not sufficiently covered | Add component-aware mass, constraints, damping, and sliver ownership so thin real pieces remain coherent |

### Research synthesis

The final design should combine six ideas from the literature and open-source ecosystem:

- Adaptive VNA: represent topology changes by duplicating material nodes/components, and handle vertex, edge, and face degeneracies directly instead of perturbing the cut away from difficult cases.[^avna]
- State-machine tetrahedral cutting: each tetrahedron needs a persistent cut state so progressive and reverse/intersecting tool motion does not create inconsistent neighboring topology.[^state]
- Efficient topological operations: real-time surgical cutting depends on limiting new nodes/elements and maintaining FEM mesh quality, not just producing a visual cut.[^paulus]
- Graph-based dissection: connected-component analysis and component merging are necessary to remove ghost forces and fragment artifacts during progressive cuts.[^graph2024]
- XPBD graph coloring: dynamic cutting needs local graph updates/recoloring because precomputed schedules become stale when topology evolves.[^pg2025]
- SOFA-style coherent refinement: affected tetrahedra and their adjacent topology must be updated together, and new cut surfaces should be mapped explicitly for rendering/materials.[^sofa]

### V8 architecture

```mermaid
flowchart TD
    accTitle: V8 continuous cutting pipeline
    accDescr: The cut tool generates an ordered cut sheet. Local tet subdivision emits explicit cut faces. Barrier-aware material components then split shared vertices, preserve thin pieces, and update rendering and solver data locally.

    tool["Tool contact samples"]
    sheet["CutSheet stroke state"]
    patch["Ordered CutPatch queue"]
    detect["Dirty-region tet detection"]
    intersect["Robust tet-patch intersection"]
    split["Existing Case1/Case2 split"]
    registry["CutFaceRegistry and CutEdgeRegistry"]
    graph["Barrier-aware component graph"]
    bridge["Residual bridge detector"]
    thin["Thin-piece stabilizer"]
    render["FaceClass renderer"]
    solver["Local XPBD graph update"]

    tool --> sheet --> patch --> detect --> intersect --> split
    split --> registry
    split --> graph
    registry --> graph
    graph --> bridge --> thin
    registry --> render
    thin --> solver
```

### Non-negotiable invariants

1. `CutSurface` faces are emitted only by subdivision templates or by clone-preserving registry propagation.
2. `OriginalSurface` faces must be supported by original surface triangles and must not be registered cut faces.
3. No rendering path may infer cut faces from "all generated vertices", normal similarity, backface direction, or shader-side VFACE.
4. A tet intersected by a valid cut patch cannot be skipped for visual quality or performance reasons.
5. The same patch can avoid duplicate work only through a per-tet/per-patch ledger, not by blocking later patches or a second tool pass.
6. Zero-width cutting means no deletion of tetrahedra to open the wound. Attachment/ownership changes are allowed only for numerical slivers.
7. A thin detached piece is real if it has a closed cut boundary or meaningful original-surface support; it must keep mass, constraints, and coherent rendering.
8. A numerical sliver is not real tissue if it has tiny volume, no closed boundary, no stable surface support, and was born in the current patch. It should be attached to a neighboring material component, not dropped as debris.

### Cut sheet and trajectory handling

V8 should replace the current plane-locking logic with an explicit stroke object:

- `CutStroke`: stroke id, entry/exit state, ordered samples, dirty bounds
- `CutPatch`: patch id, corner id, blade segment before/after, plane/ribbon frame, finite swept footprint, AABB
- `CutFeatureCache`: `(strokeId, patchId, originalEdgeKey)` to stable intersection vertex pair
- `TetPatchLedger`: `(tetId, strokeId, patchId)` records processed, split, degenerate, or no-crossing result

Trajectory rules:

- Straight cuts reuse one stable patch frame and produce a smooth plane.
- S-cuts are a sequence of short patches with C1-continuous tangents.
- Right-angle turns close the previous patch and open a new `cornerId`; the corner is represented as a crease, not a diagonal shortcut.
- If the tool later touches a residual attachment, it creates a new stroke or new patch id and must be allowed to cut that attachment.

### Robust local intersection

Every candidate tet should be classified against the patch with a feature-aware routine:

1. Test current tet AABB against patch AABB.
2. Test tet edges/faces against the finite swept ribbon.
3. Classify vertex signs with tolerance.
4. If a vertex/edge/face degeneracy is detected, snap and record the feature class.
5. Reuse the same split pair for the same original edge within a patch.
6. Never turn a valid contact into `no_intersection` because the local plane is inconvenient.

This directly addresses the current `cand > 0, hit = 0` failure mode: broad-phase candidates are not enough, but the narrow-phase must explain each skipped tet with an exact state.

### Single-tet split contract

The current Case1/Case2 templates can stay, but their output contract must be stricter:

- emit child tets with nonzero rest volume or roll back the parent;
- emit exact cut faces and cut edges into `CutFaceRegistry`;
- tag every child with `strokeId`, `patchId`, `sideLabel`, and parent tet id;
- copy registry face keys when vertex clones replace old vertices;
- do not create extra visual-only cut patches that are not backed by active tet boundary faces.

### Barrier-aware material separation

After a batch of splits, V8 must compute material components in the dirty region:

1. Collect cut parents, new children, one-ring neighbours, and incident tets of cloned vertices.
2. Build a tet graph where adjacency exists only if the shared face/edge is not a registered cut barrier and the side labels are compatible.
3. For every shared vertex, find incident tet components.
4. Clone the vertex for every real material component.
5. Run a residual bridge detector:
   - no shared vertex between opposite sides unless connected through non-cut material;
   - no shared edge crossing a registered cut barrier;
   - no same-particle XPBD constraint between separated components.
6. If a bridge remains, enqueue a local re-separation pass or mark it as a required recut target for the next tool contact.

This is the part that prevents lotus-root attachments while still allowing the second knife pass to cut anything that remains connected.

### Thin-piece shape preservation

The tenth requirement needs a dedicated pass. A thin detached component should not explode simply because it has few tets.

V8 should add a `CutComponentStabilizer`:

- compute component rest volume, current volume, centroid, velocity, original-surface anchor area, and cut-boundary closure;
- preserve real thin pieces by keeping their XPBD constraints active, inheriting parent damping, and limiting only excessive relative velocity after topology mutation;
- attach numerical slivers during vertex clone assignment when they are tiny, newly born, not closed, and not surface-supported;
- add temporary shape-preserving constraints for very thin real pieces for a short settling window after separation;
- cap impulse/velocity of new cut vertices, but do not pin them or visually freeze them;
- do not merge thin pieces that have meaningful closed cut boundaries or original-surface support.

Expected effect: a thin slice cut from the liver behaves like a soft coherent flap/piece, not a particle cloud or a collapsed ribbon.

### Rendering model

Rendering must be a pure readout of topology classes:

- main liver mesh: only `OriginalSurface`
- cut surface mesh: only `CutSurface`
- debug mesh: optional `HiddenInternal` and component ids

Implementation details:

- `CutFaceRegistry` should store face class, patch id, stroke id, side, and parent tet id.
- `SurfaceReconstructor` should rebuild only dirty face regions after a cut batch.
- `CutSurfaceRenderer` should use active boundary faces from the registry, not stale patch polygons.
- The shader should not decide whether a face is interior tissue; it only shades the mesh it is given.

### Performance plan

The performance target should be local updates, not full rebuilds:

- use spatial AABB grid/BVH for candidate tets;
- process patches in batches during a stroke;
- rebuild only dirty render faces when possible;
- update only changed XPBD constraints;
- maintain a local graph-coloring dirty set for the 2025 XPBD solver path;
- keep fallback global rebuild only as a diagnostic path.

### Implementation phases

1. `V8-0 Diagnostics`: add counters for candidate skips, exact skip reasons, registry face count, bridge count, thin component count, and sliver attachments.
2. `V8-1 CutSheet`: implement `CutStroke`, `CutPatch`, `CutFeatureCache`, and `TetPatchLedger`.
3. `V8-2 RobustIntersection`: centralize tet-patch intersection and degeneracy handling.
4. `V8-3 SplitContract`: make Case1/Case2 emit complete registry metadata and side labels.
5. `V8-4 BarrierComponents`: replace `SharedVertexSeparator` with a dirty-region component separator plus residual bridge detector.
6. `V8-5 ThinPieceStabilizer`: preserve real thin pieces and attach numerical slivers without deleting tet volume.
7. `V8-6 Rendering`: remove stale patch rendering and rebuild from `FaceClass` only.
8. `V8-7 Performance`: local render updates, batched GPU flush, local XPBD graph recoloring.

### Acceptance tests

| Test | Pass condition |
| --- | --- |
| Straight through cut | One smooth cut plane, no spikes, no wrong-color faces, no residual bridge |
| S-shaped cut | Continuous curved cut surface, no repeated patch shredding, no debris trail |
| Right-angle cut | A clean crease at the turn, no diagonal shortcut plane, no fragment burst |
| Second pass on residual attachment | The tool can cut the remaining connected part when it touches it again |
| Thin slice | The small/thin piece remains a coherent soft body component and does not explode |
| Zero-width validation | Both sides separate topologically while coincident cut vertices keep zero visual gap |
| Performance | Dirty-region processing keeps cut frames near interactive rate; global rebuild appears only in diagnostics |

[^avna]: Wang, Jiang, Schroeder, and Teran, "An Adaptive Virtual Node Algorithm with Robust Mesh Cutting", 2014. https://diglib.eg.org/items/3d5efbc1-3328-48d4-b267-71d9d2f4337d
[^state]: Bielser, Glardon, Teschner, and Gross, "A State Machine for Real-Time Cutting of Tetrahedral Meshes", 2003. https://cg.informatik.uni-freiburg.de/publications/2003_PG_cuttingStateMachine.pdf
[^paulus]: Paulus et al., "Virtual cutting of deformable objects based on efficient topological operations", 2015. https://doi.org/10.1007/s00371-015-1123-x
[^graph2024]: Yu et al., "Real-time soft body dissection simulation with parallelized graph-based shape matching on GPU", 2024. https://doi.org/10.1016/j.cmpb.2024.108171
[^pg2025]: Yu et al., "Parallel Constraint Graph Partitioning and Coloring for Realtime Soft-Body Cutting", 2025. https://diglib.eg.org/items/8d122d91-a90e-4007-80a5-fd42c5e5023c
[^sofa]: SOFA, "Cutting & Mesh Refinement". https://www.sofa-framework.org/applications/plugins/cutting-mesh-refinement/
