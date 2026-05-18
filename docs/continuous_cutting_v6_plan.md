# Continuous Cutting V6 Plan

## Goal

V6 keeps the physical cut zero-width: no tetrahedron is deleted to create a gap. A touched tet is split by the local blade plane, duplicated vertices are used to separate the two material sides, and the visible cut surface is rendered from explicit cut polygons instead of whatever boundary triangles fall out of the tetrahedral subdivision.

This plan is written for `Assets/SurgicalSim/CuttingV3` and preserves the current single-tet split templates. The main change is the continuous topology layer around those templates.

## Requirements Mapping

1. Smooth cut surface: every intersected tet contributes one planar triangle or quad patch. The renderer uses these explicit patches as the cut face and hides raw all-generated boundary faces from the outer surface mesh.
2. Arbitrary tool trajectory: the blade path is converted into ordered bounded ribbon events. Straight cuts stay on one patch plane; curved cuts are the integral of many local straight ribbon events.
3. No point bridges: after every local split, touched one-rings are split by connected components around each pivot vertex. Tets that only share the pivot are no longer considered connected.
4. Performance: candidate tets come from the spatial index and bounded ribbon intersection, not an infinite cutting plane. GPU/surface refresh is batched.
5. 2025 graph-coloring reference: the split remains graph-friendly because it only creates local child tets and explicit side groups; solver coloring can be rebuilt after topology flush.
6. High FPS target: the expensive full solver rebuild remains batched; the next performance step is incremental solver buffer growth, not changing the topology algorithm.
7. Zero-width cut: no tet removal, no gap creation. Each cut intersection creates coincident positive/negative vertices at the same position.
8. Single-tet split is reused: the continuous logic decides which tets to split and how to keep adjacent child tets topologically separated.
9. Touch means cut: V3 should not reject real contacts because of coarse global logic. Only exact local degeneracy handling remains, and that should be reduced over time by robust remeshing/snap rules instead of skipping whole strokes.

## Data Flow

1. `CuttingToolV3` samples the tool pose.
2. `BladePathSampler` emits ordered ribbon events `(A0, B0, A1, B1)`.
3. `TetSubdivisionCutter.Cut` computes one bounded local cut plane from the ribbon event.
4. The spatial index returns candidate tets overlapping the swept blade AABB.
5. Each candidate tet is tested against the ribbon footprint, not just an infinite plane.
6. Intersected tets are split by the existing Case1/Case2 single-tet templates.
7. Each successful split queues explicit cut-surface patches:
   - Case1: two coincident triangle patches, one per material side.
   - Case2: two coincident quad patches, one per material side.
8. `SharedVertexSeparator` runs local connected-component separation around touched vertices.
9. `CuttingToolV3` drains queued cut patches into `CutSurfaceRenderer`.
10. `SurfaceReconstructor` rebuilds the ordinary outer surface while excluding raw faces made entirely from generated cut vertices.

## Straight, S-Curve, and Right-Angle Cuts

### Straight Cut

A straight stroke should produce one stable plane. V3 keeps a patch plane locked while:

- the cut normal changes less than `PATCH_MAX_NORMAL_ANGLE_DEG`;
- the movement direction changes less than `PATCH_MAX_MOVE_ANGLE_DEG`;
- the plane offset stays within `PATCH_MAX_OFFSET`.

This prevents per-frame plane jitter from creating raised triangles on the visual cut surface.

### S-Curve

An S path is not one global curved boolean operation. It is processed as many ordered local straight cuts:

- each ribbon event cuts only the tets it geometrically overlaps;
- patch-plane locking only survives while the motion remains locally straight;
- when the tool curvature exceeds the patch threshold, a new patch id starts, so original-edge intersection caches do not leak across different curve sections.

The result approximates the S curve by integrating local tet cuts along the path.

### Right-Angle Turn

A right-angle turn must not be blended into one diagonal plane. The correct behavior is ordered event splitting:

1. process the pre-corner ribbon patch;
2. flush its generated child tets into adjacency/spatial data;
3. process the post-corner ribbon patch on the new child tets;
4. let the connected-component separator duplicate any corner pivot that would otherwise become a point bridge.

The important rule is: a turn inside one original tet is handled by cutting the generated child tets on the second event, not by trying to create a single multi-plane split template immediately.

## Smooth Cut Surface Strategy

The physical tetrahedral mesh after subdivision contains many small boundary faces. Those faces are valid for topology, but they are not a good visual cut surface; they show local tetrahedral tessellation and can look like spikes.

V6 separates physical topology from cut-surface rendering:

- physical mesh: split tets and duplicated vertices;
- ordinary visual surface: boundary extraction excluding faces whose three vertices are generated cut vertices;
- cut visual surface: explicit triangle/quad patches emitted at the exact split plane.

This keeps the cut zero-width while making the displayed cut face smooth and planar per local event.

## Point-Bridge Removal

The local criterion is:

Two active tets around pivot vertex `v` are in the same material component only if they share at least one additional vertex besides `v`.

If the active one-ring around `v` has more than one component:

- keep the largest component on `v`;
- clone `v` for every other component;
- rewrite those component tets from `v` to the clone.

This removes point-only bridges even when a continuous stroke creates more than two local components.

## Performance Plan

Current V3 still pays for expensive work:

- full visible surface rebuild;
- full GPU solver dispose/init during topology flush.

The correctness-first V6 step keeps those batched and reduces unnecessary split work. The next performance step should be:

1. grow GPU buffers with spare capacity;
2. upload only appended particles/tets and active flags;
3. recolor only changed graph neighborhoods or recolor batched topology chunks;
4. rebuild only local surface patches instead of all boundary faces.

That is a separate optimization layer. It should not change the zero-width topology algorithm.

## Immediate Implementation Scope

This repo update implements:

- explicit cut-surface patch records in `TetSubdivisionCutter`;
- queued patch commit only after the parent split produced valid child tets;
- `CuttingToolV3` child `CutSurfaceRenderer` creation and patch draining;
- main surface rebuild that excludes raw all-generated cut faces;
- the existing connected-component `SharedVertexSeparator` as the point-bridge removal stage.

Not implemented in this immediate patch:

- incremental GPU buffer growth;
- local-only surface reconstructor;
- multi-plane split template for one ribbon event. Right-angle cuts use ordered child-tet recutting instead.
