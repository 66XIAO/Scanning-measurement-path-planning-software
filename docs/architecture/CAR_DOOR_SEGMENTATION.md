# Car-door trimmed-surface segmentation

## Scope

This design handles large STEP surfaces whose XOY projection is suitable for a
regular grid but whose BRep face contains holes or other trimming loops.  It was
implemented on branch `CarDoor0808` against the confirmed model
`真实组件最终版.step`.

The legacy equal-parameter Boolean splitter remains the default for untrimmed
faces.  Automatic routing selects `mesh_grid` when at least one face contains
more than one wire.

## Why the old splitter is not used for this model

The confirmed model is a 65,965,998-byte STEP containing one large B-spline
face and 52 trimming wires.  OCC reports the face as `UnorientableShape`, so the
BRep is invalid even though it can be displayed and triangulated.  Repeated
iso-curve Boolean splitting rejected area-nonconserving operations and required
more than two minutes at U=8, V=6.

The CAD surface-property area is therefore retained only as an input diagnostic.
It is not treated as a reliable conservation reference for this invalid BRep.

## Strategy

`surface_segmentation.segment_shape_mesh_grid` performs the following steps:

1. Remove only cached display triangulations and remesh the original STEP face
   with bounded deflection.  Exact BRep curves and trimming wires are unchanged.
2. Clip every mesh triangle against the requested XOY grid cells.  This is
   important: assigning only a triangle centroid would not subdivide a large
   planar triangle into all requested cells.
3. Split the clipped fragments into connected components inside each grid cell.
   Material on opposite sides of a hole therefore cannot become one patch.
4. Select a representative mesh triangle for each connected component, then
   evaluate its centre and normal on the original CAD surface using the stored
   UV coordinates.
5. Store a lightweight `SurfacePatch` record containing source face, grid cell,
   connected-component index, area, centre, normal and bounds.  No temporary
   BRep face is created for every mesh fragment.

The partition must conserve the source triangulation area.  CAD-area divergence
is a separate input-quality warning when the BRep is invalid.

## Downstream contract

`state.current_faces` remains the legacy downstream collection and may now hold
either a `TopoDS_Face` or a `SurfacePatch`.  `state.surface_patches` explicitly
records the latter for inspection.

- Centre and normal generation reads the exact values from `SurfacePatch`.
- Viewpoint generation continues to consume parallel centre/normal lists.
- Patch OBBs are conservative XOY-aligned boxes reconstructed from fragment
  bounds.
- Rendering keeps the original surface visible and overlays coloured patch
  representatives.  Per-triangle coloured surface rendering is not implemented.

## Automatic routing and diagnostics

- Untrimmed face: `equal_param`.
- Face with more than one wire: `mesh_grid`.
- Equal-parameter area fallback is reported as partial segmentation.
- Invalid input BRep is reported as an input geometry warning, while a conserved
  mesh-grid partition may still complete.

Important mesh diagnostics are `source_mesh_area`, `patch_area_sum`,
`area_ratio`, `cad_mesh_area_ratio`, `input_brep_valid`, grid occupancy, connected
component splits and centre/normal validation.

## Validation and remaining acceptance

Automated validation covers an untrimmed box, a plate with a circular hole,
area conservation, connected regions, centre-on-face classification, unit
normals and patch OBB creation.  The confirmed real STEP regression is recorded
in `docs/history/CAR_DOOR0808_WORKLOG.md`.

Headless geometry validation is complete.  The remaining acceptance step is a
manual GUI check of the imported model, the 92 patch representatives produced
by the scanner-derived U=9, V=11 default, centre/viewpoint generation and
interaction responsiveness.  No robot or physical scanning validity is
implied by this segmentation change.
