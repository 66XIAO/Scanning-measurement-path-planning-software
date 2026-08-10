# software3_16_En_optimized.py optimization notes

## Scope

The optimized file keeps the original menu-driven workflow and pythonOCC/PyQt5 dependencies, while applying low-risk improvements to architecture, function behavior, method reuse, and UI state visibility.

Source file:

- `software3_16_En.py`

Optimized file:

- `Software_optimizing/software3_16_En_optimized.py`

## Architecture changes

- Added shared constants for viewpoint distance, candidate count, and zenith angle:
  - `VIEWPOINT_DISTANCE`
  - `NUM_CANDIDATES_PER_FACE`
  - `ZENITH_ANGLE_DEG`
- Added a lightweight `WorkflowSnapshot` dataclass to collect GUI-visible state.
- Added reusable path utilities:
  - `point_distance`
  - `build_distance_matrix`
  - `calculate_path_length`
  - `solve_greedy_open_path`
  - `draw_path_edges`
- Reduced repeated path-planning code in sequential, greedy, and default planning functions.
- Invalidated downstream state after re-segmentation, so stale centers/viewpoints/collision/path data do not remain attached to an older segmentation.

## Functional fixes

- Fixed duplicate CSV row writing in optimized path export.
- Fixed fallback CSV export to use the current `(viewpoint, pose)` storage format.
- Changed path-planning confirmation behavior: after the user confirms, the selected planning algorithm executes immediately instead of requiring a second menu click.
- Reordered candidate viewpoint storage so all center viewpoints are stored first, followed by non-center candidate viewpoints. This matches existing display logic that maps candidate viewpoint index back to face index.
- Replaced hard-coded candidate divisor `6` with `NUM_CANDIDATES_PER_FACE`.
- Replaced multiple hard-coded viewpoint distance values with `VIEWPOINT_DISTANCE`.
- Stored and displayed the latest path length through `last_path_length`.

## UI improvements

- Added a right-side Workflow panel that displays:
  - Model loaded
  - Selected face
  - Segmented patches
  - Patch centers
  - Center viewpoints
  - All viewpoints
  - Optimal viewpoints
  - Sensor volumes
  - OBB boxes
  - Collision checked
  - Path points
  - Path length
- Added status-bar updates for non-critical workflow feedback.
- Added Workflow panel access through both `View` and `Help` menus.
- Created the Workflow panel automatically on startup.
- Updated workflow status after import, clear, segmentation, center calculation, viewpoint generation, optimal viewpoint filtering, sensor volume creation, OBB generation, collision detection, path planning, CSV export, and face selection.
- Added a right-side Layers panel with checkbox controls for:
  - Model / segmented patches
  - Face centers
  - Normal lines
  - All viewpoints
  - Optimal viewpoints
  - Planned path
  - Sensor volumes
  - OBB boxes
- Added a shared `render_scene()` function and routed the optimized layer toggles through it.
- Replaced repeated segmentation parameter popups with one consolidated dialog for U/V values.
- Replaced repeated sensor parameter popups with one consolidated dialog for width/height/depth.
- Added a generic `BackgroundWorker` and progress dialog helper.
- Routed ABC path planning through a background worker so the GUI can remain responsive while the solver runs.

## Still recommended for a future larger refactor

- Split the single script into modules such as `geometry.py`, `viewpoints.py`, `collision.py`, `planning.py`, `renderer.py`, and `app.py`.
- Move segmentation and collision detection into `QThread` workers with progress and cancel controls.
- Split the old duplicate view-toggle implementations out of the file once runtime behavior is verified with real models. The optimized definitions are currently placed later in the file and override the old ones to reduce migration risk.
- Add true cancellation support to the ABC solver itself. The GUI now runs ABC in the background, but the underlying solver still needs cooperative cancellation hooks.
- Add persistent user presets for segmentation, sensor size, and ABC parameters.

## 2026-08-08 large STEP benchmark and optimization boundary

The user-supplied `D:\Solidworks Files\真实组件最终版.step` is 65,965,998 bytes.
Its STEP header identifies an Open CASCADE 7.6 export. Current OCC inspection
resolves it to one dense, trimmed degree-3 B-spline face with a 722 x 722 pole
grid (521,284 control points), 720 U knots, 720 V knots, 52 wires and 58 edges.

Measured with the existing `test` interpreter plus the `Pythonocc` OCC 7.9.3
runtime:

- STEP read/transfer: approximately 6.1-6.3 s.
- Topology enumeration: approximately 0.003 s.
- Explicit 1.0 mm / 0.5 rad parallel triangulation: 42.305 s, producing 224,023
  nodes and 439,703 triangles.
- `BRepMesh_IncrementalMesh.IsParallelDefault()` returned `False`; the 42.305 s
  measurement explicitly requested parallel meshing and is not a serial versus
  parallel comparison.

At the tested 1.0 mm setting, explicit OCC meshing is the largest measured
cost, at roughly 6.8 times the STEP read/transfer time. This statement is
deliberately narrower than claiming that meshing dominates the visible import.
The benchmark did not separately time `DisplayShape`, `FitAll`, `Repaint`, Qt
event processing, or a cold versus already-triangulated shape. In the current
code, STEP/IGES reading runs in a worker, but `DisplayShape`, `FitAll`, and
`Repaint` execute in the UI-thread success callback and can still make the
window appear blocked.

Rewriting the Python UI in C++ is therefore not the first import optimization.
With the same OCC version, STEP reader, meshing algorithm, mesh parameters, and
equivalent viewer behavior, both the reader and mesher already execute in C++,
so a UI-language rewrite alone is unlikely to provide an order-of-magnitude
improvement. C++ or pybind11 can still be appropriate for a Python-level loop
that later profiling proves expensive; a gain caused by changing algorithms,
mesh tolerances, caching, or data representation should not be attributed to
the language rewrite itself.

Recommended order of work:

1. **P0 - instrument the real GUI path.** Measure cold and warm runs separately
   around STEP read/transfer, `DisplayShape`, `FitAll`, `Repaint`, and completion
   of the next Qt paint event. Record whether a triangulation was already
   attached, and compare serial/parallel meshing and a small matrix of absolute
   or relative linear and angular deflections.
2. If the source model is controllable, reduce/refit the dense surface or export
   a simpler analytic B-Rep, but only against an agreed inspection-accuracy
   tolerance and a geometry/regression check.
3. Offer a coarse, explicitly parameterized preview mesh while retaining the
   exact B-Rep for geometric processing. Select the preview tolerance from the
   measured visual and inspection requirements rather than assuming that 1 mm
   is universally appropriate.
4. Pre-mesh the isolated newly loaded shape in a worker with explicit parallel
   and deflection settings. Publish it to the UI only after meshing succeeds;
   all viewer and AIS operations must remain on the UI thread.
5. Cache a defined artifact, not merely an unspecified "model cache". Viable
   artifacts are either a validated B-Rep serialization that preserves its
   attached triangulation, or a separate preview mesh. A preview-mesh cache
   does not remove the need to read the exact B-Rep for segmentation and other
   CAD operations.
6. Treat canonical path, size, and modification time as fast lookup hints. The
   authoritative cache identity must include the source content SHA-256, source
   unit and reader options, OCC/pythonOCC versions, cache schema and meshing
   algorithm versions, absolute/relative mode, linear and angular deflections,
   and parallel setting. Reject or rebuild entries when any identity field is
   different, and validate the cached bounds and topology/mesh counts on load.
7. Profile center, viewpoint, collision, and path stages independently before
   moving only measured Python hotspots to NumPy, Cython, pybind11, or C++.

## Center exclusion: canonical data contract

This section is a design for a future center-review feature; it is not an
implemented-feature claim. A center removed in the UI is a reversible planning
exclusion, not deletion of the underlying CAD face.

### One source of truth

Do not delete entries independently from `current_faces`, `face_centers`, or
`face_normals`, and do not make those parallel lists the authoritative model.
The current `get_centers()` loop may fail for one face and continue; omitting
that entry would shift every later face/center index. Use one canonical record
per segmented patch instead:

```text
PatchRecord
  patch_id: stable string
  face: TopoDS_Face
  center: gp_Pnt or None
  normal: gp_Vec or None
  center_status: pending | ready | error
  center_error: string
  enabled_for_planning: bool
  source_face_index / patch_index / area: diagnostic metadata
```

A center calculation failure keeps its `PatchRecord`, stores the error, leaves
`center`/`normal` as `None`, and makes the record ineligible for planning. The
single `active_patch_records()` projection returns only ready records whose
`enabled_for_planning` flag is true. Any legacy faces/centers/normals lists must
be derived together from the same projection and must never be mutated
independently.

Viewpoint identity must also be explicit. Evolve `ViewpointRecord` to carry
`patch_id` and `kind` (`center` or `candidate`) with its point, pose, validity,
and collision state. Rendering, optimal-view filtering, and path input must
group by these fields instead of inferring a face from list position, from the
"all centers first" layout, or from a fixed candidate count. The 3D center
display object should likewise map directly to `patch_id` so picking cannot
remove the wrong record.

### Invalidation matrix

| Change | Must clear or rebuild | May remain |
| --- | --- | --- |
| Enable, exclude, restore, or edit a center/normal | Active projection; center/candidate/optimal viewpoints and pose records; center/normal/viewpoint display handles; sensor volumes; collision results and execution flags; path objects, `optimal_path`, and `last_path_length`; speed result; speed CSV reference; local RoboDK summary | Imported shape, all `PatchRecord` objects, collision geometry, and geometry-derived OBBs when those are independent of the planning mask |
| Recompute centers/normals | The same downstream data as a center edit, plus center-status/error fields and center-marker displays | Patch IDs and patch faces |
| Re-segment the model | Patch records and their IDs; any unmatched saved mask; every target-derived view, sensor, collision-result, path, speed, export, and RoboDK state | Imported shape; independently derived collision geometry/OBBs if their source and parameters did not change |
| Load a different model | All patch, mask, viewpoint, sensor, collision, path, speed, export, RoboDK, and display state | Application preferences only |
| Change sensor-volume parameters | Sensor volumes, their displays, and collision results/flags | Patch selection, viewpoints, path, and speed result unless collision feasibility is later made a path constraint |
| Change collision geometry or OBB parameters | Collision geometry/OBBs, their displays, and collision results/flags | Inspection mask and target viewpoints |
| Change only layer visibility | Redraw the affected layer | All computed data |

Clearing `last_robodk_import` only marks the local summary stale. It must not
silently delete or overwrite a program already created in RoboDK; replacement
remains a separate, explicit user action.

### Inspection targets versus collision geometry

Maintain two separate collections:

- `inspection_patch_records`: segmented scan targets controlled by the planning
  mask and consumed by center/viewpoint/path generation.
- `collision_geometry`: the full imported shape and any designated fixtures or
  obstacles, with its own OBB/BVH representation and version.

Excluding a center changes only the first collection. It must not accidentally
remove the corresponding physical surface from collision checks. The current
`current_faces` list can represent only a selected segmented face, so it must
not be treated as synonymous with the complete collision model. The collision
policy must also state whether the intended target surface is allowed to enter
the scan volume; any such exception should be an explicit geometry role, not a
side effect of the planning mask.

### Stable IDs and cross-session masks

A saved JSON mask should contain the source content SHA-256, a segmentation
contract/algorithm version, normalized segmentation parameters, and the
`patch_id` plus enabled state for every record. Patch IDs must be reproducible
under that pinned contract, for example from the source identity, deterministic
source-face/patch keys, and the segmentation version. A `TopoDS_Face` object
address or transient OCC hash is not a persistent ID.

Reject a mask by default when the source hash, segmentation version, or
parameters differ. An optional recovery mode may propose matches using a
geometry signature such as center, normal, and area within documented
tolerances, but it must show a preview and require confirmation rather than
silently applying an ambiguous match.

### Three UI versions on the same contract

1. **Direct 3D picking:** click a center, Ctrl-click for multi-select, press
   Delete to soft-exclude, and provide Undo/Restore. Show excluded centers as
   grey or hollow markers so the action remains visible and reversible. This is
   the fastest workflow for a small number of corrections but is sensitive to
   occlusion and dense point layouts.
2. **Docked review table:** provide enabled checkboxes, patch ID, coordinates,
   normal, area, status/error, sorting/filtering, linked 3D highlighting, and
   validated JSON mask import/export. This gives the best audit trail and exact
   selection for engineering review.
3. **Batch spatial/rule filtering:** add rectangle/lasso selection, a 3D ROI,
   and rules based on position, normal, area, or neighborhood. Every batch rule
   uses preview/apply/cancel and one undoable transaction. This scales to many
   centers but has the highest interaction and validation complexity.

Recommended delivery order: first implement and test the shared
`PatchRecord`/mask/invalidation layer, then direct 3D picking, then the auditable
table and cross-session mask manifest. Add lasso/ROI/rules only when real center
counts justify the extra complexity.
