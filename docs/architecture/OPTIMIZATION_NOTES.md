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
