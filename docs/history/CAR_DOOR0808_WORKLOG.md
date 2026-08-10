# CarDoor0808 implementation and validation record

Date: 2026-08-08  
Branch: `CarDoor0808`  
Confirmed input: `真实组件最终版.step`

## Input facts

- File size: 65,965,998 bytes.
- Bounding size observed by OCC: approximately 837 x 970 x 95 mm.
- Topology: one B-spline face, 52 trimming wires.
- OCC validity: failed with `UnorientableShape`.
- CAD surface-property area: 626,093.987365 mm2.  This value is diagnostic only
  because the BRep is invalid.

## Delivered behavior

- `segment_model(..., strategy="auto")` preserves `equal_param` for untrimmed
  models and selects `mesh_grid` for trimmed faces.
- XOY triangle clipping creates material-aware U/V cells without Boolean face
  splitting.
- Connected-component separation prevents a hole from joining disjoint regions.
- Patch centres and normals come from the original CAD surface at a valid mesh
  triangle UV location.
- The UI distinguishes partial equal-parameter fallback from input-geometry
  warnings.

## Real U=8, V=6 result

The authoritative machine-specific record is ignored by Git at:

`diagnostics/cardoor0808/mesh_grid_u8_v6.json`

Summary from the final headless run:

- STEP load: 6.314 s.
- Segmentation: 10.611 s.
- Mesh deflection: 5.0 mm; angular deflection: 0.5 rad.
- Source mesh: 40,974 nodes and 78,818 non-degenerate triangles.
- Partition fragments: 87,289.
- Non-empty XOY cells: 46 of 48.
- Connected components / patches: 47; one grid cell split into two components.
- Source mesh area: 581,512.263114 mm2.
- Partition area: 581,512.201618 mm2.
- Partition/source ratio: 0.999999894247.
- Mesh/CAD ratio: 0.928793878953; retained as an invalid-input warning.
- Invalid patch centres: 0.
- Invalid/unit-normal failures: 0.
- Regression result: passed with input BRep warning.

## Automated regression

All 27 repository tests passed in the Pythonocc-loaded environment, including
the two new surface-segmentation tests.  Existing equal-parameter, speed
planning, pose transformation and mocked RoboDK tests remained green.

## Scanner-coverage parameter study

The scanner reference states a theoretical 130 x 120 mm footprint at the best
standoff, with 30 x 17 mm laser-grid spacing and half-grid margins of 15 / 8.5
mm.  Because the text does not map each margin to a footprint axis, the current
branch uses a conservative maximum cell size of 100 x 90 mm.

For the approximately 837 x 970 mm car-door XOY bounds this gives U=9, V=11.
The real-model runs were:

- U=9, V=10: 86 occupied cells, 87 connected patches, 10.750 s segmentation.
- U=9, V=11: 90 occupied cells, 92 connected patches, 10.922 s segmentation.

Both conserved the source mesh with ratio 0.999999894247.  U=9, V=11 is the
validated default because its approximately 93.0 x 88.2 mm cells satisfy the
conservative limit.  The final 9 x 11 run classified every representative UV
point inside/on the face and found zero invalid normals.

## File organization

- Algorithm and data contract: `surface_segmentation.py`.
- Integration: `geometry.py`, `main.py`, `renderer.py`, `state.py`.
- Synthetic regression: `tests/test_surface_segmentation.py`.
- Real-model runner: `scripts/validate_car_door_segmentation.py`.
- Double-click launcher: `start_software.cmd`.
- Reproducible user guide: `docs/user/CAR_DOOR_SEGMENTATION_GUIDE.md`.
- Architecture decision: `docs/architecture/CAR_DOOR_SEGMENTATION.md`.
- Generated real-model JSON: `diagnostics/cardoor0808/` (Git-ignored).

The real STEP file is not copied into the repository.  No files are generated
in the project root.

## Open boundary

GUI visual acceptance remains manual.  Current mesh-grid rendering overlays
coloured patch representatives on the original surface rather than colouring
all triangle fragments.  Direct native STL import is not included because only
the STEP version was available for this implementation.

## 2026-08-09 recovery into the combined working tree

The segmentation delivery was found intact in commit `208c228` on branch
`CarDoor0808` after another task switched the shared checkout back to `main`.
Its files and integration hunks were restored into the current uncommitted
working tree without overwriting the newer planned-path RoboDK import, model
drop, or JSON internationalisation work. The unrelated MSCGA default reduction
contained in that commit was deliberately not restored.

Combined verification after the merge:

- 72 repository tests passed in the Pythonocc-loaded environment.
- 30 Python files passed in-memory compilation.
- `start_software.cmd --check` resolved the launcher and working directory.
- A fresh real-model U=9, V=11 run again produced 92 patches, area ratio
  0.999999894247, and zero invalid representative centres or normals.
- The machine-specific result is stored under ignored
  `diagnostics/cardoor0808/scanner_grid_study/mesh_grid_u9_v11_restored.json`.
- A Qt/OCC introspection smoke confirmed the fifth Path Planning action, live
  English/Chinese retranslation with state preservation, and the installed
  viewer drop filter. Windows pixel-level automation was unavailable because
  the Computer Use window enumerator returned `0x80070003`; real mouse-drop and
  screenshot acceptance therefore remain manual.

The active checkout remains `main` so the other task's uncommitted work is not
lost. No commit, branch switch, stash, reset, or cleanup operation was used.
