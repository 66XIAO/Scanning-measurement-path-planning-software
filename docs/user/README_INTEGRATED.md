# Integrated CAD Path, Speed Planning and RoboDK Software

This branch combines the existing pythonOCC CAD/viewpoint/path application with
post-path constrained speed planning and RoboDK program generation.

## Workflow

1. Import STEP/IGES model.
2. Segment CAD faces and compute face centers/normals.
3. Generate/filter scanner viewpoints and order the measurement path.
4. Load a validated `T_tool_scanner` from the **Calibration** menu.
5. Open **Speed Planning -> plan_path_speeds**.
6. Choose the deterministic baseline or thesis-style Double Q mode.
7. Export Pose+Speed CSV or import directly to RoboDK.

RoboDK output contains one `Set Speed` instruction immediately before every
`MoveJ/MoveL`. Its API order is linear speed, **joint speed**, linear
acceleration and **joint acceleration**. TCP orientation rate/acceleration are
kept as geometric diagnostics and are not passed off as robot-joint values.

## Start without installing packages

From the repository root, run:

```powershell
& '.\run_integrated_app.ps1'
```

The launcher uses the existing `test` Conda Python 3.12 interpreter and loads
OCC/PySide6 from the existing `Pythonocc` Conda environment. It does not install
or modify packages and does not use Conda base.

## Tests

Pure speed-planning tests can run in the existing Pytorch environment:

```powershell
$prefix='D:\Env\conda\2024\envs\Pytorch'
$env:PATH="$prefix;$prefix\Library\bin;$prefix\Scripts;$env:PATH"
& "$prefix\python.exe" -m unittest discover -s tests -v
```

## Safety boundary

The current planner enforces TCP/geometric speed, acceleration, curvature and
orientation-rate constraints. It does **not** yet claim robot dynamic validity.
The configured joint speed/acceleration values are RoboDK command limits, not
the result of inverse-kinematics or dynamics validation.
Before production execution, configure scanner-to-tool extrinsics and validate
IK branch continuity, joint speed, joint acceleration and joint torque against
the real robot model.

RoboDK import also requires a validated scanner-to-tool calibration. The
example identity matrix is format documentation only and does not unlock import.

## Review and next steps

- [`TASK_SUMMARY_2026-07-11.md`](../history/TASK_SUMMARY_2026-07-11.md):
  short, user-facing explanation of this delivery.
- [`INTEGRATED_SOFTWARE_PLAN.md`](../architecture/INTEGRATED_SOFTWARE_PLAN.md):
  implemented architecture and thesis mapping.
- [`NEXT_PHASE_RESEARCH_PLAN.md`](../plans/NEXT_PHASE_RESEARCH_PLAN.md):
  ordered TODO plan for extrinsics, continuous IK,
  joint dynamics, and geodesic/heat-method segmentation.
- [`TRAE_REVIEW_RESOLUTION_2026-07-11.md`](../history/TRAE_REVIEW_RESOLUTION_2026-07-11.md):
  independent adjudication and actual
  OCC regression evidence for the external review.
- [`CALIBRATION_GUIDE.md`](CALIBRATION_GUIDE.md): frame convention, JSON contract,
  validation workflow,
  CSV provenance, and the data still required from real calibration.
