# Integrated CAD Path, Speed Planning and RoboDK Software

This branch combines the existing pythonOCC CAD/viewpoint/path application with
post-path constrained speed planning and RoboDK program generation.

## Workflow

1. Import a STEP/IGES model from **File**, or drag one model file directly onto
   the 3D viewer.
2. Segment CAD faces and compute face centers/normals.
3. Generate/filter scanner viewpoints and order the measurement path.
4. Use **Calibration -> Read current RoboDK station mapping...** to capture the
   open station's Base/workpiece/scanner-TCP transforms, or load an existing
   validated scanner/tool mapping.
5. Use **Path Planning -> Check UR10 path reachability** to evaluate all path
   poses with the current RoboDK UR10 model, its joint limits, and an FK
   residual check. This operation is read-only.
6. Optionally use **Path Planning -> Import planned path to RoboDK** to create a
   movement-only program for checking the ordered poses before speed planning.
7. Open **Speed Planning -> Plan path speeds**.
8. Choose the deterministic baseline or thesis-style Double Q mode.
9. Export Pose+Speed CSV or import the speed plan directly to RoboDK.

The **Language** menu switches the live UI between English and Simplified
Chinese. Text is read from UTF-8 JSON catalogs in `locales/`; **Load language
JSON...** accepts an external catalog with the same validated key/placeholder
contract. Switching language only retranslates widgets and does not clear the
current model or planning state.

Catalogs are strict UTF-8 JSON objects with `meta` and `strings` sections; use
`locales/en.json` or `locales/zh_CN.json` as a template. Duplicate keys,
non-string values, invalid format placeholders, and placeholder-name changes
are rejected atomically, leaving the current language unchanged. Missing keys
in an otherwise valid external catalog fall back to English.

RoboDK output contains one `Set Speed` instruction immediately before every
`MoveJ/MoveL`. Its API order is linear speed, **joint speed**, linear
acceleration and **joint acceleration**. TCP orientation rate/acceleration are
kept as geometric diagnostics and are not passed off as robot-joint values.

The earlier planned-path import deliberately contains only `MoveJ/MoveL`
instructions. It does not set or validate speed, so it is a geometry/station
inspection artifact rather than a production speed-validation result. Both
RoboDK import modes retain station/frame/tool checks and transactional rollback.

RoboDK tree lookup is exact and rejects blank or duplicate robot, frame, tool,
program and generated-target names; RoboDK's closest-name fallback is never
accepted. Replacing a generated namespace inventories every exact
`<namespace>P<number>` target, including targets left by a previously longer
path, while leaving similar prefixes untouched. Publication uses a temporary
program and targets. On failure it vacates newly published names, restores all
old backups independently, removes new objects best-effort, and restores
rendering before returning the original error plus any cleanup diagnostics.

For `robodk_tcp_is_scanner`, `T_flange_scanner` must match the selected RoboDK
Tool TCP. For `separate_tool_frame`, planning/export can still read an older
configuration without `T_flange_tool`, but RoboDK import is blocked until a
measured `T_flange_tool` is present and matches `Tool.PoseTool()` within the
configured position/orientation tolerances.

Schema 1.2 keeps station and robot-Base relationships separate:
`T_station_reference_frame` expresses the workpiece/reference frame in the
RoboDK station origin, while `T_base_workpiece` expresses it in UR10 Base.
The capture action also stores `T_station_robot_base` and rejects a file unless
`inverse(T_station_robot_base) * T_station_reference_frame` equals
`T_base_workpiece`. Capturing is read-only and marks the result
`station_verified`; it is not physical scanner/CAD calibration.

UR10 reachability reuses the robot-model APIs built into RoboDK rather than the
Universal Robots postprocessor. For each command TCP pose it evaluates
`T_base_flange = T_base_workpiece * T_workpiece_tcp * inverse(T_flange_tcp)`,
calls `SolveIK_All`, filters solutions using `JointLimits`, selects a branch
near the previous joints, and verifies it with `SolveFK`. It creates no station
objects and does not move the robot. Collision, dynamics and real-hardware
safety remain separate acceptance gates.

## Start without installing packages

For normal use, double-click `start_software.cmd` in the repository root. It
sets the working directory and delegates to the existing PowerShell launcher.

For diagnostics, run the underlying launcher directly from the repository
root:

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
the result of dynamics validation. The new reachability report validates pose
IK and model joint limits, and records a continuity-oriented branch candidate;
it does not yet validate the resulting joint-speed, joint-acceleration or torque
trajectory. Before production execution, validate those quantities, collision
clearance and the physical scanner installation against the real system.

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
