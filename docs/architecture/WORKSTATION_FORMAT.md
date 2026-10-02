# Workstation file format (`.swstation`)

The first workstation format is a portable ZIP container. It stores the
current application state without serializing live Python, Qt, viewer, thread,
or RoboDK connection objects.

## Contents

- `manifest.json`: format version, workstation identity, UTC timestamps, unit
  conventions, runtime information, and SHA-256 for every data member.
- `state.json`: parameters, geometry IDs, segmentation results, viewpoints,
  collision results, path and speed data, calibration, layer visibility,
  camera state, and the operation log.
- `geometry/*.brep`: the current model, selected/segmented BREP faces, source
  faces referenced by `SurfacePatch`, and completed sensor-volume geometry.
- `source/original.*`: the imported STEP/IGES file when it is still available
  at save time. Loading never depends on the original path.

All positions and lengths use millimetres. Quaternions use `wxyz`. Explicit
geometry IDs are used for references; loader code never guesses face identity
from a new STEP traversal.

## Reliability and compatibility

Saving writes a sibling temporary archive, verifies it, and then atomically
replaces the destination. Loading checks member paths, per-member and total
expanded size, required members, SHA-256 values, format version, BREP decoding,
geometry references, and path indices before returning a candidate state. The
GUI commits that candidate only after validation and restores the prior state
if display reconstruction fails.

Version 1 rejects newer format versions. It is intended for compatible
PythonOCC/OpenCASCADE installations and does not promise arbitrary cross-OCC
compatibility. RoboDK import/reachability data is retained as historical
evidence only and must be revalidated against the live station.

## Use

1. Import and process a model to any completed stage.
2. Choose **File > Save Workstation** (`Ctrl+S`).
3. Move the single `.swstation` file if desired; the source CAD file may be
   absent.
4. Choose **File > Open Workstation** (`Ctrl+O`) and continue processing.

A `.swstation` file may also be dragged onto the 3D viewer. The shared drop
entry point validates one local file and dispatches workstations to the archive
loader while retaining the existing STEP/IGES model-import path.

Version 1 uses manual save. It does not store an algorithm mid-iteration,
embed a RoboDK station, provide autosave, or maintain revision history.

## State classification

- Persisted: model/source metadata, explicit BREP geometry and references,
  segmentation structures/parameters/diagnostics, centers and normals,
  viewpoint poses and provenance, collision inputs/results, OBB definitions,
  path order/parameters/diagnostics, complete speed result, calibration,
  external-operation summaries, visibility, camera, and operation history.
- Rebuilt after loading: viewer objects for model/patches, coordinate systems,
  lines, viewpoint markers, sensor volumes, OBB boxes, and path edges.
- Runtime only: Qt widgets and dialogs, OCC interactive handles, background
  threads, open file handles, and live RoboDK/API connections.
