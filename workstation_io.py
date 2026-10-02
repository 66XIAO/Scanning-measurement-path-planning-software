"""Versioned, portable workstation archives for the integrated application.

The ``.swstation`` file is a ZIP container containing JSON plus OpenCASCADE
BREP members.  It never unpickles Python objects and validates all archive
paths, member sizes, checksums and geometry references before returning a
candidate state.
"""

from dataclasses import asdict, dataclass, fields, is_dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
import platform
import shutil
import tempfile
import zipfile

import OCC

from OCC.Core.BRep import BRep_Builder
from OCC.Core.BRepTools import breptools
from OCC.Core.Bnd import Bnd_OBB
from OCC.Core.gp import gp_Dir, gp_Pnt, gp_Vec
from OCC.Core.TopoDS import TopoDS_Shape

from pose_transform import parse_extrinsic_config
from speed_planning_core import SpeedPlanPoint, SpeedPlanResult
from state import AppState, ViewpointRecord
from surface_segmentation import SurfacePatch, is_surface_patch


FORMAT_NAME = "software_optimizing_workstation"
FORMAT_VERSION = 1
APPLICATION_VERSION = "1.0"
MAX_MEMBER_BYTES = 256 * 1024 * 1024
MAX_TOTAL_BYTES = 1024 * 1024 * 1024
MANIFEST_NAME = "manifest.json"


class WorkstationError(ValueError):
    pass


@dataclass
class WorkstationSnapshot:
    state: AppState
    settings: dict
    view_state: dict


@dataclass
class WorkstationLoadResult:
    state: AppState
    settings: dict
    view_state: dict
    warnings: list
    manifest: dict


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _json_safe(value):
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise WorkstationError("Non-finite number cannot be saved")
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if hasattr(value, "tolist"):
        return _json_safe(value.tolist())
    if hasattr(value, "item"):
        return _json_safe(value.item())
    raise WorkstationError("Unsupported value in workstation metadata: {}".format(
        type(value).__name__))


def _point(value):
    if value is None:
        return None
    return [float(value.X()), float(value.Y()), float(value.Z())]


def _make_point(value):
    if value is None:
        return None
    if not isinstance(value, list) or len(value) != 3:
        raise WorkstationError("Point/vector must contain three values")
    numbers = [float(item) for item in value]
    if not all(math.isfinite(item) for item in numbers):
        raise WorkstationError("Point/vector contains a non-finite value")
    return gp_Pnt(*numbers)


def _make_vec(value):
    point = _make_point(value)
    return None if point is None else gp_Vec(point.X(), point.Y(), point.Z())


def _pose(value):
    if value is None:
        return None
    result = [float(item) for item in value]
    if len(result) != 4 or not all(math.isfinite(item) for item in result):
        raise WorkstationError("Quaternion must contain four finite wxyz values")
    return result


def _pose_pairs(values):
    return [{"point": _point(point), "pose": _pose(pose)} for point, pose in values]


def _decode_pose_pairs(values):
    result = []
    for item in values:
        pose = _pose(item["pose"])
        result.append((_make_point(item["point"]), None if pose is None else tuple(pose)))
    return result


def _record(value):
    return {
        "point": _point(value.point), "pose": _pose(value.pose),
        "face_index": int(value.face_index), "kind": str(value.kind),
        "valid": bool(value.valid), "collision": bool(value.collision),
        "global_index": int(value.global_index),
        "candidate_index": int(value.candidate_index),
    }


def _decode_record(value):
    pose = _pose(value["pose"])
    return ViewpointRecord(
        point=_make_point(value["point"]),
        pose=None if pose is None else tuple(pose),
        face_index=int(value["face_index"]), kind=str(value["kind"]),
        valid=bool(value.get("valid", True)),
        collision=bool(value.get("collision", False)),
        global_index=int(value.get("global_index", -1)),
        candidate_index=int(value.get("candidate_index", -1)))


def _obb(value):
    return {
        "center": _point(value.Center()),
        "x_direction": _point(value.XDirection()),
        "y_direction": _point(value.YDirection()),
        "z_direction": _point(value.ZDirection()),
        "half_sizes": [float(value.XHSize()), float(value.YHSize()),
                       float(value.ZHSize())],
    }


def _decode_obb(value):
    center = _make_point(value["center"])
    directions = [gp_Dir(*[float(x) for x in value[name]]) for name in (
        "x_direction", "y_direction", "z_direction")]
    sizes = [float(x) for x in value["half_sizes"]]
    if len(sizes) != 3 or not all(math.isfinite(x) and x >= 0 for x in sizes):
        raise WorkstationError("Invalid OBB half sizes")
    return Bnd_OBB(center, directions[0], directions[1], directions[2], *sizes)


def _colour(value):
    try:
        return [float(value.Red()), float(value.Green()), float(value.Blue())]
    except Exception:
        return [0.2, 0.8, 0.2]


def _decode_colour(value):
    from OCC.Display.OCCViewer import rgb_color
    return rgb_color(*[float(item) for item in value])


def capture_workstation(state, settings=None, view_state=None):
    """Capture only stable application data; no Qt/OCC display handles."""
    return WorkstationSnapshot(state=state, settings=dict(settings or {}),
                               view_state=dict(view_state or {}))


def _write_shape(path, shape):
    if shape is None or shape.IsNull():
        raise WorkstationError("Cannot save a null BREP shape")
    if not breptools.Write(shape, path):
        raise WorkstationError("OpenCASCADE could not write geometry")


def _read_shape(path):
    shape = TopoDS_Shape()
    builder = BRep_Builder()
    if not breptools.Read(shape, path, builder) or shape.IsNull():
        raise WorkstationError("OpenCASCADE could not read geometry")
    return shape


def _state_document(state, geometry_ids):
    def gid(shape):
        if shape is None:
            return None
        key = id(shape)
        if key not in geometry_ids:
            raise WorkstationError("Geometry reference was not registered")
        return geometry_ids[key]

    faces = []
    for face in state.current_faces:
        if is_surface_patch(face):
            faces.append({
                "kind": "surface_patch", "source_geometry_id": gid(face.source_face),
                "source_face_index": int(face.source_face_index),
                "grid_u_index": int(face.grid_u_index),
                "grid_v_index": int(face.grid_v_index),
                "component_index": int(face.component_index),
                "center": _point(face.center), "normal": _point(face.normal),
                "area": float(face.area), "triangle_count": int(face.triangle_count),
                "bounds": _json_safe(face.bounds),
                "representative_uv": _json_safe(face.representative_uv),
                "strategy": str(face.strategy),
            })
        else:
            faces.append({"kind": "brep", "geometry_id": gid(face)})

    speed = None
    if state.speed_plan_result is not None:
        speed = {
            "algorithm": state.speed_plan_result.algorithm,
            "points": [asdict(point) for point in state.speed_plan_result.points],
            "total_time": state.speed_plan_result.total_time,
            "feasible": state.speed_plan_result.feasible,
            "warnings": list(state.speed_plan_result.warnings),
            "diagnostics": _json_safe(state.speed_plan_result.diagnostics),
        }

    return {
        "session": {
            "workstation_id": state.workstation_id,
            "created_at": state.workstation_created_at,
            "model_source_path": state.model_source_path,
            "model_source_name": state.model_source_name,
            "model_source_format": state.model_source_format,
        },
        "geometry": {
            "current_shape_id": gid(state.current_shape),
            "selected_face_id": gid(state.selected_face),
            "selected_face_source_index": int(state.selected_face_source_index),
            "current_faces": faces,
        },
        "parameters": {
            "segmentation": _json_safe(state.segmentation_parameters),
            "algorithms": _json_safe(state.algorithm_parameters),
            "sensor_size": _json_safe(state.sensor_size_config),
        },
        "geometry_data": {
            "original_face_normal": _point(state.original_face_normal),
            "reference_normal": _point(state.reference_normal),
            "face_centers": [_point(x) for x in state.face_centers],
            "face_normals": [_point(x) for x in state.face_normals],
            "center_view_points": [_point(x) for x in state.center_view_points],
            "view_points": [_point(x) for x in state.view_points],
            "center_view_points_with_pose": _pose_pairs(state.center_view_points_with_pose),
            "view_points_with_pose": _pose_pairs(state.view_points_with_pose),
            "optimal_viewpoints": [_point(x) for x in state.optimal_viewpoints],
            "optimal_viewpoints_with_pose": _pose_pairs(state.optimal_viewpoints_with_pose),
            "viewpoint_records": [_record(x) for x in state.viewpoint_records],
            "center_viewpoint_records": [_record(x) for x in state.center_viewpoint_records],
            "optimal_viewpoint_records": [_record(x) for x in state.optimal_viewpoint_records],
        },
        "segmentation_diagnostics": _json_safe(state.last_segmentation_diagnostics),
        "collision": {
            "results": [bool(x) for x in state.collision_results],
            "enabled": bool(state.collision_detection_executed),
            "sensor_volumes": [{
                "vertices": [_point(x) for x in vertices], "color": _colour(color),
                "shape_id": gid(box),
            } for box, vertices, color in state.sensor_volumes_list],
            "face_obbs": [_obb(x) for x in state.face_obbs],
            "process_started": bool(state.collision_process_started),
            "process_finished": bool(state.collision_process_finished),
            "process_cancelled": bool(state.collision_process_cancelled),
            "sensor_volumes_created": bool(state.sensor_volumes_created),
            "obb_boxes_generated": bool(state.obb_boxes_generated),
        },
        "path": {
            "order": [int(x) for x in state.optimal_path],
            "length": float(state.last_path_length),
            "algorithm": state.last_path_algorithm,
            "diagnostics": _json_safe(state.path_algorithm_diagnostics),
        },
        "speed_plan": speed,
        "calibration": (state.extrinsic_config.to_dict()
                        if state.extrinsic_config is not None else None),
        "calibration_source": {
            "path": state.extrinsic_config_path,
            "sha256": state.extrinsic_config_sha256,
        },
        "external_history": {
            "last_speed_csv_path": state.last_speed_csv_path,
            "last_robodk_import": _json_safe(state.last_robodk_import),
            "last_reachability_report": _json_safe(state.last_reachability_report),
        },
        "visibility": {name: bool(getattr(state, name)) for name in (
            "show_model", "show_workpiece_coordinate_system", "show_face_centers",
            "show_normal_lines", "show_all_viewpoints", "show_optimal_viewpoints",
            "show_planned_path", "show_sensor_volumes", "show_obb_boxes")},
        "workpiece_coordinate_system_size": float(state.workpiece_coordinate_system_size),
        "operation_history": _json_safe(state.operation_history),
        "workflow_steps": _json_safe(state.workflow_steps),
    }


def _collect_geometry(state):
    shapes = []
    seen = set()
    def add(shape, label):
        if shape is None:
            return
        key = id(shape)
        if key not in seen:
            seen.add(key)
            shapes.append((shape, label))
    add(state.current_shape, "model")
    add(state.selected_face, "selected_face")
    for index, face in enumerate(state.current_faces):
        add(face.source_face if is_surface_patch(face) else face, "face_{}".format(index))
    for index, item in enumerate(state.sensor_volumes_list):
        add(item[0], "sensor_{}".format(index))
    ids = {id(shape): "g{:06d}".format(index) for index, (shape, _label) in enumerate(shapes)}
    return shapes, ids


def save_workstation(path, snapshot):
    """Write and verify an archive, then atomically replace *path*."""
    if not isinstance(snapshot, WorkstationSnapshot):
        raise TypeError("snapshot must be returned by capture_workstation")
    path = os.path.abspath(os.fspath(path))
    if not path.lower().endswith(".swstation"):
        path += ".swstation"
    parent = os.path.dirname(path) or os.getcwd()
    os.makedirs(parent, exist_ok=True)
    shapes, geometry_ids = _collect_geometry(snapshot.state)
    document = _state_document(snapshot.state, geometry_ids)
    document["settings"] = _json_safe(snapshot.settings)
    document["view_state"] = _json_safe(snapshot.view_state)

    staging = tempfile.mkdtemp(prefix="swstation-build-")
    temp_archive = None
    try:
        members = {}
        for shape, label in shapes:
            gid = geometry_ids[id(shape)]
            filename = "geometry/{}_{}.brep".format(gid, label)
            disk_path = os.path.join(staging, filename.replace("/", os.sep))
            os.makedirs(os.path.dirname(disk_path), exist_ok=True)
            _write_shape(disk_path, shape)
            members[filename] = disk_path

        source_path = snapshot.state.model_source_path
        if source_path and os.path.isfile(source_path):
            ext = os.path.splitext(source_path)[1].lower()
            original_name = "source/original{}".format(ext)
            copied = os.path.join(staging, original_name.replace("/", os.sep))
            os.makedirs(os.path.dirname(copied), exist_ok=True)
            shutil.copyfile(source_path, copied)
            members[original_name] = copied
            document["session"]["embedded_source_member"] = original_name

        state_bytes = json.dumps(document, ensure_ascii=False, sort_keys=True,
                                 separators=(",", ":"), allow_nan=False).encode("utf-8")
        checksums = {"state.json": hashlib.sha256(state_bytes).hexdigest()}
        for name, disk_path in members.items():
            with open(disk_path, "rb") as stream:
                checksums[name] = hashlib.sha256(stream.read()).hexdigest()
        manifest = {
            "format": FORMAT_NAME, "format_version": FORMAT_VERSION,
            "workstation_id": snapshot.state.workstation_id,
            "created_at": snapshot.state.workstation_created_at,
            "saved_at": _utc_now(), "length_unit": "mm",
            "quaternion_order": "wxyz", "platform": platform.platform(),
            "python_version": platform.python_version(), "members": checksums,
            "application_version": APPLICATION_VERSION,
            "occ_version": str(getattr(OCC, "VERSION", "unknown")),
        }
        manifest_bytes = json.dumps(manifest, ensure_ascii=False, sort_keys=True,
                                    indent=2, allow_nan=False).encode("utf-8")
        fd, temp_archive = tempfile.mkstemp(prefix=".swstation-", suffix=".tmp", dir=parent)
        os.close(fd)
        with zipfile.ZipFile(temp_archive, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr(MANIFEST_NAME, manifest_bytes)
            archive.writestr("state.json", state_bytes)
            for name, disk_path in members.items():
                archive.write(disk_path, name)
        _validate_archive(temp_archive, load_geometry=False)
        os.replace(temp_archive, path)
        temp_archive = None
        return path, manifest
    finally:
        shutil.rmtree(staging, ignore_errors=True)
        if temp_archive and os.path.exists(temp_archive):
            os.unlink(temp_archive)


def _safe_infos(archive):
    infos = archive.infolist()
    total = 0
    names = set()
    for info in infos:
        name = info.filename.replace("\\", "/")
        parts = name.split("/")
        if (not name or name.startswith("/") or any(part in ("", ".", "..") for part in parts)
                or ":" in parts[0]):
            raise WorkstationError("Unsafe archive member path")
        if name in names:
            raise WorkstationError("Duplicate archive member")
        names.add(name)
        if info.file_size > MAX_MEMBER_BYTES:
            raise WorkstationError("Archive member exceeds size limit")
        total += info.file_size
        if total > MAX_TOTAL_BYTES:
            raise WorkstationError("Archive exceeds expanded size limit")
    return names


def _validate_archive(path, load_geometry=True):
    with zipfile.ZipFile(path, "r") as archive:
        names = _safe_infos(archive)
        if MANIFEST_NAME not in names or "state.json" not in names:
            raise WorkstationError("Workstation is missing required metadata")
        manifest = json.loads(archive.read(MANIFEST_NAME).decode("utf-8"))
        if manifest.get("format") != FORMAT_NAME:
            raise WorkstationError("Not a supported workstation archive")
        if int(manifest.get("format_version", -1)) != FORMAT_VERSION:
            raise WorkstationError("Unsupported workstation format version")
        checksums = manifest.get("members")
        if not isinstance(checksums, dict):
            raise WorkstationError("Workstation checksum table is missing")
        for name, expected in checksums.items():
            if name not in names:
                raise WorkstationError("Workstation member is missing: {}".format(name))
            actual = hashlib.sha256(archive.read(name)).hexdigest()
            if actual != expected:
                raise WorkstationError("Workstation member checksum failed: {}".format(name))
        document = json.loads(archive.read("state.json").decode("utf-8"))
        return manifest, document


def load_workstation(path):
    """Fully validate and decode a candidate state without touching the GUI."""
    path = os.path.abspath(os.fspath(path))
    manifest, document = _validate_archive(path)
    state = AppState()
    temp_dir = tempfile.mkdtemp(prefix="swstation-load-")
    try:
        geometry = {}
        with zipfile.ZipFile(path, "r") as archive:
            for name in manifest["members"]:
                if not name.startswith("geometry/"):
                    continue
                target = os.path.join(temp_dir, os.path.basename(name))
                with archive.open(name) as source, open(target, "wb") as output:
                    shutil.copyfileobj(source, output)
                gid = os.path.basename(name).split("_", 1)[0]
                geometry[gid] = _read_shape(target)

        def shape(gid, required=False):
            if gid is None and not required:
                return None
            if gid not in geometry:
                raise WorkstationError("Invalid geometry reference: {}".format(gid))
            return geometry[gid]

        session = document["session"]
        state.workstation_id = str(session["workstation_id"])
        state.workstation_created_at = str(session["created_at"])
        state.workstation_saved_at = str(manifest["saved_at"])
        state.workstation_path = path
        state.workstation_dirty = False
        state.model_source_path = str(session.get("model_source_path", ""))
        state.model_source_name = str(session.get("model_source_name", ""))
        state.model_source_format = str(session.get("model_source_format", ""))

        geo = document["geometry"]
        state.current_shape = shape(geo.get("current_shape_id"))
        state.selected_face = shape(geo.get("selected_face_id"))
        state.selected_face_source_index = int(
            geo.get("selected_face_source_index", -1))
        for item in geo.get("current_faces", []):
            if item.get("kind") == "brep":
                state.current_faces.append(shape(item.get("geometry_id"), required=True))
            elif item.get("kind") == "surface_patch":
                patch = SurfacePatch(
                    source_face=shape(item.get("source_geometry_id"), required=True),
                    source_face_index=int(item["source_face_index"]),
                    grid_u_index=int(item["grid_u_index"]),
                    grid_v_index=int(item["grid_v_index"]),
                    component_index=int(item["component_index"]),
                    center=_make_point(item["center"]), normal=_make_vec(item["normal"]),
                    area=float(item["area"]), triangle_count=int(item["triangle_count"]),
                    bounds=tuple(float(x) for x in item["bounds"]),
                    representative_uv=tuple(float(x) for x in item["representative_uv"]),
                    strategy=str(item.get("strategy", "mesh_grid")))
                state.current_faces.append(patch)
                state.surface_patches.append(patch)
            else:
                raise WorkstationError("Unknown segmented-face representation")

        params = document.get("parameters", {})
        state.segmentation_parameters = dict(params.get("segmentation", {}))
        state.algorithm_parameters = dict(params.get("algorithms", {}))
        state.sensor_size_config = dict(params.get("sensor_size", {}))
        data = document.get("geometry_data", {})
        state.original_face_normal = _make_vec(data.get("original_face_normal"))
        state.reference_normal = _make_vec(data.get("reference_normal"))
        for name in ("face_centers", "center_view_points", "view_points", "optimal_viewpoints"):
            setattr(state, name, [_make_point(x) for x in data.get(name, [])])
        state.face_normals = [_make_vec(x) for x in data.get("face_normals", [])]
        for name in ("center_view_points_with_pose", "view_points_with_pose",
                     "optimal_viewpoints_with_pose"):
            setattr(state, name, _decode_pose_pairs(data.get(name, [])))
        for name in ("viewpoint_records", "center_viewpoint_records",
                     "optimal_viewpoint_records"):
            setattr(state, name, [_decode_record(x) for x in data.get(name, [])])
        state.last_segmentation_diagnostics = dict(
            document.get("segmentation_diagnostics", {}))

        collision = document.get("collision", {})
        state.collision_results = [bool(x) for x in collision.get("results", [])]
        for item in collision.get("sensor_volumes", []):
            state.sensor_volumes_list.append((
                shape(item.get("shape_id"), required=True),
                [_make_point(x) for x in item["vertices"]],
                _decode_colour(item["color"])))
        state.face_obbs = [_decode_obb(x) for x in collision.get("face_obbs", [])]
        state.collision_detection_executed = bool(collision.get("enabled", False))
        state.collision_process_started = bool(collision.get("process_started", False))
        state.collision_process_finished = bool(collision.get("process_finished", False))
        state.collision_process_cancelled = bool(collision.get("process_cancelled", False))
        state.sensor_volumes_created = bool(collision.get("sensor_volumes_created", False))
        state.obb_boxes_generated = bool(collision.get("obb_boxes_generated", False))

        path_data = document.get("path", {})
        state.optimal_path = [int(x) for x in path_data.get("order", [])]
        state.last_path_length = float(path_data.get("length", 0.0))
        state.last_path_algorithm = str(path_data.get("algorithm", ""))
        state.path_algorithm_diagnostics = dict(path_data.get("diagnostics", {}))
        if state.optimal_path and (min(state.optimal_path) < 0 or
                                   max(state.optimal_path) >= len(state.optimal_viewpoints) or
                                   len(set(state.optimal_path)) != len(state.optimal_path)):
            raise WorkstationError("Saved path contains invalid viewpoint indices")

        speed = document.get("speed_plan")
        if speed is not None:
            state.speed_plan_result = SpeedPlanResult(
                algorithm=str(speed["algorithm"]),
                points=[SpeedPlanPoint(**item) for item in speed["points"]],
                total_time=float(speed["total_time"]), feasible=bool(speed["feasible"]),
                warnings=list(speed.get("warnings", [])),
                diagnostics=dict(speed.get("diagnostics", {})))
        calibration = document.get("calibration")
        if calibration is not None:
            state.extrinsic_config = parse_extrinsic_config(calibration)
        calibration_source = document.get("calibration_source", {})
        state.extrinsic_config_path = str(calibration_source.get("path", ""))
        state.extrinsic_config_sha256 = str(calibration_source.get("sha256", ""))
        external = document.get("external_history", {})
        state.last_speed_csv_path = str(external.get("last_speed_csv_path", ""))
        state.last_robodk_import = dict(external.get("last_robodk_import", {}))
        state.last_reachability_report = dict(external.get("last_reachability_report", {}))
        for name, value in document.get("visibility", {}).items():
            if hasattr(state, name):
                setattr(state, name, bool(value))
        state.workpiece_coordinate_system_size = float(
            document.get("workpiece_coordinate_system_size", 0.0))
        state.operation_history = list(document.get("operation_history", []))
        state.workflow_steps = list(document.get("workflow_steps", []))

        warnings = []
        if state.last_robodk_import or state.last_reachability_report:
            warnings.append("robodk_history_requires_revalidation")
        return WorkstationLoadResult(
            state=state, settings=dict(document.get("settings", {})),
            view_state=dict(document.get("view_state", {})), warnings=warnings,
            manifest=manifest)
    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)
