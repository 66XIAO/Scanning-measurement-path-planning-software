# 3D Model Processing and Path Planning System — Main Orchestrator
#
# This file initialises the OCC viewer, owns the mutable application state,
# wires menu actions to the algorithm modules, and starts the Qt event loop.
# All heavy logic lives in the sibling modules:
#   geometry · viewpoints · collision · planning · renderer · ui_panels · workers · export_utils

import sys
import os
import math
import csv
import hashlib
import traceback

import numpy as np

from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeSphere
from OCC.Display.OCCViewer import rgb_color
from OCC.Display.SimpleGui import init_display
from OCC.Display.backend import get_qt_modules

from config import VIEWPOINT_DISTANCE, NUM_CANDIDATES_PER_FACE, ZENITH_ANGLE_DEG
from state import AppState
from geometry import (
    calculate_face_normal, segment_model, generate_viewpoint,
    display_coordinate_system, generate_face_obb, ConvertBndToShape,
)
from viewpoints import (
    calculate_viewpoint_pose, build_center_viewpoints, build_candidate_viewpoints,
    ordered_face_candidates, choose_closest_to_distance,
)
from collision import (
    create_sensor_volume, check_collision_sweep, summarize_collision_results,
)
from planning import (
    point_distance, build_distance_matrix, calculate_path_length,
    solve_greedy_open_path, run_abc_solver, run_mscga_solver,
    solve_sequential_open_path,
)
from renderer import render_scene as _render_scene, draw_path_edges, LayerVisibility
from ui_panels import (
    show_topmost_message, get_main_window, set_status_message,
    create_workflow_panel, create_layer_panel, update_workflow_status as _update_ws,
    sync_layer_panel as _sync_layer_panel, get_user_segment_params,
    get_sensor_parameters_dialog, show_usage_instructions, build_workflow_snapshot,
    format_workflow_snapshot,
)
from workers import TaskRunner
from export_utils import write_path_pose_csv, write_speed_plan_csv
from speed_planning_core import ConstraintProfile, plan_speed_profile
from speed_planning_ui import get_speed_planning_settings, get_robodk_import_settings
from robodk_bridge import import_speed_plan
from pose_transform import load_extrinsic_config, transform_pose_records
from cad_io import load_cad_shape


# ---------------------------------------------------------------------------
# 1. OCC viewer initialisation (must happen before any Qt widget work)
# ---------------------------------------------------------------------------

try:
    import PyQt5  # noqa: F401
    QT_BACKEND = "pyqt5"
except ImportError:
    import PySide6  # noqa: F401
    QT_BACKEND = "pyside6"

display, start_display, add_menu, add_function_to_menu = init_display(QT_BACKEND)
QtCore, QtGui, QtWidgets, QtOpenGL = get_qt_modules()

# Set a recognisable window title so ui_panels.get_main_window() can find it.
try:
    _app = QtWidgets.QApplication.instance()
    if _app:
        import time
        time.sleep(0.01)
        for _w in _app.topLevelWidgets():
            if hasattr(_w, "setWindowTitle"):
                _t = _w.windowTitle()
                if "pythonOCC" in _t or "3D Viewer" in _t:
                    _w.setWindowTitle("3D Model Processing and Path Planning System")
                    break
except Exception as _e:
    pass


# ---------------------------------------------------------------------------
# 2. Application state
# ---------------------------------------------------------------------------

state = AppState()

# Extra lists not in AppState (visual object handles that accumulate between renders)
center_points_objects = []
normal_line_objects = []

# Background-task infrastructure
task_runner = TaskRunner(QtCore, QtWidgets, get_main_window)

# Path-planning confirmation flag
path_planning_prompt_confirmed = False

# Collision-detection toggle (distinct from per-volume collision results)
collision_detection_enabled = True

# Guard: suppress close-event dialog until the app is fully running
_app_ready = False

# Reentrancy guard: prevent nested close-event dialogs
_closing_dialog_active = False


# ---------------------------------------------------------------------------
# 3. Close-event interceptor
# ---------------------------------------------------------------------------

class MainWindowCloseEvent(QtCore.QObject):
    def eventFilter(self, obj, event):
        global _closing_dialog_active
        if (event.type() == QtCore.QEvent.Close
                and _app_ready
                and not _closing_dialog_active):
            # Only intercept Close events from the main window itself,
            # not from child widgets (e.g. QMessageBox closing).
            main_win = get_main_window()
            if main_win is None or obj is not main_win:
                return super(MainWindowCloseEvent, self).eventFilter(obj, event)
            _closing_dialog_active = True
            try:
                result = show_topmost_message(
                    "Confirm Exit", "Are you sure you want to exit the program?",
                    type="question")
            finally:
                _closing_dialog_active = False
            if result == "no":
                event.ignore()
                return True
        return super(MainWindowCloseEvent, self).eventFilter(obj, event)


# ---------------------------------------------------------------------------
# 4. Rendering helper (bridges state → renderer.render_scene)
# ---------------------------------------------------------------------------

def _do_render(fit_all=True):
    """Centralised scene redraw from current *state*."""
    vis = LayerVisibility(
        model=state.show_model,
        face_centers=state.show_face_centers,
        normal_lines=state.show_normal_lines,
        all_viewpoints=state.show_all_viewpoints,
        optimal_viewpoints=state.show_optimal_viewpoints,
        planned_path=state.show_planned_path,
        sensor_volumes=state.show_sensor_volumes,
        obb_boxes=state.show_obb_boxes,
    )
    _render_scene(
        display, vis,
        state.current_shape, state.current_faces,
        state.face_centers, state.face_normals,
        state.center_view_points, state.view_points,
        state.optimal_viewpoints, state.optimal_path,
        state.sensor_volumes_list, state.face_obbs,
        state.coordinate_systems, state.optimal_path_objects,
        state.sensor_volume_objects, state.obb_visualizations,
        fit_all=fit_all,
    )


def _update_workflow(message=None):
    _update_ws(message, state)


def _sync_layer():
    _sync_layer_panel({
        "model": state.show_model,
        "face_centers": state.show_face_centers,
        "normal_lines": state.show_normal_lines,
        "all_viewpoints": state.show_all_viewpoints,
        "optimal_viewpoints": state.show_optimal_viewpoints,
        "planned_path": state.show_planned_path,
        "sensor_volumes": state.show_sensor_volumes,
        "obb_boxes": state.show_obb_boxes,
    })


# ---------------------------------------------------------------------------
# 5. Background-task runner
# ---------------------------------------------------------------------------

def run_background_task(title, message, fn, on_success, on_error=None):
    def failure(error_text):
        if on_error:
            on_error(error_text)
        else:
            show_topmost_message("Error", error_text, type="error")
    return task_runner.run(title, message, fn, on_success, failure)


# ---------------------------------------------------------------------------
# 6. Path helpers
# ---------------------------------------------------------------------------

def _delete_existing_path():
    state.optimal_path.clear()
    state.last_path_length = 0.0
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    for obj in state.optimal_path_objects:
        try:
            display.Context.Erase(obj, True)
        except Exception:
            pass
    state.optimal_path_objects.clear()


def _confirm_path_prompt():
    global path_planning_prompt_confirmed
    if not path_planning_prompt_confirmed:
        result = show_topmost_message(
            "Prompt",
            "It is recommended to hide existing paths before path planning. Proceed?",
            type="question")
        if result == "no":
            return False
        path_planning_prompt_confirmed = True
    return True


# ---------------------------------------------------------------------------
# 7. File menu handlers
# ---------------------------------------------------------------------------

def import_model(event=None):
    global path_planning_prompt_confirmed
    parent = get_main_window()
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
        parent, "Import Model", "",
        "Model Files (*.step *.stp *.iges *.igs);;"
        "STEP Files (*.step *.stp);;"
        "IGES Files (*.iges *.igs);;"
        "All Files (*)")
    if not file_path:
        return

    ext = os.path.splitext(file_path)[1].lower()
    if ext not in ('.step', '.stp', '.iges', '.igs'):
        show_topmost_message("Error", "Unsupported file format: {}".format(ext), type="error")
        return

    def load_shape():
        return load_cad_shape(file_path)

    def on_success(new_shape):
        global path_planning_prompt_confirmed
        # Commit atomically only after parsing succeeds. Cancelling or a read
        # error leaves the previously loaded model and downstream state intact.
        display.EraseAll()
        state.reset_all()
        state.current_shape = new_shape
        path_planning_prompt_confirmed = False
        display.DisplayShape(new_shape, color=rgb_color(0.7, 0.7, 0.7))
        display.FitAll()
        display.Repaint()
        _sync_layer()
        _update_workflow("Model imported: {}".format(os.path.basename(file_path)))
        print("Model imported successfully: {}".format(os.path.basename(file_path)))

    def on_error(error_text):
        print("Error importing model: {}".format(error_text))
        show_topmost_message("Error", "Error importing model:\n{}".format(error_text), type="error")

    set_status_message("Loading CAD model in background: {}".format(os.path.basename(file_path)))
    run_background_task("Import Model", "Reading STEP/IGES model...",
                        load_shape, on_success, on_error)


def clear_model(event=None):
    global path_planning_prompt_confirmed
    result = show_topmost_message(
        "Confirm Clear", "Are you sure you want to clear all models and data?", type="question")
    if result == 'no':
        return

    try:
        for cs in state.coordinate_systems:
            try:
                display.Context.Erase(cs, True)
            except Exception:
                pass
        for obj in state.sensor_volume_objects:
            try:
                display.Context.Remove(obj, True)
            except Exception:
                try:
                    display.Context.Erase(obj, True)
                except Exception:
                    pass
        for obj in state.obb_visualizations:
            try:
                display.Context.Remove(obj, True)
            except Exception:
                try:
                    display.Context.Erase(obj, True)
                except Exception:
                    pass

        display.EraseAll()
        display.Context.UpdateCurrentViewer()
        display.Repaint()

        state.reset_all()
        center_points_objects.clear()
        normal_line_objects.clear()
        path_planning_prompt_confirmed = False

        _update_workflow("All models and data have been cleared")
        show_topmost_message("Success", "All models and data have been cleared")
    except Exception as e:
        print("Error clearing model: {}".format(str(e)))


def exit_program(event=None):
    try:
        result = show_topmost_message(
            "Confirm Exit", "Are you sure you want to exit the program?", type="question")
        if result == 'yes':
            print("Program exited")
            sys.exit(0)
    except Exception as e:
        print("Error exiting program: {}".format(str(e)))
        sys.exit(1)


# ---------------------------------------------------------------------------
# 8. Model processing handlers
# ---------------------------------------------------------------------------

def select_single_face(event=None):
    if not state.current_shape:
        show_topmost_message("Prompt", "Please import a model first")
        return
    try:
        print("Please click a face in the 3D view to select...")
        display.SetSelectionModeFace()
        display.register_select_callback(select_face_clicked)
    except Exception as e:
        print("Error selecting face: {}".format(str(e)))
        traceback.print_exc()


def select_face_clicked(shapes, x, y):
    if not shapes:
        return
    shape = shapes[0]
    if shape.ShapeType() == TopAbs_FACE:
        state.selected_face = shape
        state.original_face_normal = calculate_face_normal(state.selected_face)
        print("Face selected, normal: ({:.4f}, {:.4f}, {:.4f})".format(
            state.original_face_normal.X(),
            state.original_face_normal.Y(),
            state.original_face_normal.Z()))

        from OCC.Core.AIS import AIS_Shape
        from OCC.Core.Quantity import Quantity_Color, Quantity_NOC_RED
        ais_shape = AIS_Shape(state.selected_face)
        ais_shape.SetColor(Quantity_Color(Quantity_NOC_RED))
        display.Context.Display(ais_shape, True)
        display.Context.UpdateCurrent()
        display.SetSelectionModeNeutral()
        display.Repaint()
        _update_workflow("Face selected successfully")


def segment_faces(event=None):
    if not state.current_shape:
        show_topmost_message("Prompt", "Please import a model first")
        return
    try:
        params = get_user_segment_params()
        if not params:
            return
        u, v = params
        shape_to_segment = state.current_shape
        if state.selected_face is not None:
            from OCC.Core.TopoDS import TopoDS_Compound
            from OCC.Core.BRep import BRep_Builder
            compound = TopoDS_Compound()
            builder = BRep_Builder()
            builder.MakeCompound(compound)
            builder.Add(compound, state.selected_face)
            shape_to_segment = compound

        def on_success(result):
            faces, diagnostics = result
            if not faces:
                show_topmost_message("Error", "Segmentation returned no valid faces", type="error")
                return
            state.current_faces = faces
            state.invalidate_after_segmentation()
            state.last_segmentation_diagnostics = diagnostics
            display.EraseAll()
            colors = [
                rgb_color(0.8, 0.8, 1.0), rgb_color(1.0, 0.8, 0.8),
                rgb_color(0.8, 1.0, 0.8), rgb_color(1.0, 1.0, 0.8),
            ]
            for i, face in enumerate(state.current_faces):
                display.DisplayShape(face, color=colors[i % len(colors)], update=False)
            display.FitAll()
            display.Repaint()
            print("Segmentation complete: {} patches (u={}, v={}), area ratio {:.6f}".format(
                len(faces), u, v, diagnostics.get("area_ratio", 0.0)))
            _update_workflow("Face segmentation complete: {} patches, area ratio {:.6f}".format(
                len(faces), diagnostics.get("area_ratio", 0.0)))
            if diagnostics.get("warnings"):
                show_topmost_message(
                    "Segmentation Diagnostics",
                    "Segmentation completed with diagnostics:\n\n" +
                    "\n".join(diagnostics["warnings"]),
                    type="warning")

        def on_error(error_text):
            print("Error segmenting faces: {}".format(error_text))
            show_topmost_message("Error", "Face segmentation failed:\n{}".format(error_text), type="error")

        set_status_message("Face segmentation running in background")
        run_background_task("Segment Faces", "Splitting CAD faces...",
                            lambda: segment_model(shape_to_segment, u, v,
                                                  return_diagnostics=True),
                            on_success, on_error)
    except Exception as e:
        print("Error segmenting faces: {}".format(str(e)))
        traceback.print_exc()


def get_centers(event=None):
    if not state.current_faces:
        show_topmost_message("Prompt", "Please segment faces first")
        return
    try:
        state.face_centers.clear()
        state.face_normals.clear()

        for cs in state.coordinate_systems:
            try:
                display.Context.Erase(cs, True)
            except Exception:
                pass
        state.coordinate_systems.clear()
        for obj in center_points_objects:
            try:
                display.Context.Erase(obj, True)
            except Exception:
                pass
        center_points_objects.clear()

        from OCC.Core.BRepGProp import brepgprop
        from OCC.Core.GProp import GProp_GProps

        for face in state.current_faces:
            try:
                props = GProp_GProps()
                brepgprop.SurfaceProperties(face, props)
                center = props.CentreOfMass()
                normal = calculate_face_normal(face)
                state.face_centers.append(center)
                state.face_normals.append(normal)

                obj = display.DisplayShape(center, color=rgb_color(1, 0, 0), update=False)
                if obj:
                    center_points_objects.append(obj)
                tri = display_coordinate_system(display, center, normal, size=50.0)
                if tri:
                    state.coordinate_systems.append(tri)
            except Exception as e:
                print("Error calculating patch center: {}".format(str(e)))

        display.FitAll()
        print("Centers calculated: {}".format(len(state.face_centers)))
        _update_workflow("Center calculation complete: {} centers".format(len(state.face_centers)))
    except Exception as e:
        print("Error getting centers: {}".format(str(e)))
        traceback.print_exc()


def generate_center_viewpoints(event=None):
    if not state.current_faces:
        show_topmost_message("Prompt", "Please segment faces first")
        return
    try:
        state.view_points.clear()
        state.center_view_points.clear()
        state.center_view_points_with_pose.clear()

        for obj in normal_line_objects:
            try:
                display.Context.Erase(obj, True)
            except Exception:
                pass
        normal_line_objects.clear()

        if not state.face_centers:
            get_centers()

        center_vps, center_vps_with_pose, line_objs = build_center_viewpoints(
            display, state.current_faces, state.face_centers, state.face_normals)

        state.center_view_points = center_vps
        state.view_points = list(center_vps)
        state.center_view_points_with_pose = center_vps_with_pose
        normal_line_objects.extend(line_objs)

        display.FitAll()
        print("Center viewpoints generated: {}".format(len(center_vps)))
        _update_workflow("Center viewpoints generated: {} viewpoints".format(len(center_vps)))
    except Exception as e:
        print("Error generating center viewpoints: {}".format(str(e)))
        traceback.print_exc()


def generate_candidate_viewpoints(event=None):
    if not state.current_faces:
        show_topmost_message("Prompt", "Please segment faces first")
        return
    try:
        state.view_points.clear()
        state.center_view_points.clear()
        state.center_view_points_with_pose.clear()
        state.view_points_with_pose.clear()

        for obj in normal_line_objects:
            try:
                display.Context.Erase(obj, True)
            except Exception:
                pass
        normal_line_objects.clear()

        if not state.face_centers:
            get_centers()

        all_vps, all_vps_with_pose, center_vps, line_objs, coord_sys = (
            build_candidate_viewpoints(
                display, state.current_faces, state.face_centers, state.face_normals))

        state.view_points = all_vps
        state.center_view_points = center_vps
        state.view_points_with_pose = all_vps_with_pose
        state.center_view_points_with_pose = [
            (vp, pose) for vp, pose in all_vps_with_pose[:len(center_vps)]]
        normal_line_objects.extend(line_objs)
        state.coordinate_systems.extend(coord_sys)

        display.FitAll()
        print("Candidate viewpoints generated: {} (including {} centres)".format(
            len(all_vps), len(center_vps)))
        _update_workflow("Candidate viewpoints generated: {} viewpoints".format(len(all_vps)))
    except Exception as e:
        print("Error generating candidate viewpoints: {}".format(str(e)))
        traceback.print_exc()


def filter_optimal_viewpoints(event=None):
    if not state.current_faces:
        show_topmost_message("Prompt", "Please segment faces first")
        return
    if not state.face_centers:
        show_topmost_message("Prompt", "Please get centers first")
        return
    try:
        state.optimal_viewpoints.clear()
        state.optimal_viewpoints_with_pose.clear()

        if not state.view_points:
            generate_candidate_viewpoints()
        if not state.view_points:
            show_topmost_message("Prompt", "No viewpoints available")
            return

        num_faces = len(state.current_faces)
        all_with_pose = state.view_points_with_pose

        for face_idx in range(num_faces):
            face_center = state.face_centers[face_idx]

            if all_with_pose:
                candidates = ordered_face_candidates(
                    all_with_pose, face_idx, num_faces, NUM_CANDIDATES_PER_FACE)
            else:
                candidates = []
                for vp in state.view_points:
                    d = point_distance(vp, face_center)
                    if abs(d - VIEWPOINT_DISTANCE) < 50:
                        normal = calculate_face_normal(state.current_faces[face_idx])
                        pose = calculate_viewpoint_pose(vp, normal, face_center)
                        candidates.append((vp, pose))

            if not candidates:
                print("Face {} has no candidate viewpoints".format(face_idx))
                continue

            best_vp, best_pose = choose_closest_to_distance(
                candidates, face_center, VIEWPOINT_DISTANCE, point_distance)

            state.optimal_viewpoints.append(best_vp)
            state.optimal_viewpoints_with_pose.append((best_vp, best_pose))

        # Display optimal viewpoints (red spheres)
        for vp in state.optimal_viewpoints:
            sphere = BRepPrimAPI_MakeSphere(vp, 5.0).Shape()
            display.DisplayShape(sphere, color=rgb_color(1, 0, 0), update=False)

        display.FitAll()
        print("Optimal viewpoints filtered: {}".format(len(state.optimal_viewpoints)))
        _update_workflow("Optimal viewpoints filtered: {}".format(len(state.optimal_viewpoints)))
    except Exception as e:
        print("Error filtering optimal viewpoints: {}".format(str(e)))
        traceback.print_exc()


# ---------------------------------------------------------------------------
# 9. Collision detection handlers
# ---------------------------------------------------------------------------

def toggle_collision_detection(event=None):
    global collision_detection_enabled
    collision_detection_enabled = not collision_detection_enabled
    status = "Enabled" if collision_detection_enabled else "Disabled"
    show_topmost_message("Collision Detection", "Collision detection is now {}".format(status))
    print("Collision detection: {}".format(status))


def set_sensor_parameters(event=None):
    result = get_sensor_parameters_dialog(current_config=state.sensor_size_config)
    if result is not None:
        state.sensor_size_config.update(result)
        _update_workflow("Sensor parameters updated: W={}, H={}, D={}".format(
            result['width'], result['height'], result['depth']))
        show_topmost_message("Success", "Sensor parameters updated: W={}, H={}, D={}".format(
            result['width'], result['height'], result['depth']))


def create_sensor_volumes_ui(event=None):
    viewpoints_with_pose = (
        state.optimal_viewpoints_with_pose
        or state.view_points_with_pose
        or state.center_view_points_with_pose)
    if not viewpoints_with_pose:
        show_topmost_message("Prompt", "Please generate viewpoints first")
        return

    try:
        # Clear previous sensor volume visuals
        for obj in state.sensor_volume_objects:
            try:
                display.Context.Remove(obj, True)
            except Exception:
                try:
                    display.Context.Erase(obj, True)
                except Exception:
                    pass
        state.sensor_volume_objects.clear()
        state.sensor_volumes_list.clear()
        display.Context.UpdateCurrentViewer()
        display.Repaint()

        created = 0
        for vp, pose in viewpoints_with_pose:
            if pose is None:
                continue
            try:
                box, vertices, color = create_sensor_volume(vp, pose, state.sensor_size_config)
                if box and vertices:
                    state.sensor_volumes_list.append((box, vertices, color))
                    obj = display.DisplayShape(box, color=color, transparency=0.3, update=False)
                    if obj:
                        state.sensor_volume_objects.append(obj)
                        created += 1
            except Exception as e:
                print("Failed to create sensor volume: {}".format(str(e)))

        display.Repaint()
        state.sensor_volumes_created = True
        print("Sensor volumes created: {}".format(created))
        _update_workflow("Sensor volume creation complete: {} volumes".format(created))
        show_topmost_message("Success", "Sensor volume creation complete, total {}".format(created))
    except Exception as e:
        print("Error creating sensor volumes: {}".format(str(e)))
        traceback.print_exc()
        show_topmost_message("Error", "Failed to create sensor volumes: {}".format(str(e)), type="error")


def generate_min_bounding_boxes(event=None):
    if not state.current_faces:
        show_topmost_message("Prompt", "Please segment faces first")
        return
    try:
        for obj in state.obb_visualizations:
            try:
                display.Context.Remove(obj, True)
            except Exception:
                try:
                    display.Context.Erase(obj, True)
                except Exception:
                    pass
        state.obb_visualizations.clear()
        state.face_obbs.clear()
        display.Context.UpdateCurrentViewer()
        display.Repaint()

        for face in state.current_faces:
            obb, box_shape, obb_color = generate_face_obb(face)
            if obb and box_shape:
                state.face_obbs.append(obb)
                vis = display.DisplayShape(box_shape, color=obb_color, transparency=0.7, update=False)
                if vis:
                    state.obb_visualizations.append(vis)

        display.Repaint()
        state.obb_boxes_generated = True
        print("OBB boxes generated: {}".format(len(state.face_obbs)))
        _update_workflow("Minimum bounding boxes generated: {} boxes".format(len(state.face_obbs)))
        show_topmost_message("Success", "Minimum bounding boxes generated, total {}".format(
            len(state.face_obbs)))
    except Exception as e:
        print("Error generating OBB: {}".format(str(e)))
        show_topmost_message("Error", "Failed to generate OBB: {}".format(str(e)))


def execute_collision_detection(event=None):
    if not state.view_points and not state.center_view_points:
        show_topmost_message("Prompt", "Please generate viewpoints first")
        return
    try:
        # Auto-generate OBBs if needed
        if not state.obb_boxes_generated:
            if not state.current_faces:
                show_topmost_message("Prompt", "Please segment faces first")
                return
            state.face_obbs.clear()
            for obj in state.obb_visualizations:
                try:
                    display.Context.Remove(obj, True)
                except Exception:
                    try:
                        display.Context.Erase(obj, True)
                    except Exception:
                        pass
            state.obb_visualizations.clear()
            display.Context.UpdateCurrentViewer()
            display.Repaint()

            for face in state.current_faces:
                obb, box_shape, obb_color = generate_face_obb(face)
                if obb and box_shape:
                    state.face_obbs.append(obb)
                    vis = display.DisplayShape(box_shape, color=obb_color, transparency=0.7, update=False)
                    if vis:
                        state.obb_visualizations.append(vis)
            display.Repaint()
            state.obb_boxes_generated = True
            _update_workflow("OBB boxes generated: {}".format(len(state.face_obbs)))

        if not state.sensor_volumes_created:
            show_topmost_message("Prompt", "Please create sensor volumes first")
            return

        if state.face_obbs and state.sensor_volumes_list:
            results = check_collision_sweep(state.face_obbs, state.sensor_volumes_list)
            summary = summarize_collision_results(results)
            show_topmost_message("Collision Result",
                "Detection complete: {} viewpoints, {} collisions detected".format(
                    summary.total, summary.collisions))
            state.collision_detection_executed = True
            _update_workflow("Collision detection complete: {}/{} collisions".format(
                summary.collisions, summary.total))
        else:
            show_topmost_message("Prompt", "No bounding boxes or sensor volumes available")
    except Exception as e:
        print("Error executing collision detection: {}".format(str(e)))
        show_topmost_message("Error", "Collision detection failed: {}".format(str(e)), type="error")


def demo_collision_detection(event=None):
    if not state.face_obbs:
        show_topmost_message("Prompt", "Please generate OBB bounding boxes first")
        return
    try:
        from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
        first_obb = state.face_obbs[0]
        obb_center = first_obb.Center()
        test_center = gp_Pnt(obb_center.X() + 10, obb_center.Y() + 10, obb_center.Z() + 10)
        test_cube = BRepPrimAPI_MakeBox(test_center, 200, 200, 200).Shape()
        display.DisplayShape(test_cube, color=rgb_color(1, 0, 0), update=False)
        display.Repaint()

        collision = any(
            math.sqrt((test_center.X() - obb.Center().X()) ** 2 +
                      (test_center.Y() - obb.Center().Y()) ** 2 +
                      (test_center.Z() - obb.Center().Z()) ** 2) < 100
            for obb in state.face_obbs)

        if collision:
            show_topmost_message("Demo Result", "Collision detected! Test cube intersects with OBB.")
        else:
            show_topmost_message("Demo Result", "No collision detected.")
    except Exception as e:
        print("Error demoing collision detection: {}".format(str(e)))
        traceback.print_exc()


# ---------------------------------------------------------------------------
# 10. Path planning handlers
# ---------------------------------------------------------------------------

def connect_sequentially(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message("Prompt", "Please filter optimal viewpoints first")
        return
    if not _confirm_path_prompt():
        return
    try:
        _delete_existing_path()
        n = len(state.optimal_viewpoints)
        if n < 2:
            show_topmost_message("Prompt", "At least 2 optimal viewpoints required")
            return
        state.optimal_path = list(range(n))
        dist_matrix = build_distance_matrix(state.optimal_viewpoints)
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        display.FitAll()
        _update_workflow("Sequential path complete, total length: {:.6f}".format(total))
        show_topmost_message("Success", "Sequential connection complete, total path length: {:.6f}".format(total))
    except Exception as e:
        print("Error in sequential path planning: {}".format(str(e)))
        traceback.print_exc()


def solve_with_greedy_algorithm(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message("Prompt", "Please filter optimal viewpoints first")
        return
    if not _confirm_path_prompt():
        return
    try:
        _delete_existing_path()
        state.optimal_path, dist_matrix = solve_greedy_open_path(state.optimal_viewpoints)
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        display.FitAll()
        _update_workflow("Greedy path complete, total length: {:.6f}".format(total))
        show_topmost_message("Success", "Greedy algorithm complete, total path length: {:.6f}".format(total))
    except Exception as e:
        print("Error in greedy algorithm: {}".format(str(e)))
        traceback.print_exc()


def solve_with_abc_algorithm(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message("Prompt", "Please filter optimal viewpoints first")
        return
    if not _confirm_path_prompt():
        return
    n = len(state.optimal_viewpoints)
    if n < 2:
        show_topmost_message("Prompt", "At least 2 optimal viewpoints required")
        return

    _delete_existing_path()
    abc_points = np.array([[p.X(), p.Y(), p.Z()] for p in state.optimal_viewpoints])
    set_status_message("ABC path planning started in the background")

    def on_success(result):
        state.optimal_path = list(result["best_tour"])
        state.last_path_length = result["best_length"]
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        display.FitAll()
        display.Repaint()
        _update_workflow("ABC path complete, total length: {:.6f}".format(result["best_length"]))
        show_topmost_message("Success", "ABC algorithm complete, total path length: {:.6f}".format(
            result["best_length"]))

    def on_error(error_text):
        print("Error in ABC path planning: {}".format(error_text))
        show_topmost_message("Error", "ABC path planning failed:\n{}".format(error_text), type="error")

    run_background_task("ABC Path Planning", "Executing ABC algorithm...",
                        lambda: run_abc_solver(abc_points), on_success, on_error)


def solve_with_mscga_algorithm(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message("Prompt", "Please filter optimal viewpoints first")
        return
    if not _confirm_path_prompt():
        return
    n = len(state.optimal_viewpoints)
    if n < 2:
        show_topmost_message("Prompt", "At least 2 optimal viewpoints required")
        return

    _delete_existing_path()
    set_status_message("MSCGA path planning started in the background")

    def on_success(result):
        state.optimal_path = list(result["best_tour"])
        state.last_path_length = result["best_length"]
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        display.FitAll()
        display.Repaint()
        _update_workflow("MSCGA path complete, total length: {:.6f}".format(result["best_length"]))
        show_topmost_message("Success", "MSCGA algorithm complete, total path length: {:.6f}".format(
            result["best_length"]))

    def on_error(error_text):
        print("Error in MSCGA path planning: {}".format(error_text))
        show_topmost_message("Error", "MSCGA path planning failed:\n{}".format(error_text), type="error")

    run_background_task("MSCGA Path Planning", "Executing MSCGA algorithm...",
                        lambda: run_mscga_solver(state.optimal_viewpoints), on_success, on_error)


def plan_path(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message("Prompt", "Please filter optimal viewpoints first")
        return
    try:
        _delete_existing_path()
        state.optimal_path, dist_matrix = solve_greedy_open_path(state.optimal_viewpoints)
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        display.FitAll()
        _update_workflow("Path planning complete, total length: {:.6f}".format(total))
    except Exception as e:
        print("Error in path planning: {}".format(str(e)))
        traceback.print_exc()


# ---------------------------------------------------------------------------
# 11. Export handler
# ---------------------------------------------------------------------------

def export_path_to_csv(event=None):
    if not state.optimal_viewpoints or not state.optimal_viewpoints_with_pose:
        show_topmost_message("Prompt", "Please filter optimal viewpoints first")
        return
    try:
        parent = get_main_window()
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            parent, "Save Path Points as CSV", "optimal_path_pose_list.csv",
            "CSV Files (*.csv);;All Files (*)")
        if not file_path:
            return

        count = write_path_pose_csv(file_path, state.optimal_viewpoints_with_pose, state.optimal_path)
        _update_workflow("Exported {} path points to CSV".format(count))
        print("Path points exported to CSV: {}".format(file_path))
    except Exception as e:
        print("Error exporting path points: {}".format(str(e)))
        traceback.print_exc()
        show_topmost_message("Error", "Error exporting path points: {}".format(str(e)), type="error")


# ---------------------------------------------------------------------------
# 12. Constrained speed planning and RoboDK integration
# ---------------------------------------------------------------------------

def load_scanner_tool_extrinsic(event=None):
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
        get_main_window(), "Load Scanner-to-Tool Extrinsic", "calibration",
        "JSON Files (*.json);;All Files (*)")
    if not file_path:
        return
    try:
        config = load_extrinsic_config(file_path)
        state.extrinsic_config = config
        state.extrinsic_config_path = file_path
        with open(file_path, "rb") as stream:
            state.extrinsic_config_sha256 = hashlib.sha256(stream.read()).hexdigest()
        state.speed_plan_result = None
        state.last_speed_csv_path = ""
        state.last_robodk_import.clear()
        relationship = ("T_flange_scanner (RoboDK TCP is scanner)"
                        if config.mapping_mode == "robodk_tcp_is_scanner"
                        else "T_tool_scanner (separate command tool)")
        message = ("Loaded tool mapping {}: status={}, mode={}, {}"
                   .format(config.config_id, config.calibration_status,
                           config.mapping_mode, relationship))
        _update_workflow(message)
        show_topmost_message(
            "Scanner-to-Tool Extrinsic",
            message + ("\n\nRoboDK simulation import is enabled. Physical mounting "
                       "validation is still required before production execution."
                       if config.validated and not config.physically_validated else
                       "\n\nRoboDK import is enabled for this physically validated calibration."
                       if config.physically_validated else
                       "\n\nThis configuration is not validated. Planning/export are allowed, "
                       "but RoboDK import remains blocked."),
            type="info" if config.physically_validated else "warning")
    except Exception as e:
        show_topmost_message("Error", "Invalid extrinsic configuration:\n{}".format(e),
                             type="error")


def clear_scanner_tool_extrinsic(event=None):
    state.extrinsic_config = None
    state.extrinsic_config_path = ""
    state.extrinsic_config_sha256 = ""
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    _update_workflow("Scanner-to-tool extrinsic cleared; RoboDK import is blocked")
    show_topmost_message(
        "Scanner-to-Tool Extrinsic",
        "Extrinsic cleared. Speed planning may still run on scanner poses for research, "
        "but RoboDK import is blocked until a validated calibration is loaded.",
        type="warning")


def _ordered_pose_records():
    if not state.optimal_viewpoints_with_pose:
        raise ValueError("No path poses are available")
    ordered = list(state.optimal_path) if state.optimal_path else list(range(len(state.optimal_viewpoints_with_pose)))
    records = []
    for order_index, viewpoint_index in enumerate(ordered, start=1):
        if viewpoint_index < 0 or viewpoint_index >= len(state.optimal_viewpoints_with_pose):
            raise IndexError("Path index out of range: {}".format(viewpoint_index))
        point, pose = state.optimal_viewpoints_with_pose[viewpoint_index]
        qw, qx, qy, qz = pose
        records.append({
            "index": order_index,
            "x": point.X(), "y": point.Y(), "z": point.Z(),
            "qw": qw, "qx": qx, "qy": qy, "qz": qz,
        })
    if state.extrinsic_config is not None:
        command_records, metadata = transform_pose_records(records, state.extrinsic_config)
        metadata["extrinsic_config_path"] = state.extrinsic_config_path
        metadata["extrinsic_config_sha256"] = state.extrinsic_config_sha256
        return command_records, metadata
    return records, {
        "extrinsic_applied": False,
        "extrinsic_validated": False,
        "extrinsic_config_id": "",
        "calibration_status": "missing",
        "source_pose_frame": "scanner",
        "command_pose_frame": "scanner_untransformed",
        "T_tool_scanner": None,
        "extrinsic_config_path": "",
        "extrinsic_config_sha256": "",
        "source_pose_records": [dict(record) for record in records],
    }


def plan_path_speeds(event=None):
    if not state.optimal_path or not state.optimal_viewpoints_with_pose:
        show_topmost_message("Prompt", "Please complete path planning before speed planning")
        return
    settings = get_speed_planning_settings(get_main_window())
    if not settings:
        return
    algorithm = settings.pop("algorithm")
    try:
        profile = ConstraintProfile(**settings)
        records, pose_metadata = _ordered_pose_records()
    except Exception as e:
        show_topmost_message("Error", "Invalid speed planning input: {}".format(e), type="error")
        return

    def work():
        result = plan_speed_profile(records, profile, algorithm)
        result.diagnostics.update(pose_metadata)
        if not pose_metadata["extrinsic_validated"]:
            result.warnings.append(
                "Scanner-to-tool extrinsic is missing or unvalidated; RoboDK import is blocked.")
        elif not pose_metadata.get("physical_calibration_validated", False):
            result.warnings.append(
                "RoboDK station TCP mapping is verified, but the physical scanner mounting "
                "has not been independently calibrated; use for simulation validation only.")
        return result

    def on_success(result):
        state.speed_plan_result = result
        state.last_speed_csv_path = ""
        state.last_robodk_import.clear()
        message = ("Speed planning complete: {} poses, {:.3f} s, algorithm={}, feasible={}"
                   .format(len(result.points), result.total_time, result.algorithm, result.feasible))
        _update_workflow(message)
        warning_text = "\n\n".join(result.warnings)
        show_topmost_message("Speed Planning Complete", message + "\n\n" + warning_text,
                             type="warning" if result.warnings else "info")

    def on_error(error_text):
        show_topmost_message("Error", "Speed planning failed:\n{}".format(error_text), type="error")

    run_background_task("Speed Planning", "Optimizing speed at every ordered pose...",
                        work,
                        on_success, on_error)


def export_speed_plan_to_csv(event=None):
    if state.speed_plan_result is None:
        show_topmost_message("Prompt", "Please run speed planning first")
        return
    file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
        get_main_window(), "Save Pose + Speed CSV", "planned_pose_speed.csv",
        "CSV Files (*.csv);;All Files (*)")
    if not file_path:
        return
    try:
        count = write_speed_plan_csv(file_path, state.speed_plan_result)
        state.last_speed_csv_path = file_path
        _update_workflow("Exported {} poses with speed parameters".format(count))
        show_topmost_message(
            "Success",
            "Pose + speed CSV exported:\n{}\n\nMetadata sidecar:\n{}.metadata.json".format(
                file_path, file_path))
    except Exception as e:
        show_topmost_message("Error", "Speed CSV export failed: {}".format(e), type="error")


def import_speed_plan_to_robodk(event=None):
    if state.speed_plan_result is None:
        show_topmost_message("Prompt", "Please run speed planning first")
        return
    if not state.speed_plan_result.feasible:
        show_topmost_message("Error", "Speed plan has constraint violations and cannot be imported", type="error")
        return
    if not state.speed_plan_result.diagnostics.get("extrinsic_validated", False):
        show_topmost_message(
            "Error",
            "RoboDK import is blocked: load a scanner-to-tool calibration whose "
            "calibration_status is validated, then rerun speed planning.",
            type="error")
        return
    settings = get_robodk_import_settings(get_main_window())
    if not settings:
        return
    if settings.get("replace"):
        answer = show_topmost_message(
            "Confirm Replace",
            "RoboDK same-name generated program/targets will be deleted and rebuilt. Continue?",
            type="question")
        if answer != "yes":
            return

    def on_success(summary):
        state.last_robodk_import = summary
        message = ("RoboDK import complete: program={}, poses={}, instructions={}"
                   .format(summary["program"], summary["pose_count"], summary["instruction_count"]))
        _update_workflow(message)
        show_topmost_message("RoboDK Import Complete", message)

    def on_error(error_text):
        show_topmost_message("Error", "RoboDK import failed:\n{}".format(error_text), type="error")

    run_background_task("RoboDK Import", "Creating Set Speed -> Move instructions...",
                        lambda: import_speed_plan(state.speed_plan_result, **settings),
                        on_success, on_error)


# ---------------------------------------------------------------------------
# 13. Toggle / layer handlers
# ---------------------------------------------------------------------------

def _toggle_flag(flag_name, label, required_condition=True, prompt=None):
    if not required_condition:
        show_topmost_message("Prompt", prompt or "Required data is not available")
        _sync_layer()
        return
    current = getattr(state, flag_name)
    setattr(state, flag_name, not current)
    action = "shown" if not current else "hidden"
    _do_render()
    set_status_message("{} {}".format(label, action))


def toggle_model_layer(event=None):
    _toggle_flag("show_model", "Model / patches",
                 state.current_shape is not None, "Please import a model first")

def toggle_face_centers(event=None):
    _toggle_flag("show_face_centers", "Face centers",
                 bool(state.face_centers), "Please get face centers first")

def toggle_normal_lines(event=None):
    _toggle_flag("show_normal_lines", "Normal lines",
                 bool(state.center_view_points), "Please generate center viewpoints first")

def toggle_all_viewpoints(event=None):
    _toggle_flag("show_all_viewpoints", "All viewpoints",
                 bool(state.view_points), "Please generate viewpoints first")

def toggle_optimal_viewpoints(event=None):
    _toggle_flag("show_optimal_viewpoints", "Optimal viewpoints",
                 bool(state.optimal_viewpoints), "Please filter optimal viewpoints first")

def toggle_planned_path(event=None):
    _toggle_flag("show_planned_path", "Planned path",
                 bool(state.optimal_path and state.optimal_viewpoints), "Please plan path first")

def toggle_sensor_volumes(event=None):
    _toggle_flag("show_sensor_volumes", "Sensor volumes",
                 bool(state.sensor_volumes_list), "Please create sensor volumes first")

def toggle_obb_boxes(event=None):
    _toggle_flag("show_obb_boxes", "OBB boxes",
                 bool(state.face_obbs), "Please generate OBB bounding boxes first")


# ---------------------------------------------------------------------------
# 13. Workflow / layer panel wrappers
# ---------------------------------------------------------------------------

def create_workflow_panel_ui(event=None):
    create_workflow_panel()

def create_layer_panel_ui(event=None):
    create_layer_panel({
        "model": toggle_model_layer,
        "face_centers": toggle_face_centers,
        "normal_lines": toggle_normal_lines,
        "all_viewpoints": toggle_all_viewpoints,
        "optimal_viewpoints": toggle_optimal_viewpoints,
        "planned_path": toggle_planned_path,
        "sensor_volumes": toggle_sensor_volumes,
        "obb_boxes": toggle_obb_boxes,
    })


# ---------------------------------------------------------------------------
# 14. Entry point
# ---------------------------------------------------------------------------

USAGE_TEXT_INLINE = """\
Usage Instructions:
  1. Use the mouse in the 3D viewer to rotate and zoom the model
  2. Please use functions in order:
     Import Model -> Segment Faces -> Get Centers -> Generate Viewpoints
     -> Filter Optimal Viewpoints -> Path Planning
  3. Click 'File' menu to import/clear models or exit
  4. Click 'Model Processing' menu for segmentation and viewpoint workflow
  5. Click 'Collision Detection' menu for sensor and OBB operations
  6. Click 'View' menu to show/hide layers
  7. Click 'Path Planning' menu for sequential/greedy/ABC/MSCGA planning
  8. Load a validated T_tool_scanner from 'Calibration' before production import
  9. After path ordering, use 'Speed Planning' to plan constrained per-pose speeds
 10. Export Pose+Speed CSV or import Set Speed -> Move pairs to RoboDK
 11. Click 'Help' menu to show usage instructions again
"""


def run():
    print("=== 3D Model Processing and Path Planning System ===")
    # Print usage to console instead of showing a blocking dialog at startup.
    # Users can access it later via Help -> show_usage_instructions.
    print(USAGE_TEXT_INLINE)

    # File menu
    add_menu("File")
    add_function_to_menu("File", import_model)
    add_function_to_menu("File", clear_model)
    add_function_to_menu("File", exit_program)

    # Model Processing menu
    add_menu("Model Processing")
    add_function_to_menu("Model Processing", select_single_face)
    add_function_to_menu("Model Processing", segment_faces)
    add_function_to_menu("Model Processing", get_centers)
    add_function_to_menu("Model Processing", generate_center_viewpoints)
    add_function_to_menu("Model Processing", generate_candidate_viewpoints)
    add_function_to_menu("Model Processing", filter_optimal_viewpoints)
    add_function_to_menu("Model Processing", plan_path)

    # Collision Detection menu
    add_menu("Collision Detection")
    add_function_to_menu("Collision Detection", toggle_collision_detection)
    add_function_to_menu("Collision Detection", set_sensor_parameters)
    add_function_to_menu("Collision Detection", create_sensor_volumes_ui)
    add_function_to_menu("Collision Detection", generate_min_bounding_boxes)
    add_function_to_menu("Collision Detection", execute_collision_detection)
    add_function_to_menu("Collision Detection", demo_collision_detection)

    # View menu
    add_menu("View")
    add_function_to_menu("View", toggle_model_layer)
    add_function_to_menu("View", toggle_face_centers)
    add_function_to_menu("View", toggle_normal_lines)
    add_function_to_menu("View", toggle_all_viewpoints)
    add_function_to_menu("View", toggle_optimal_viewpoints)
    add_function_to_menu("View", toggle_planned_path)
    add_function_to_menu("View", toggle_sensor_volumes)
    add_function_to_menu("View", toggle_obb_boxes)
    add_function_to_menu("View", create_workflow_panel_ui)
    add_function_to_menu("View", create_layer_panel_ui)

    # Path Planning menu
    add_menu("Path Planning")
    add_function_to_menu("Path Planning", connect_sequentially)
    add_function_to_menu("Path Planning", solve_with_greedy_algorithm)
    add_function_to_menu("Path Planning", solve_with_abc_algorithm)
    add_function_to_menu("Path Planning", solve_with_mscga_algorithm)

    # Speed Planning menu (must run after path ordering)
    add_menu("Speed Planning")
    add_function_to_menu("Speed Planning", plan_path_speeds)
    add_function_to_menu("Speed Planning", export_speed_plan_to_csv)
    add_function_to_menu("Speed Planning", import_speed_plan_to_robodk)

    # Calibration menu. A validated T_tool_scanner is required for RoboDK import.
    add_menu("Calibration")
    add_function_to_menu("Calibration", load_scanner_tool_extrinsic)
    add_function_to_menu("Calibration", clear_scanner_tool_extrinsic)

    # Export menu
    add_menu("Export")
    add_function_to_menu("Export", export_path_to_csv)
    add_function_to_menu("Export", export_speed_plan_to_csv)

    # Help menu
    add_menu("Help")
    add_function_to_menu("Help", create_workflow_panel_ui)
    add_function_to_menu("Help", create_layer_panel_ui)
    add_function_to_menu("Help", show_usage_instructions)

    # Install close-event interceptor
    close_filter = MainWindowCloseEvent(QtWidgets.QApplication.instance())
    QtWidgets.QApplication.instance().installEventFilter(close_filter)

    # Create panels by default
    create_workflow_panel_ui()
    create_layer_panel_ui()

    global _app_ready
    _app_ready = True

    start_display()


if __name__ == "__main__":
    run()
