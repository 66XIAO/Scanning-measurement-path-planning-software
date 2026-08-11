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
    calculate_face_normal, calculate_workpiece_coordinate_size,
    segment_model, generate_viewpoint,
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
    format_workflow_snapshot, retranslate_panels,
)
from workers import TaskRunner
from export_utils import write_path_pose_csv, write_speed_plan_csv
from speed_planning_core import ConstraintProfile, plan_speed_profile
from speed_planning_ui import get_speed_planning_settings, get_robodk_import_settings
from robodk_bridge import import_planned_path, import_speed_plan
from pose_transform import load_extrinsic_config, transform_pose_records
from cad_io import load_cad_shape
from i18n import (
    LocaleValidationError, set_language, set_language_from_file,
    subscribe_language_changed, tr,
)
from model_drop import install_model_drop_support
from surface_segmentation import is_surface_patch


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
                    _w.setObjectName("MainWindow")
                    _w.setWindowTitle(tr("app.title"))
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

# Stable menu IDs remain English because pythonOCC uses them as dictionary
# keys.  Only the visible titles/actions are retranslated at runtime.
_translated_menus = {}
_translated_actions = []
_language_subscription = None


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
                    tr("message.confirm_exit.title"),
                    tr("message.confirm_exit.body"),
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
        workpiece_coordinate_system=state.show_workpiece_coordinate_system,
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
        state.workpiece_coordinate_system_size,
        state.workpiece_coordinate_system_objects, state.coordinate_systems,
        state.optimal_path_objects,
        state.sensor_volume_objects, state.obb_visualizations,
        fit_all=fit_all,
    )


def _update_workflow(message=None):
    _update_ws(message, state)


def _sync_layer():
    _sync_layer_panel({
        "model": state.show_model,
        "workpiece_coordinate_system": state.show_workpiece_coordinate_system,
        "face_centers": state.show_face_centers,
        "normal_lines": state.show_normal_lines,
        "all_viewpoints": state.show_all_viewpoints,
        "optimal_viewpoints": state.show_optimal_viewpoints,
        "planned_path": state.show_planned_path,
        "sensor_volumes": state.show_sensor_volumes,
        "obb_boxes": state.show_obb_boxes,
    })


def _add_translated_menu(stable_name, translation_key):
    """Create one pythonOCC menu while keeping a stable internal key."""
    add_menu(stable_name)
    window = get_main_window()
    menu = getattr(window, "_menus", {}).get(stable_name) if window else None
    if menu is not None:
        _translated_menus[stable_name] = (menu, translation_key)
        menu.setTitle(tr(translation_key))
    return menu


def _add_translated_action(stable_menu_name, callback, translation_key):
    """Add an action and register its text for runtime retranslation."""
    add_function_to_menu(stable_menu_name, callback)
    window = get_main_window()
    menu = getattr(window, "_menus", {}).get(stable_menu_name) if window else None
    if menu is None or not menu.actions():
        return None
    action = menu.actions()[-1]
    action.setProperty("i18n_key", translation_key)
    action.setText(tr(translation_key))
    _translated_actions.append(action)
    return action


def _retranslate_main_ui(_locale=None):
    """Retranslate persistent widgets without resetting application state."""
    window = get_main_window()
    if window is not None:
        window.setObjectName("MainWindow")
        window.setWindowTitle(tr("app.title"))
    for menu, translation_key in _translated_menus.values():
        menu.setTitle(tr(translation_key))
    for action in _translated_actions:
        translation_key = action.property("i18n_key")
        if translation_key:
            action.setText(tr(str(translation_key)))
    retranslate_panels(state)


def switch_to_english(event=None):
    info = set_language("en")
    set_status_message(tr("language.switched", language=info.native_name))


def switch_to_chinese(event=None):
    info = set_language("zh_CN")
    set_status_message(tr("language.switched", language=info.native_name))


def load_language_json(event=None):
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
        get_main_window(), tr("language.file_dialog_title"), "locales",
        tr("language.file_filter"))
    if not file_path:
        return
    try:
        info = set_language_from_file(file_path)
        set_status_message(tr("language.switched", language=info.native_name))
    except (LocaleValidationError, OSError, ValueError) as exc:
        show_topmost_message(
            tr("common.error"),
            tr("language.load_failed", error=exc),
            type="error")


# ---------------------------------------------------------------------------
# 5. Background-task runner
# ---------------------------------------------------------------------------

def run_background_task(title, message, fn, on_success, on_error=None):
    def failure(error_text):
        if on_error:
            on_error(error_text)
        else:
            show_topmost_message(tr("common.error"), error_text, type="error")
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
            tr("common.prompt"),
            tr("message.path_visibility_prompt"),
            type="question")
        if result == "no":
            return False
        path_planning_prompt_confirmed = True
    return True


# ---------------------------------------------------------------------------
# 7. File menu handlers
# ---------------------------------------------------------------------------

def import_model(event=None):
    parent = get_main_window()
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
        parent, tr("file.import_model.title"), "",
        tr("file.import_model.filter"))
    if not file_path:
        return
    import_model_from_path(file_path)


def import_model_from_path(file_path):
    """Start the existing atomic background import for a chosen/dropped file."""
    global path_planning_prompt_confirmed
    file_path = os.path.abspath(os.fspath(file_path))

    ext = os.path.splitext(file_path)[1].lower()
    if ext not in ('.step', '.stp', '.iges', '.igs'):
        show_topmost_message(
            tr("common.error"),
            tr("message.import.unsupported", extension=ext),
            type="error")
        return

    def load_shape():
        shape = load_cad_shape(file_path)
        axis_size = calculate_workpiece_coordinate_size(shape)
        return shape, axis_size

    def on_success(import_result):
        global path_planning_prompt_confirmed
        new_shape, axis_size = import_result
        # Commit atomically only after parsing succeeds. Cancelling or a read
        # error leaves the previously loaded model and downstream state intact.
        display.EraseAll()
        state.reset_all()
        state.current_shape = new_shape
        state.workpiece_coordinate_system_size = axis_size
        path_planning_prompt_confirmed = False
        _do_render()
        _sync_layer()
        _update_workflow(tr(
            "message.import.success", filename=os.path.basename(file_path)))
        print("Model imported successfully: {}".format(os.path.basename(file_path)))

    def on_error(error_text):
        print("Error importing model: {}".format(error_text))
        show_topmost_message(
            tr("common.error"),
            tr("message.import.failed", error=error_text),
            type="error")

    set_status_message(tr(
        "message.import.loading", filename=os.path.basename(file_path)))
    run_background_task(tr("file.import_model.title"), tr("message.import.reading"),
                        load_shape, on_success, on_error)


def clear_model(event=None):
    global path_planning_prompt_confirmed
    result = show_topmost_message(
        tr("message.confirm_clear.title"),
        tr("message.confirm_clear.body"), type="question")
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

        _update_workflow(tr("message.clear.success"))
        show_topmost_message(tr("common.success"), tr("message.clear.success"))
    except Exception as e:
        print("Error clearing model: {}".format(str(e)))


def exit_program(event=None):
    try:
        result = show_topmost_message(
            tr("message.confirm_exit.title"),
            tr("message.confirm_exit.body"), type="question")
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
        show_topmost_message(tr("common.prompt"), tr("message.require.model"))
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
        _update_workflow(tr("message.face_selected"))


def segment_faces(event=None):
    if not state.current_shape:
        show_topmost_message(tr("common.prompt"), tr("message.require.model"))
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
                show_topmost_message(
                    tr("common.error"), tr("message.segmentation.empty"),
                    type="error")
                return
            state.current_faces = faces
            state.surface_patches = [face for face in faces if is_surface_patch(face)]
            state.invalidate_after_segmentation()
            state.last_segmentation_diagnostics = diagnostics
            _do_render()
            strategy = diagnostics.get("strategy", "equal_param")
            print("Segmentation complete: {} patches (u={}, v={}, strategy={}), "
                  "area ratio {:.6f}".format(
                      len(faces), u, v, strategy,
                      diagnostics.get("area_ratio", 0.0)))
            _update_workflow(tr(
                "message.segmentation.complete", count=len(faces),
                strategy=strategy,
                ratio=diagnostics.get("area_ratio", 0.0)))
            diagnostic_messages = (
                list(diagnostics.get("warnings", []))
                + list(diagnostics.get("input_warnings", [])))
            if diagnostic_messages:
                partial = bool(
                    diagnostics.get("area_rejected_splits")
                    or diagnostics.get("fallback_faces"))
                input_only = bool(diagnostics.get("input_warnings")) and not bool(
                    diagnostics.get("warnings"))
                if partial:
                    title_key = "dialog.segmentation_partial.title"
                    message_key = "message.segmentation.partial"
                elif input_only:
                    title_key = "dialog.input_geometry_warning.title"
                    message_key = "message.segmentation.diagnostics"
                else:
                    title_key = "dialog.segmentation_diagnostics.title"
                    message_key = "message.segmentation.diagnostics"
                show_topmost_message(
                    tr(title_key),
                    tr(message_key, details="\n".join(diagnostic_messages)),
                    type="warning")

        def on_error(error_text):
            print("Error segmenting faces: {}".format(error_text))
            show_topmost_message(
                tr("common.error"),
                tr("message.segmentation.failed", error=error_text),
                type="error")

        set_status_message(tr("message.segmentation.running"))
        run_background_task(tr("action.segment_faces"), tr("message.segmentation.task"),
                            lambda: segment_model(shape_to_segment, u, v,
                                                  return_diagnostics=True,
                                                  strategy="auto"),
                            on_success, on_error)
    except Exception as e:
        print("Error segmenting faces: {}".format(str(e)))
        traceback.print_exc()


def get_centers(event=None):
    if not state.current_faces:
        show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
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
                if is_surface_patch(face):
                    center = face.center
                    normal = calculate_face_normal(face)
                else:
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
        _update_workflow(tr(
            "message.centers.complete", count=len(state.face_centers)))
    except Exception as e:
        print("Error getting centers: {}".format(str(e)))
        traceback.print_exc()


def generate_center_viewpoints(event=None):
    if not state.current_faces:
        show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
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
        _update_workflow(tr(
            "message.center_viewpoints.complete", count=len(center_vps)))
    except Exception as e:
        print("Error generating center viewpoints: {}".format(str(e)))
        traceback.print_exc()


def generate_candidate_viewpoints(event=None):
    if not state.current_faces:
        show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
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
        _update_workflow(tr(
            "message.candidate_viewpoints.complete", count=len(all_vps)))
    except Exception as e:
        print("Error generating candidate viewpoints: {}".format(str(e)))
        traceback.print_exc()


def filter_optimal_viewpoints(event=None):
    if not state.current_faces:
        show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
        return
    if not state.face_centers:
        show_topmost_message(tr("common.prompt"), tr("message.require.centers"))
        return
    try:
        state.optimal_viewpoints.clear()
        state.optimal_viewpoints_with_pose.clear()

        if not state.view_points:
            generate_candidate_viewpoints()
        if not state.view_points:
            show_topmost_message(
                tr("common.prompt"), tr("message.viewpoints.none"))
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
        _update_workflow(tr(
            "message.optimal_viewpoints.complete",
            count=len(state.optimal_viewpoints)))
    except Exception as e:
        print("Error filtering optimal viewpoints: {}".format(str(e)))
        traceback.print_exc()


# ---------------------------------------------------------------------------
# 9. Collision detection handlers
# ---------------------------------------------------------------------------

def toggle_collision_detection(event=None):
    global collision_detection_enabled
    collision_detection_enabled = not collision_detection_enabled
    status = tr("common.enabled") if collision_detection_enabled else tr("common.disabled")
    show_topmost_message(
        tr("menu.collision_detection"),
        tr("message.collision.toggle", state=status))
    print("Collision detection: {}".format(status))


def set_sensor_parameters(event=None):
    result = get_sensor_parameters_dialog(current_config=state.sensor_size_config)
    if result is not None:
        state.sensor_size_config.update(result)
        message = tr(
            "message.sensor.updated", width=result['width'],
            height=result['height'], depth=result['depth'])
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)


def create_sensor_volumes_ui(event=None):
    viewpoints_with_pose = (
        state.optimal_viewpoints_with_pose
        or state.view_points_with_pose
        or state.center_view_points_with_pose)
    if not viewpoints_with_pose:
        show_topmost_message(tr("common.prompt"), tr("message.require.viewpoints"))
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
        message = tr("message.sensor_volumes.complete", count=created)
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)
    except Exception as e:
        print("Error creating sensor volumes: {}".format(str(e)))
        traceback.print_exc()
        show_topmost_message(
            tr("common.error"),
            tr("message.sensor_volumes.failed", error=e),
            type="error")


def generate_min_bounding_boxes(event=None):
    if not state.current_faces:
        show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
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
        message = tr("message.obb.complete", count=len(state.face_obbs))
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)
    except Exception as e:
        print("Error generating OBB: {}".format(str(e)))
        show_topmost_message(
            tr("common.error"), tr("message.obb.failed", error=e),
            type="error")


def execute_collision_detection(event=None):
    if not state.view_points and not state.center_view_points:
        show_topmost_message(tr("common.prompt"), tr("message.require.viewpoints"))
        return
    try:
        # Auto-generate OBBs if needed
        if not state.obb_boxes_generated:
            if not state.current_faces:
                show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
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
            _update_workflow(tr(
                "message.obb.complete", count=len(state.face_obbs)))

        if not state.sensor_volumes_created:
            show_topmost_message(
                tr("common.prompt"), tr("message.require.sensor_volumes"))
            return

        if state.face_obbs and state.sensor_volumes_list:
            results = check_collision_sweep(state.face_obbs, state.sensor_volumes_list)
            summary = summarize_collision_results(results)
            message = tr(
                "message.collision.complete", collisions=summary.collisions,
                total=summary.total)
            show_topmost_message(tr("menu.collision_detection"), message)
            state.collision_detection_executed = True
            _update_workflow(message)
        else:
            show_topmost_message(
                tr("common.prompt"), tr("message.collision.inputs_missing"))
    except Exception as e:
        print("Error executing collision detection: {}".format(str(e)))
        show_topmost_message(
            tr("common.error"), tr("message.collision.failed", error=e),
            type="error")


def demo_collision_detection(event=None):
    if not state.face_obbs:
        show_topmost_message(tr("common.prompt"), tr("message.require.obb"))
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
            show_topmost_message(
                tr("dialog.collision_demo.title"),
                tr("message.collision.demo_detected"))
        else:
            show_topmost_message(
                tr("dialog.collision_demo.title"),
                tr("message.collision.demo_clear"))
    except Exception as e:
        print("Error demoing collision detection: {}".format(str(e)))
        traceback.print_exc()


# ---------------------------------------------------------------------------
# 10. Path planning handlers
# ---------------------------------------------------------------------------

def connect_sequentially(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.optimal_viewpoints"))
        return
    if not _confirm_path_prompt():
        return
    try:
        _delete_existing_path()
        n = len(state.optimal_viewpoints)
        if n < 2:
            show_topmost_message(
                tr("common.prompt"), tr("message.require.two_viewpoints"))
            return
        state.optimal_path = list(range(n))
        dist_matrix = build_distance_matrix(state.optimal_viewpoints)
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        display.FitAll()
        message = tr("message.path.sequential_complete", length=total)
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)
    except Exception as e:
        print("Error in sequential path planning: {}".format(str(e)))
        traceback.print_exc()


def solve_with_greedy_algorithm(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.optimal_viewpoints"))
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
        message = tr("message.path.greedy_complete", length=total)
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)
    except Exception as e:
        print("Error in greedy algorithm: {}".format(str(e)))
        traceback.print_exc()


def solve_with_abc_algorithm(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.optimal_viewpoints"))
        return
    if not _confirm_path_prompt():
        return
    n = len(state.optimal_viewpoints)
    if n < 2:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.two_viewpoints"))
        return

    _delete_existing_path()
    abc_points = np.array([[p.X(), p.Y(), p.Z()] for p in state.optimal_viewpoints])
    set_status_message(tr("message.path.abc_running"))

    def on_success(result):
        state.optimal_path = list(result["best_tour"])
        state.last_path_length = result["best_length"]
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        display.FitAll()
        display.Repaint()
        message = tr("message.path.abc_complete", length=result["best_length"])
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)

    def on_error(error_text):
        print("Error in ABC path planning: {}".format(error_text))
        show_topmost_message(
            tr("common.error"),
            tr("message.path.abc_failed", error=error_text),
            type="error")

    run_background_task(tr("action.solve_abc"),
                        tr("message.path.abc_executing"),
                        lambda: run_abc_solver(abc_points), on_success, on_error)


def solve_with_mscga_algorithm(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.optimal_viewpoints"))
        return
    if not _confirm_path_prompt():
        return
    n = len(state.optimal_viewpoints)
    if n < 2:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.two_viewpoints"))
        return

    _delete_existing_path()
    set_status_message(tr("message.path.mscga_running"))

    def on_success(result):
        state.optimal_path = list(result["best_tour"])
        state.last_path_length = result["best_length"]
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        display.FitAll()
        display.Repaint()
        message = tr("message.path.mscga_complete", length=result["best_length"])
        _update_workflow(message)
        show_topmost_message(tr("common.success"), message)

    def on_error(error_text):
        print("Error in MSCGA path planning: {}".format(error_text))
        show_topmost_message(
            tr("common.error"),
            tr("message.path.mscga_failed", error=error_text),
            type="error")

    run_background_task(tr("action.solve_mscga"),
                        tr("message.path.mscga_executing"),
                        lambda: run_mscga_solver(state.optimal_viewpoints), on_success, on_error)


def plan_path(event=None):
    if not state.optimal_viewpoints:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.optimal_viewpoints"))
        return
    try:
        _delete_existing_path()
        state.optimal_path, dist_matrix = solve_greedy_open_path(state.optimal_viewpoints)
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        display.FitAll()
        _update_workflow(tr("message.path.complete", length=total))
    except Exception as e:
        print("Error in path planning: {}".format(str(e)))
        traceback.print_exc()


# ---------------------------------------------------------------------------
# 11. Export handler
# ---------------------------------------------------------------------------

def export_path_to_csv(event=None):
    if not state.optimal_viewpoints or not state.optimal_viewpoints_with_pose:
        show_topmost_message(
            tr("common.prompt"), tr("message.require.optimal_viewpoints"))
        return
    try:
        parent = get_main_window()
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
            parent, tr("file.export_path.title"),
            tr("file.export_path.default_name"), tr("file.csv_filter"))
        if not file_path:
            return

        count = write_path_pose_csv(file_path, state.optimal_viewpoints_with_pose, state.optimal_path)
        _update_workflow(tr("message.export.path_complete", path=file_path))
        print("Path points exported to CSV: {}".format(file_path))
    except Exception as e:
        print("Error exporting path points: {}".format(str(e)))
        traceback.print_exc()
        show_topmost_message(
            tr("common.error"),
            tr("message.export.path_failed", error=e),
            type="error")


# ---------------------------------------------------------------------------
# 12. Constrained speed planning and RoboDK integration
# ---------------------------------------------------------------------------

def load_scanner_tool_extrinsic(event=None):
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
        get_main_window(), tr("file.load_extrinsic.title"), "calibration",
        tr("file.json_filter"))
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
        relationship = tr(
            "message.extrinsic.relationship_scanner_tcp"
            if config.mapping_mode == "robodk_tcp_is_scanner"
            else "message.extrinsic.relationship_separate_tool")
        message = tr(
            "message.extrinsic.loaded", config=config.config_id,
            status=config.calibration_status, mode=config.mapping_mode,
            relationship=relationship)
        _update_workflow(message)
        show_topmost_message(
            tr("dialog.extrinsic.title"),
            message + "\n\n" + tr(
                "message.extrinsic.simulation_only"
                if config.validated and not config.physically_validated else
                "message.extrinsic.physically_validated"
                if config.physically_validated else
                "message.extrinsic.unvalidated"),
            type="info" if config.physically_validated else "warning")
    except Exception as e:
        show_topmost_message(
            tr("common.error"), tr("message.extrinsic.invalid", error=e),
            type="error")


def clear_scanner_tool_extrinsic(event=None):
    state.extrinsic_config = None
    state.extrinsic_config_path = ""
    state.extrinsic_config_sha256 = ""
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    _update_workflow(tr("message.extrinsic.cleared_status"))
    show_topmost_message(
        tr("dialog.extrinsic.title"),
        tr("message.extrinsic.cleared_body"),
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


def import_planned_path_to_robodk(event=None):
    """Import ordered, calibrated poses without creating speed commands."""
    if not state.optimal_path or not state.optimal_viewpoints_with_pose:
        show_topmost_message(tr("common.prompt"), tr("message.require.path"))
        return
    try:
        records, pose_metadata = _ordered_pose_records()
    except Exception as exc:
        show_topmost_message(
            tr("common.error"),
            tr("message.path.input_invalid", error=exc),
            type="error")
        return
    if not pose_metadata.get("extrinsic_validated", False):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.path_calibration_required"),
            type="error")
        return

    settings = get_robodk_import_settings(get_main_window(), import_kind="path")
    if not settings:
        return
    if settings.get("replace"):
        answer = show_topmost_message(
            tr("message.robodk.confirm_replace.title"),
            tr("message.robodk.confirm_replace.body"),
            type="question")
        if answer != "yes":
            return

    def on_success(summary):
        state.last_robodk_import = summary
        message = tr(
            "message.robodk.complete",
            program=summary["program"],
            poses=summary["pose_count"],
            instructions=summary["instruction_count"])
        _update_workflow(message)
        show_topmost_message(tr("dialog.robodk.path_title"), message)

    def on_error(error_text):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.path_failed", error=error_text),
            type="error")

    run_background_task(
        tr("dialog.robodk.path_title"),
        tr("message.robodk.path_running"),
        lambda: import_planned_path(records, pose_metadata, **settings),
        on_success, on_error)


def plan_path_speeds(event=None):
    if not state.optimal_path or not state.optimal_viewpoints_with_pose:
        show_topmost_message(tr("common.prompt"), tr("message.require.path"))
        return
    settings = get_speed_planning_settings(get_main_window())
    if not settings:
        return
    algorithm = settings.pop("algorithm")
    try:
        profile = ConstraintProfile(**settings)
        records, pose_metadata = _ordered_pose_records()
    except Exception as e:
        show_topmost_message(
            tr("common.error"),
            tr("message.speed.input_invalid", error=e),
            type="error")
        return

    def work():
        result = plan_speed_profile(records, profile, algorithm)
        result.diagnostics.update(pose_metadata)
        if not pose_metadata["extrinsic_validated"]:
            result.warnings.append(tr(
                "message.speed.extrinsic_missing_warning"))
        elif not pose_metadata.get("physical_calibration_validated", False):
            result.warnings.append(tr(
                "message.speed.physical_unvalidated_warning"))
        return result

    def on_success(result):
        state.speed_plan_result = result
        state.last_speed_csv_path = ""
        state.last_robodk_import.clear()
        message = tr(
            "message.speed.complete", count=len(result.points),
            seconds=result.total_time, algorithm=result.algorithm,
            feasible=tr("common.yes") if result.feasible else tr("common.no"))
        _update_workflow(message)
        warning_text = "\n\n".join(result.warnings)
        show_topmost_message(tr("dialog.speed.title"), message + "\n\n" + warning_text,
                             type="warning" if result.warnings else "info")

    def on_error(error_text):
        show_topmost_message(
            tr("common.error"),
            tr("message.speed.failed", error=error_text),
            type="error")

    run_background_task(tr("menu.speed_planning"), tr("message.speed.running"),
                        work,
                        on_success, on_error)


def export_speed_plan_to_csv(event=None):
    if state.speed_plan_result is None:
        show_topmost_message(tr("common.prompt"), tr("message.require.speed_plan"))
        return
    file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
        get_main_window(), tr("file.export_speed.title"),
        tr("file.export_speed.default_name"), tr("file.csv_filter"))
    if not file_path:
        return
    try:
        count = write_speed_plan_csv(file_path, state.speed_plan_result)
        state.last_speed_csv_path = file_path
        message = tr("message.export.speed_complete", path=file_path)
        _update_workflow(message)
        show_topmost_message(
            tr("common.success"), message)
    except Exception as e:
        show_topmost_message(
            tr("common.error"),
            tr("message.export.speed_failed", error=e),
            type="error")


def import_speed_plan_to_robodk(event=None):
    if state.speed_plan_result is None:
        show_topmost_message(tr("common.prompt"), tr("message.require.speed_plan"))
        return
    if not state.speed_plan_result.feasible:
        show_topmost_message(
            tr("common.error"), tr("message.robodk.constraint_violation"),
            type="error")
        return
    if not state.speed_plan_result.diagnostics.get("extrinsic_validated", False):
        show_topmost_message(
            tr("common.error"), tr("message.robodk.calibration_required"),
            type="error")
        return
    settings = get_robodk_import_settings(get_main_window(), import_kind="speed")
    if not settings:
        return
    if settings.get("replace"):
        answer = show_topmost_message(
            tr("message.robodk.confirm_replace.title"),
            tr("message.robodk.confirm_replace.body"),
            type="question")
        if answer != "yes":
            return

    def on_success(summary):
        state.last_robodk_import = summary
        message = tr(
            "message.robodk.complete", program=summary["program"],
            poses=summary["pose_count"],
            instructions=summary["instruction_count"])
        _update_workflow(message)
        show_topmost_message(tr("dialog.robodk.speed_title"), message)

    def on_error(error_text):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.speed_failed", error=error_text),
            type="error")

    run_background_task(tr("dialog.robodk.speed_title"),
                        tr("message.robodk.speed_running"),
                        lambda: import_speed_plan(state.speed_plan_result, **settings),
                        on_success, on_error)


# ---------------------------------------------------------------------------
# 13. Toggle / layer handlers
# ---------------------------------------------------------------------------

def _toggle_flag(flag_name, layer_key, required_condition=True, prompt_key=None):
    if not required_condition:
        show_topmost_message(
            tr("common.prompt"),
            tr(prompt_key) if prompt_key else tr("message.require.data"))
        _sync_layer()
        return
    current = getattr(state, flag_name)
    setattr(state, flag_name, not current)
    action = tr("common.shown") if not current else tr("common.hidden")
    _do_render()
    set_status_message(tr(
        "message.layer_toggled", layer=tr("layer." + layer_key), state=action))


def toggle_model_layer(event=None):
    _toggle_flag("show_model", "model",
                 state.current_shape is not None, "message.require.model")

def toggle_workpiece_coordinate_system(event=None):
    _toggle_flag(
        "show_workpiece_coordinate_system", "workpiece_coordinate_system",
        state.current_shape is not None, "message.require.model")

def toggle_face_centers(event=None):
    _toggle_flag("show_face_centers", "face_centers",
                 bool(state.face_centers), "message.require.centers")

def toggle_normal_lines(event=None):
    _toggle_flag("show_normal_lines", "normal_lines",
                 bool(state.center_view_points), "message.require.viewpoints")

def toggle_all_viewpoints(event=None):
    _toggle_flag("show_all_viewpoints", "all_viewpoints",
                 bool(state.view_points), "message.require.viewpoints")

def toggle_optimal_viewpoints(event=None):
    _toggle_flag("show_optimal_viewpoints", "optimal_viewpoints",
                 bool(state.optimal_viewpoints), "message.require.optimal_viewpoints")

def toggle_planned_path(event=None):
    _toggle_flag("show_planned_path", "planned_path",
                 bool(state.optimal_path and state.optimal_viewpoints),
                 "message.require.path")

def toggle_sensor_volumes(event=None):
    _toggle_flag("show_sensor_volumes", "sensor_volumes",
                 bool(state.sensor_volumes_list), "message.require.sensor_volumes")

def toggle_obb_boxes(event=None):
    _toggle_flag("show_obb_boxes", "obb_boxes",
                 bool(state.face_obbs), "message.require.obb")


# ---------------------------------------------------------------------------
# 13. Workflow / layer panel wrappers
# ---------------------------------------------------------------------------

def create_workflow_panel_ui(event=None):
    create_workflow_panel()

def create_layer_panel_ui(event=None):
    create_layer_panel({
        "model": toggle_model_layer,
        "workpiece_coordinate_system": toggle_workpiece_coordinate_system,
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
    global _language_subscription
    print("=== 3D Model Processing and Path Planning System ===")
    # Print usage to console instead of showing a blocking dialog at startup.
    # Users can access it later via Help -> show_usage_instructions.
    print(USAGE_TEXT_INLINE)

    # Menu IDs are stable; visible labels are read from the active JSON catalog.
    _add_translated_menu("File", "menu.file")
    _add_translated_action("File", import_model, "action.import_model")
    _add_translated_action("File", clear_model, "action.clear_model")
    _add_translated_action("File", exit_program, "action.exit_program")

    # Model Processing menu
    _add_translated_menu("Model Processing", "menu.model_processing")
    _add_translated_action("Model Processing", select_single_face,
                           "action.select_single_face")
    _add_translated_action("Model Processing", segment_faces, "action.segment_faces")
    _add_translated_action("Model Processing", get_centers, "action.get_centers")
    _add_translated_action("Model Processing", generate_center_viewpoints,
                           "action.generate_center_viewpoints")
    _add_translated_action("Model Processing", generate_candidate_viewpoints,
                           "action.generate_candidate_viewpoints")
    _add_translated_action("Model Processing", filter_optimal_viewpoints,
                           "action.filter_optimal_viewpoints")
    _add_translated_action("Model Processing", plan_path, "action.plan_path")

    # Collision Detection menu
    _add_translated_menu("Collision Detection", "menu.collision_detection")
    _add_translated_action("Collision Detection", toggle_collision_detection,
                           "action.toggle_collision_detection")
    _add_translated_action("Collision Detection", set_sensor_parameters,
                           "action.set_sensor_parameters")
    _add_translated_action("Collision Detection", create_sensor_volumes_ui,
                           "action.create_sensor_volumes")
    _add_translated_action("Collision Detection", generate_min_bounding_boxes,
                           "action.generate_min_bounding_boxes")
    _add_translated_action("Collision Detection", execute_collision_detection,
                           "action.execute_collision_detection")
    _add_translated_action("Collision Detection", demo_collision_detection,
                           "action.demo_collision_detection")

    # View menu
    _add_translated_menu("View", "menu.view")
    _add_translated_action("View", toggle_model_layer, "action.toggle_model_layer")
    _add_translated_action(
        "View", toggle_workpiece_coordinate_system,
        "action.toggle_workpiece_coordinate_system")
    _add_translated_action("View", toggle_face_centers, "action.toggle_face_centers")
    _add_translated_action("View", toggle_normal_lines, "action.toggle_normal_lines")
    _add_translated_action("View", toggle_all_viewpoints, "action.toggle_all_viewpoints")
    _add_translated_action("View", toggle_optimal_viewpoints,
                           "action.toggle_optimal_viewpoints")
    _add_translated_action("View", toggle_planned_path, "action.toggle_planned_path")
    _add_translated_action("View", toggle_sensor_volumes,
                           "action.toggle_sensor_volumes")
    _add_translated_action("View", toggle_obb_boxes, "action.toggle_obb_boxes")
    _add_translated_action("View", create_workflow_panel_ui,
                           "action.create_workflow_panel")
    _add_translated_action("View", create_layer_panel_ui,
                           "action.create_layer_panel")

    # Path Planning menu
    _add_translated_menu("Path Planning", "menu.path_planning")
    _add_translated_action("Path Planning", connect_sequentially,
                           "action.connect_sequentially")
    _add_translated_action("Path Planning", solve_with_greedy_algorithm,
                           "action.solve_greedy")
    _add_translated_action("Path Planning", solve_with_abc_algorithm,
                           "action.solve_abc")
    _add_translated_action("Path Planning", solve_with_mscga_algorithm,
                           "action.solve_mscga")
    _add_translated_action("Path Planning", import_planned_path_to_robodk,
                           "action.import_planned_path_robodk")

    # Speed Planning menu (must run after path ordering)
    _add_translated_menu("Speed Planning", "menu.speed_planning")
    _add_translated_action("Speed Planning", plan_path_speeds,
                           "action.plan_path_speeds")
    _add_translated_action("Speed Planning", export_speed_plan_to_csv,
                           "action.export_speed_plan_csv")
    _add_translated_action("Speed Planning", import_speed_plan_to_robodk,
                           "action.import_speed_plan_robodk")

    # Calibration menu. A validated T_tool_scanner is required for RoboDK import.
    _add_translated_menu("Calibration", "menu.calibration")
    _add_translated_action("Calibration", load_scanner_tool_extrinsic,
                           "action.load_extrinsic")
    _add_translated_action("Calibration", clear_scanner_tool_extrinsic,
                           "action.clear_extrinsic")

    # Export menu
    _add_translated_menu("Export", "menu.export")
    _add_translated_action("Export", export_path_to_csv, "action.export_path_csv")
    _add_translated_action("Export", export_speed_plan_to_csv,
                           "action.export_speed_plan_csv")

    # Language catalog menu. Built-in choices and external UTF-8 JSON are supported.
    _add_translated_menu("Language", "language.menu")
    _add_translated_action("Language", switch_to_english, "language.english")
    _add_translated_action("Language", switch_to_chinese, "language.chinese")
    _add_translated_action("Language", load_language_json, "language.load_json")

    # Help menu
    _add_translated_menu("Help", "menu.help")
    _add_translated_action("Help", create_workflow_panel_ui,
                           "action.create_workflow_panel")
    _add_translated_action("Help", create_layer_panel_ui,
                           "action.create_layer_panel")
    _add_translated_action("Help", show_usage_instructions, "action.show_usage")

    if _language_subscription is None:
        _language_subscription = subscribe_language_changed(_retranslate_main_ui)

    # Install close-event interceptor
    close_filter = MainWindowCloseEvent(QtWidgets.QApplication.instance())
    QtWidgets.QApplication.instance().installEventFilter(close_filter)

    # Create panels by default
    create_workflow_panel_ui()
    create_layer_panel_ui()
    _retranslate_main_ui()

    # Accept one local STEP/IGES model dropped directly on the 3D canvas.
    window = get_main_window()
    viewer = getattr(window, "canva", None) if window is not None else None
    if viewer is not None:
        install_model_drop_support(
            QtCore, viewer, import_model_from_path,
            show_error=lambda message: show_topmost_message(
                tr("file.import_model.title"), message, type="error"),
            set_status=set_status_message,
            translate=tr)

    global _app_ready
    _app_ready = True

    start_display()


if __name__ == "__main__":
    run()
