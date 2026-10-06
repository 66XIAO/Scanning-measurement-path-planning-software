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
import OCC

from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.gp import gp_Pnt, gp_Vec
from OCC.Display.OCCViewer import rgb_color
from OCC.Display.SimpleGui import init_display
from OCC.Display import backend as occ_backend
from OCC.Display.backend import get_qt_modules

from config import (
    VIEWPOINT_DISTANCE, NUM_CANDIDATES_PER_FACE, ZENITH_ANGLE_DEG,
    DEFAULT_ABC_CONFIG, DEFAULT_MSCGA_CONFIG,
)
from state import AppState, ViewpointRecord
from geometry import (
    calculate_face_normal, calculate_workpiece_coordinate_size,
    segment_model, generate_viewpoint,
    generate_face_obb, ConvertBndToShape,
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
from renderer import (
    LayerVisibility,
    clear_interactive_selection,
    draw_path_edges,
    erase_all_safely,
    render_scene as _render_scene,
)
from ui_panels import (
    show_topmost_message, get_main_window, set_status_message,
    create_workflow_panel, create_layer_panel, update_workflow_status as _update_ws,
    sync_layer_panel as _sync_layer_panel, get_user_segment_params,
    get_sensor_parameters_dialog, show_usage_instructions, build_workflow_snapshot,
    format_workflow_snapshot, retranslate_panels,
    create_operation_panel, update_operation_panel,
)
from workers import TaskRunner, supports_background_occ_objects
from export_utils import write_path_pose_csv, write_speed_plan_csv
from speed_planning_core import ConstraintProfile, plan_speed_profile
from speed_planning_ui import (
    get_speed_planning_settings, get_robodk_import_settings,
    get_robodk_mapping_settings,
)
from robodk_bridge import (
    analyze_ur10_reachability, capture_robodk_station_mapping,
    import_planned_path, import_speed_plan,
)
from pose_transform import (
    load_extrinsic_config, save_extrinsic_config, transform_pose_records,
)
from reachability_repair import (
    build_candidate_lookup, build_ordered_path_records, build_viewpoint_records,
    compute_path_signature, repair_unreachable_points,
)
from cad_io import load_cad_shape
from i18n import (
    LocaleValidationError, set_language, set_language_from_file,
    subscribe_language_changed, tr,
)
from model_drop import install_model_drop_support
from surface_segmentation import is_surface_patch
from ui_theme import (
    apply_application_theme, create_primary_toolbar, decorate_action,
    retranslate_ribbon, ribbon_page, set_ribbon_size,
)
from scene_style import ERROR, configure_viewer
from workstation_io import (
    capture_workstation, load_workstation, save_workstation,
)
from ui_workspace import WorkspaceUI
from local_frame_style import read_local_frame_style


# ---------------------------------------------------------------------------
# 1. OCC viewer initialisation (must happen before any Qt widget work)
# ---------------------------------------------------------------------------

try:
    import PyQt5  # noqa: F401
    # pythonocc 7.4 names this backend ``qt-pyqt5``; newer releases expose
    # their own constant as well, so prefer the installed version's token.
    QT_BACKEND = getattr(occ_backend, "PYQT5", "pyqt5")
except ImportError:
    import PySide6  # noqa: F401
    QT_BACKEND = getattr(occ_backend, "PYSIDE6", "pyside6")

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
PYTHONOCC_VERSION = getattr(OCC, "VERSION", "unknown")
OCC_BACKGROUND_OBJECTS_SUPPORTED = supports_background_occ_objects(
    PYTHONOCC_VERSION)

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
_face_callback_registered = False
_awaiting_face_selection = False
_workstation_io_active = False
_workspace_ui = None
_recent_workstations_menu = None


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
                proceed = _confirm_discard_or_save()
            finally:
                _closing_dialog_active = False
            if not proceed:
                event.ignore()
                return True
            if _workspace_ui is not None:
                _workspace_ui.save_layout()
        return super(MainWindowCloseEvent, self).eventFilter(obj, event)


# ---------------------------------------------------------------------------
# 4. Rendering helper (bridges state → renderer.render_scene)
# ---------------------------------------------------------------------------

def _clear_selected_face_highlight(update=False):
    """Remove the retained face highlight before a whole-scene erase."""
    clear_interactive_selection(display)
    highlight = state.selected_face_highlight
    if highlight is None:
        return
    try:
        display.Context.Remove(highlight, bool(update))
    except Exception:
        try:
            display.Context.Erase(highlight, bool(update))
        except Exception:
            pass
    finally:
        state.selected_face_highlight = None


def _do_render(fit_all=True):
    """Centralised scene redraw from current *state*."""
    _clear_selected_face_highlight(update=False)
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
        selected_face=state.selected_face,
        local_frame_style=read_local_frame_style(_workspace_ui.settings),
    )


def _update_workflow(message=None):
    _update_ws(message, state)
    update_operation_panel(state)


def _update_window_title():
    window = get_main_window()
    if window is None:
        return
    name = (os.path.basename(state.workstation_path)
            if state.workstation_path else tr("workstation.untitled"))
    dirty = " *" if state.workstation_dirty else ""
    window.setWindowTitle("{} - {}{}".format(tr("app.title"), name, dirty))


def _record_operation(operation, parameters=None, status="completed", summary=""):
    state.record_operation(operation, parameters, status, summary)
    _update_window_title()
    update_operation_panel(state)


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
    decorate_action(QtGui, action, translation_key)
    _translated_actions.append(action)
    return action


def _retranslate_main_ui(_locale=None):
    """Retranslate persistent widgets without resetting application state."""
    window = get_main_window()
    if window is not None:
        window.setObjectName("MainWindow")
        _update_window_title()
    for menu, translation_key in _translated_menus.values():
        menu.setTitle(tr(translation_key))
    for action in _translated_actions:
        translation_key = action.property("i18n_key")
        if translation_key:
            action.setText(tr(str(translation_key)))
    retranslate_ribbon()
    retranslate_panels(state)
    if _workspace_ui is not None:
        _workspace_ui.refresh_favorites()
        if _workspace_ui.search is not None:
            _workspace_ui.search.setPlaceholderText(tr("ui.search.placeholder"))
    if _recent_workstations_menu is not None:
        _recent_workstations_menu.setTitle(tr("workstation.recent"))


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


def run_occ_native_task(title, message, fn, on_success, on_error=None):
    """Run an OCC object-producing task on a version-compatible thread."""
    if OCC_BACKGROUND_OBJECTS_SUPPORTED:
        return run_background_task(
            title, message, fn, on_success, on_error)

    # pythonOCC/OCCT 7.4 can complete the calculation in a QThread but crash
    # natively when the returned TopoDS objects are rendered on the GUI thread.
    # Keep object construction and consumption on the main thread for this
    # legacy runtime. Pure-Python solvers continue to use TaskRunner.
    print("pythonOCC {} compatibility: running '{}' on the main thread".format(
        PYTHONOCC_VERSION, title))
    parent = get_main_window()
    progress = QtWidgets.QProgressDialog(message, None, 0, 0, parent)
    progress.setWindowTitle(title)
    progress.setWindowModality(QtCore.Qt.WindowModal)
    progress.setMinimumDuration(0)
    progress.setAutoClose(False)
    progress.setAutoReset(False)
    progress.show()
    QtWidgets.QApplication.processEvents()
    try:
        result = fn()
    except Exception as exc:
        error_text = "{}\n{}".format(str(exc), traceback.format_exc())
        if on_error:
            on_error(error_text)
        else:
            show_topmost_message(tr("common.error"), error_text, type="error")
        return None
    finally:
        progress.close()
    on_success(result)
    return None


# ---------------------------------------------------------------------------
# 6. Path helpers
# ---------------------------------------------------------------------------

def _delete_existing_path():
    state.optimal_path.clear()
    state.last_path_length = 0.0
    state.last_path_algorithm = ""
    state.path_algorithm_diagnostics.clear()
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    state.last_reachability_report.clear()
    for obj in state.optimal_path_objects:
        try:
            display.Context.Erase(obj, True)
        except Exception:
            pass
    state.optimal_path_objects.clear()


def _resolve_optimal_viewpoint_records():
    if len(state.optimal_viewpoint_records) == len(state.optimal_viewpoints_with_pose):
        return list(state.optimal_viewpoint_records)
    records = []
    for index, (point, pose) in enumerate(state.optimal_viewpoints_with_pose):
        if index < len(state.optimal_viewpoint_records):
            existing = state.optimal_viewpoint_records[index]
            face_index = getattr(existing, "face_index", index)
            kind = getattr(existing, "kind", "optimal")
            global_index = getattr(existing, "global_index", index)
            candidate_index = getattr(existing, "candidate_index", 0)
        else:
            face_index = index
            kind = "optimal"
            global_index = index
            candidate_index = 0
        records.append(ViewpointRecord(
            point=point, pose=pose, face_index=face_index, kind=kind,
            global_index=global_index, candidate_index=candidate_index))
    return records


def _current_path_signature(records=None, metadata=None):
    if records is None or metadata is None:
        records, metadata = _ordered_pose_records()
    return compute_path_signature(
        metadata.get("source_pose_records", records),
        state.optimal_path,
        metadata)


def _invalidate_after_path_mutation(keep_path=True):
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    state.last_reachability_report.clear()
    state.sensor_volumes_list.clear()
    state.sensor_volume_objects.clear()
    state.collision_detection_executed = False
    state.sensor_volumes_created = False
    if not keep_path:
        state.optimal_path.clear()
        state.last_path_length = 0.0
        state.last_path_algorithm = ""
        state.path_algorithm_diagnostics.clear()
        for obj in state.optimal_path_objects:
            try:
                display.Context.Erase(obj, True)
            except Exception:
                pass
        state.optimal_path_objects.clear()
    else:
        for obj in state.optimal_path_objects:
            try:
                display.Context.Erase(obj, True)
            except Exception:
                pass
        state.optimal_path_objects.clear()
        if state.optimal_path and state.optimal_viewpoints:
            dist_matrix = build_distance_matrix(state.optimal_viewpoints)
            state.last_path_length = calculate_path_length(state.optimal_path, dist_matrix)
    _do_render(fit_all=True)


def _repair_settings_from_metadata(pose_metadata):
    return {
        "robot_name": pose_metadata.get("expected_robodk_robot_name") or "UR10",
        "frame_name": pose_metadata.get("expected_robodk_frame_name") or "Frame 2",
        "tool_name": pose_metadata.get("expected_robodk_tool_name") or
                     "Creaform MetraSCAN",
    }


def _run_path_algorithm_after_repair(algorithm):
    selected = algorithm or state.last_path_algorithm or "greedy"
    if selected == "sequential":
        connect_sequentially()
    elif selected == "abc":
        solve_with_abc_algorithm()
    elif selected == "mscga":
        solve_with_mscga_algorithm()
    else:
        solve_with_greedy_algorithm()


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

def _workstation_busy():
    return bool(_workstation_io_active or task_runner.active_workers)


def _remember_recent_workstation(path):
    if _workspace_ui is None:
        return
    settings = _workspace_ui.settings
    path = os.path.abspath(path)
    previous = settings.value("recentWorkstations", [])
    if isinstance(previous, str):
        previous = [previous]
    recent = [path] + [item for item in previous if item != path and os.path.isfile(item)]
    settings.setValue("recentWorkstations", recent[:10])


def _populate_recent_workstations(menu):
    menu.clear()
    previous = _workspace_ui.settings.value("recentWorkstations", [])
    if isinstance(previous, str):
        previous = [previous]
    for path in previous:
        if os.path.isfile(path):
            action = menu.addAction(os.path.basename(path))
            action.setToolTip(path)
            action.triggered.connect(lambda checked=False, selected=path:
                                     open_workstation_from_path(selected))
    menu.setEnabled(bool(menu.actions()))


def _confirm_discard_or_save():
    """Return True when a destructive lifecycle action may continue."""
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return False
    if not state.workstation_dirty:
        return True
    box = QtWidgets.QMessageBox(get_main_window())
    box.setWindowTitle(tr("workstation.unsaved.title"))
    box.setText(tr("workstation.unsaved.body"))
    box.setIcon(QtWidgets.QMessageBox.Warning)
    box.setStandardButtons(
        QtWidgets.QMessageBox.Save | QtWidgets.QMessageBox.Discard |
        QtWidgets.QMessageBox.Cancel)
    box.setDefaultButton(QtWidgets.QMessageBox.Save)
    result = (getattr(box, "exec", None) or getattr(box, "exec_"))()
    if result == QtWidgets.QMessageBox.Cancel:
        return False
    if result == QtWidgets.QMessageBox.Discard:
        return True
    return save_workstation_ui()


def _capture_view_state():
    result = {}
    try:
        camera = display.View.Camera()
        for name, getter in (("eye", camera.Eye), ("center", camera.Center),
                             ("up", camera.Up)):
            value = getter()
            result[name] = [value.X(), value.Y(), value.Z()]
        result["scale"] = float(camera.Scale())
    except Exception:
        pass
    return result


def _restore_view_state(view_state):
    try:
        camera = display.View.Camera()
        if "eye" in view_state:
            camera.SetEye(gp_Pnt(*view_state["eye"]))
        if "center" in view_state:
            camera.SetCenter(gp_Pnt(*view_state["center"]))
        if "up" in view_state:
            from OCC.Core.gp import gp_Dir
            camera.SetUp(gp_Dir(*view_state["up"]))
        if "scale" in view_state:
            camera.SetScale(float(view_state["scale"]))
        display.Repaint()
    except Exception as exc:
        print("View restoration warning: {}".format(exc))


def _clear_runtime_scene():
    global _awaiting_face_selection
    _awaiting_face_selection = False
    _clear_selected_face_highlight(update=False)
    erase_all_safely(display)
    center_points_objects.clear()
    normal_line_objects.clear()


def new_workstation(event=None):
    global path_planning_prompt_confirmed
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return False
    if not _confirm_discard_or_save():
        return False
    _clear_runtime_scene()
    state.reset_all()
    state.begin_new_workstation()
    state.record_operation("new_workstation", mark_dirty=False)
    path_planning_prompt_confirmed = False
    _sync_layer()
    _update_workflow(tr("workstation.new.complete"))
    _update_window_title()
    return True


def _save_workstation_to(path):
    global _workstation_io_active
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return False
    _workstation_io_active = True
    try:
        state.record_operation(
            "save_workstation", {"filename": os.path.basename(path)},
            summary="manual save")
        snapshot = capture_workstation(
            state,
            settings={"collision_detection_enabled": collision_detection_enabled},
            view_state=_capture_view_state())
        saved_path, manifest = save_workstation(path, snapshot)
        state.workstation_path = saved_path
        state.workstation_saved_at = manifest["saved_at"]
        state.workstation_dirty = False
        _remember_recent_workstation(saved_path)
        _update_window_title()
        _update_workflow(tr("workstation.save.complete", filename=os.path.basename(saved_path)))
        return True
    except Exception as exc:
        state.workstation_dirty = True
        show_topmost_message(
            tr("common.error"), tr("workstation.save.failed", error=exc), type="error")
        return False
    finally:
        _workstation_io_active = False


def save_workstation_ui(event=None):
    if state.workstation_path:
        return _save_workstation_to(state.workstation_path)
    return save_workstation_as_ui()


def save_workstation_as_ui(event=None):
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return False
    path, _ = QtWidgets.QFileDialog.getSaveFileName(
        get_main_window(), tr("workstation.save_as.title"), "",
        tr("workstation.file_filter"))
    if not path:
        return False
    return _save_workstation_to(path)


def open_workstation_from_path(path, confirm_changes=True):
    global _workstation_io_active, collision_detection_enabled
    path = os.path.abspath(os.fspath(path))
    if os.path.splitext(path)[1].lower() != ".swstation":
        show_topmost_message(
            tr("common.error"),
            tr("message.import.unsupported", extension=os.path.splitext(path)[1]),
            type="error")
        return False
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return False
    if confirm_changes and not _confirm_discard_or_save():
        return False
    _workstation_io_active = True
    try:
        candidate = load_workstation(path)
        old_values = dict(state.__dict__)
        try:
            _clear_runtime_scene()
            state.__dict__.clear()
            state.__dict__.update(candidate.state.__dict__)
            collision_detection_enabled = bool(
                candidate.settings.get("collision_detection_enabled", True))
            _do_render(fit_all=not bool(candidate.view_state))
            _restore_view_state(candidate.view_state)
            _sync_layer()
            _update_workflow(tr(
                "workstation.open.complete", filename=os.path.basename(path)))
            _update_window_title()
        except Exception:
            state.__dict__.clear()
            state.__dict__.update(old_values)
            _do_render()
            raise
        if candidate.warnings:
            show_topmost_message(
                tr("common.prompt"), "\n".join(
                    tr("workstation.warning." + warning)
                    for warning in candidate.warnings), type="warning")
        _remember_recent_workstation(path)
        return True
    except Exception as exc:
        show_topmost_message(
            tr("common.error"), tr("workstation.open.failed", error=exc), type="error")
        return False
    finally:
        _workstation_io_active = False


def open_workstation(event=None):
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return False
    if not _confirm_discard_or_save():
        return False
    path, _ = QtWidgets.QFileDialog.getOpenFileName(
        get_main_window(), tr("workstation.open.title"), "",
        tr("workstation.file_filter"))
    if not path:
        return False
    return open_workstation_from_path(path, confirm_changes=False)


def open_dropped_file(path):
    """Dispatch a validated drop to the workstation or CAD loader."""
    if os.path.splitext(os.fspath(path))[1].lower() == ".swstation":
        return open_workstation_from_path(path)
    return import_model_from_path(path)

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
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return
    if state.workstation_dirty and not _confirm_discard_or_save():
        return
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
        global path_planning_prompt_confirmed, _awaiting_face_selection
        new_shape, axis_size = import_result
        # Commit atomically only after parsing succeeds. Cancelling or a read
        # error leaves the previously loaded model and downstream state intact.
        _clear_selected_face_highlight(update=False)
        erase_all_safely(display)
        state.reset_all()
        _awaiting_face_selection = False
        state.current_shape = new_shape
        state.workpiece_coordinate_system_size = axis_size
        state.model_source_path = file_path
        state.model_source_name = os.path.basename(file_path)
        state.model_source_format = ext.lstrip(".")
        _record_operation("import_model", {"source": file_path},
                          summary=os.path.basename(file_path))
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
    run_occ_native_task(tr("file.import_model.title"), tr("message.import.reading"),
                        load_shape, on_success, on_error)


def clear_model(event=None):
    global path_planning_prompt_confirmed, _awaiting_face_selection
    if _workstation_busy():
        show_topmost_message(tr("common.prompt"), tr("workstation.busy"), type="warning")
        return
    if state.workstation_dirty and not _confirm_discard_or_save():
        return
    result = show_topmost_message(
        tr("message.confirm_clear.title"),
        tr("message.confirm_clear.body"), type="question")
    if result == 'no':
        return

    try:
        _clear_selected_face_highlight(update=False)
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

        erase_all_safely(display)
        display.Context.UpdateCurrentViewer()
        display.Repaint()

        state.reset_all()
        _awaiting_face_selection = False
        _record_operation("clear_model", summary="model and derived results cleared")
        center_points_objects.clear()
        normal_line_objects.clear()
        path_planning_prompt_confirmed = False

        _update_workflow(tr("message.clear.success"))
        show_topmost_message(tr("common.success"), tr("message.clear.success"))
    except Exception as e:
        print("Error clearing model: {}".format(str(e)))


def exit_program(event=None):
    try:
        if _confirm_discard_or_save():
            print("Program exited")
            sys.exit(0)
    except Exception as e:
        print("Error exiting program: {}".format(str(e)))
        sys.exit(1)


# ---------------------------------------------------------------------------
# 8. Model processing handlers
# ---------------------------------------------------------------------------

def select_single_face(event=None):
    global _face_callback_registered, _awaiting_face_selection
    if not state.current_shape:
        show_topmost_message(tr("common.prompt"), tr("message.require.model"))
        return
    try:
        print("Please click a face in the 3D view to select...")
        _awaiting_face_selection = True
        display.SetSelectionModeFace()
        if not _face_callback_registered:
            display.register_select_callback(select_face_clicked)
            _face_callback_registered = True
    except Exception as e:
        print("Error selecting face: {}".format(str(e)))
        traceback.print_exc()


def select_face_clicked(shapes, x, y):
    global _awaiting_face_selection
    if not _awaiting_face_selection or not shapes:
        return
    shape = shapes[0]
    if shape.ShapeType() == TopAbs_FACE:
        _awaiting_face_selection = False
        state.selected_face = shape
        state.selected_face_source_index = -1
        try:
            from OCC.Core.TopExp import TopExp_Explorer
            explorer = TopExp_Explorer(state.current_shape, TopAbs_FACE)
            source_index = 0
            while explorer.More():
                if explorer.Current().IsSame(shape):
                    state.selected_face_source_index = source_index
                    break
                source_index += 1
                explorer.Next()
        except Exception:
            pass
        state.original_face_normal = calculate_face_normal(state.selected_face)
        _record_operation("select_face", {
            "source_face_index": state.selected_face_source_index})
        print("Face selected, normal: ({:.4f}, {:.4f}, {:.4f})".format(
            state.original_face_normal.X(),
            state.original_face_normal.Y(),
            state.original_face_normal.Z()))

        # The callback runs while OCCViewer still owns the Select() result.
        # Release that selected/detected AIS owner before any later EraseAll.
        _clear_selected_face_highlight(update=False)
        display.SetSelectionModeNeutral()
        _do_render(fit_all=False)
        _update_workflow(tr("message.face_selected"))


def segment_faces(event=None):
    if not state.current_shape:
        show_topmost_message(tr("common.prompt"), tr("message.require.model"))
        return
    try:
        params = get_user_segment_params(current=state.segmentation_parameters)
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
            state.selected_face = None
            state.selected_face_source_index = -1
            state.original_face_normal = None
            state.last_segmentation_diagnostics = diagnostics
            state.segmentation_parameters = {
                "u": int(u), "v": int(v),
                "strategy": diagnostics.get("strategy", "equal_param")}
            _record_operation(
                "segment_faces", state.segmentation_parameters,
                summary="{} patches".format(len(faces)))
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
        run_occ_native_task(tr("action.segment_faces"), tr("message.segmentation.task"),
                            lambda: segment_model(shape_to_segment, u, v,
                                                  return_diagnostics=True,
                                                  strategy="auto"),
                            on_success, on_error)
    except Exception as e:
        print("Error segmenting faces: {}".format(str(e)))
        traceback.print_exc()


def get_centers(event=None, render=True):
    if not state.current_faces:
        show_topmost_message(tr("common.prompt"), tr("message.require.segment"))
        return
    try:
        state.invalidate_after_centers()
        state.face_centers.clear()
        state.face_normals.clear()

        # Runtime handles are discarded without incremental viewer updates;
        # the single unified render below clears and rebuilds the scene.
        state.coordinate_systems.clear()
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

            except Exception as e:
                print("Error calculating patch center: {}".format(str(e)))

        if render:
            _do_render()
        print("Centers calculated: {}".format(len(state.face_centers)))
        _record_operation("calculate_centers", summary="{} centers".format(
            len(state.face_centers)))
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
        state.invalidate_after_viewpoints()
        state.view_points.clear()
        state.center_view_points.clear()
        state.center_view_points_with_pose.clear()
        state.viewpoint_records.clear()
        state.center_viewpoint_records.clear()
        state.optimal_viewpoint_records.clear()

        normal_line_objects.clear()

        if not state.face_centers:
            get_centers(render=False)

        center_vps, center_vps_with_pose, line_objs = build_center_viewpoints(
            display, state.current_faces, state.face_centers, state.face_normals,
            render=False)

        state.center_view_points = center_vps
        state.view_points = list(center_vps)
        state.center_view_points_with_pose = center_vps_with_pose
        state.center_viewpoint_records = [
            ViewpointRecord(
                point=vp, pose=pose, face_index=index, kind="center",
                global_index=index, candidate_index=0)
            for index, (vp, pose) in enumerate(center_vps_with_pose)
        ]
        normal_line_objects.extend(line_objs)

        _do_render()
        print("Center viewpoints generated: {}".format(len(center_vps)))
        _record_operation("generate_center_viewpoints",
                          summary="{} viewpoints".format(len(center_vps)))
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
        state.invalidate_after_viewpoints()
        state.view_points.clear()
        state.center_view_points.clear()
        state.center_view_points_with_pose.clear()
        state.view_points_with_pose.clear()
        state.viewpoint_records.clear()
        state.center_viewpoint_records.clear()
        state.optimal_viewpoint_records.clear()

        normal_line_objects.clear()

        if not state.face_centers:
            get_centers(render=False)

        all_vps, all_vps_with_pose, center_vps, line_objs, coord_sys = (
            build_candidate_viewpoints(
                display, state.current_faces, state.face_centers, state.face_normals,
                render=False))

        state.view_points = all_vps
        state.center_view_points = center_vps
        state.view_points_with_pose = all_vps_with_pose
        state.center_view_points_with_pose = [
            (vp, pose) for vp, pose in all_vps_with_pose[:len(center_vps)]]
        state.viewpoint_records = build_viewpoint_records(
            all_vps_with_pose, len(state.current_faces), NUM_CANDIDATES_PER_FACE)
        state.center_viewpoint_records = list(
            state.viewpoint_records[:len(center_vps)])
        normal_line_objects.extend(line_objs)
        state.coordinate_systems.extend(coord_sys)

        _do_render()
        print("Candidate viewpoints generated: {} (including {} centres)".format(
            len(all_vps), len(center_vps)))
        _record_operation("generate_candidate_viewpoints", {
            "candidates_per_face": NUM_CANDIDATES_PER_FACE,
            "viewpoint_distance": VIEWPOINT_DISTANCE,
            "zenith_angle_deg": ZENITH_ANGLE_DEG,
        }, summary="{} viewpoints".format(len(all_vps)))
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
        state.invalidate_after_optimal_viewpoints()
        state.optimal_viewpoints.clear()
        state.optimal_viewpoints_with_pose.clear()
        state.optimal_viewpoint_records.clear()

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
                candidate_records = ordered_face_candidates(
                    state.viewpoint_records, face_idx, num_faces, NUM_CANDIDATES_PER_FACE)
            else:
                candidates = []
                candidate_records = []
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
            best_index = candidates.index((best_vp, best_pose)) if candidates else -1
            best_record = None
            if candidate_records and 0 <= best_index < len(candidate_records):
                best_record = candidate_records[best_index]
            elif state.viewpoint_records:
                best_record = state.viewpoint_records[face_idx]
            if best_record is None:
                best_record = ViewpointRecord(
                    point=best_vp, pose=best_pose, face_index=face_idx,
                    kind="optimal", global_index=face_idx, candidate_index=0)

            state.optimal_viewpoints.append(best_vp)
            state.optimal_viewpoints_with_pose.append((best_vp, best_pose))
            state.optimal_viewpoint_records.append(best_record)

        _do_render()
        print("Optimal viewpoints filtered: {}".format(len(state.optimal_viewpoints)))
        _record_operation("filter_optimal_viewpoints",
                          summary="{} selected".format(len(state.optimal_viewpoints)))
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
    _record_operation("set_collision_detection", {
        "enabled": collision_detection_enabled})


def set_sensor_parameters(event=None):
    result = get_sensor_parameters_dialog(current_config=state.sensor_size_config)
    if result is not None:
        state.sensor_size_config.update(result)
        state.sensor_volumes_list.clear()
        state.sensor_volume_objects.clear()
        state.collision_results.clear()
        state.sensor_volumes_created = False
        state.collision_detection_executed = False
        _do_render(fit_all=False)
        _record_operation("set_sensor_parameters", result)
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
        state.collision_results.clear()
        state.collision_detection_executed = False
        _record_operation("create_sensor_volumes", state.sensor_size_config,
                          summary="{} volumes".format(created))
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
        state.collision_results.clear()
        state.collision_detection_executed = False
        _record_operation("generate_obb_boxes",
                          summary="{} boxes".format(len(state.face_obbs)))
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
            state.collision_results = list(results)
            summary = summarize_collision_results(results)
            message = tr(
                "message.collision.complete", collisions=summary.collisions,
                total=summary.total)
            show_topmost_message(tr("menu.collision_detection"), message)
            state.collision_detection_executed = True
            _record_operation("execute_collision_detection", {
                "enabled": collision_detection_enabled},
                summary="{} / {} collisions".format(
                    summary.collisions, summary.total))
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
        display.DisplayShape(test_cube, color=ERROR, update=False)
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
        state.last_path_algorithm = "sequential"
        state.path_algorithm_diagnostics = {"algorithm": "sequential"}
        dist_matrix = build_distance_matrix(state.optimal_viewpoints)
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        state.algorithm_parameters["path"] = {"algorithm": "sequential"}
        _record_operation("plan_path", state.algorithm_parameters["path"],
                          summary="length {:.3f} mm".format(total))
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
        state.last_path_algorithm = "greedy"
        state.path_algorithm_diagnostics = {"algorithm": "greedy"}
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        state.algorithm_parameters["path"] = {"algorithm": "greedy"}
        _record_operation("plan_path", state.algorithm_parameters["path"],
                          summary="length {:.3f} mm".format(total))
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
    state.algorithm_parameters["path"] = dict(
        vars(DEFAULT_ABC_CONFIG), algorithm="abc")
    abc_points = np.array([[p.X(), p.Y(), p.Z()] for p in state.optimal_viewpoints])
    set_status_message(tr("message.path.abc_running"))

    def on_success(result):
        state.optimal_path = list(result["best_tour"])
        state.last_path_algorithm = "abc"
        state.path_algorithm_diagnostics = {
            key: value for key, value in result.items() if key != "best_tour"}
        state.last_path_length = result["best_length"]
        _record_operation("plan_path", state.algorithm_parameters["path"],
                          summary="length {:.3f} mm".format(result["best_length"]))
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
    state.algorithm_parameters["path"] = dict(
        vars(DEFAULT_MSCGA_CONFIG), algorithm="mscga", closed_tour=False)
    set_status_message(tr("message.path.mscga_running"))

    def on_success(result):
        state.optimal_path = list(result["best_tour"])
        state.last_path_algorithm = "mscga"
        state.path_algorithm_diagnostics = {
            key: value for key, value in result.items() if key != "best_tour"}
        state.last_path_length = result["best_length"]
        _record_operation("plan_path", state.algorithm_parameters["path"],
                          summary="length {:.3f} mm".format(result["best_length"]))
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
        state.last_path_algorithm = "greedy"
        state.path_algorithm_diagnostics = {"algorithm": "greedy"}
        draw_path_edges(display, state.optimal_path, state.optimal_viewpoints, state.optimal_path_objects)
        total = calculate_path_length(state.optimal_path, dist_matrix)
        state.last_path_length = total
        state.algorithm_parameters["path"] = {"algorithm": "greedy"}
        _record_operation("plan_path", state.algorithm_parameters["path"],
                          summary="length {:.3f} mm".format(total))
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
        _record_operation("export_path", {"path": file_path},
                          summary="{} rows".format(count))
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

def _activate_extrinsic_config(config, file_path):
    state.extrinsic_config = config
    state.extrinsic_config_path = file_path
    with open(file_path, "rb") as stream:
        state.extrinsic_config_sha256 = hashlib.sha256(stream.read()).hexdigest()
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    state.last_reachability_report.clear()
    _record_operation("load_calibration", {
        "config_id": config.config_id,
        "calibration_status": config.calibration_status,
        "mapping_mode": config.mapping_mode,
    }, summary=os.path.basename(file_path))


def load_scanner_tool_extrinsic(event=None):
    file_path, _ = QtWidgets.QFileDialog.getOpenFileName(
        get_main_window(), tr("file.load_extrinsic.title"), "calibration",
        tr("file.json_filter"))
    if not file_path:
        return
    try:
        config = load_extrinsic_config(file_path)
        _activate_extrinsic_config(config, file_path)
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


def capture_current_robodk_station_mapping(event=None):
    """Read current RoboDK frame/tool relationships and save a JSON mapping."""
    settings = get_robodk_mapping_settings(get_main_window())
    if not settings:
        return
    try:
        config = capture_robodk_station_mapping(**settings)
    except Exception as e:
        show_topmost_message(
            tr("common.error"),
            tr("message.extrinsic.capture_failed", error=e), type="error")
        return
    default_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "calibration",
        "robodk_live_station_mapping.json")
    file_path, _ = QtWidgets.QFileDialog.getSaveFileName(
        get_main_window(), tr("file.save_robodk_mapping.title"),
        default_path, tr("file.json_filter"))
    if not file_path:
        return
    try:
        saved_path = save_extrinsic_config(file_path, config)
        loaded = load_extrinsic_config(saved_path)
        _activate_extrinsic_config(loaded, saved_path)
        message = tr(
            "message.extrinsic.captured",
            station=loaded.robodk_station_name,
            base=loaded.robodk_base_name,
            frame=loaded.robodk_frame_name,
            tool=loaded.robodk_tool_name,
            path=saved_path)
        _update_workflow(message)
        show_topmost_message(
            tr("dialog.extrinsic.capture_title"),
            message + "\n\n" + tr("message.extrinsic.simulation_only"),
            type="warning")
    except Exception as e:
        show_topmost_message(
            tr("common.error"),
            tr("message.extrinsic.save_failed", error=e), type="error")


def clear_scanner_tool_extrinsic(event=None):
    state.extrinsic_config = None
    state.extrinsic_config_path = ""
    state.extrinsic_config_sha256 = ""
    state.speed_plan_result = None
    state.last_speed_csv_path = ""
    state.last_robodk_import.clear()
    state.last_reachability_report.clear()
    _record_operation("clear_calibration")
    _update_workflow(tr("message.extrinsic.cleared_status"))
    show_topmost_message(
        tr("dialog.extrinsic.title"),
        tr("message.extrinsic.cleared_body"),
        type="warning")


def _ordered_pose_records():
    if not state.optimal_viewpoints_with_pose:
        raise ValueError("No path poses are available")
    optimal_records = _resolve_optimal_viewpoint_records()
    records = build_ordered_path_records(
        state.optimal_path, state.optimal_viewpoints_with_pose, optimal_records)
    if state.extrinsic_config is not None:
        command_records, metadata = transform_pose_records(records, state.extrinsic_config)
        metadata["extrinsic_config_path"] = state.extrinsic_config_path
        metadata["extrinsic_config_sha256"] = state.extrinsic_config_sha256
        metadata["path_signature"] = compute_path_signature(
            metadata.get("source_pose_records", records), state.optimal_path, metadata)
        metadata["path_provenance"] = [
            {key: record.get(key) for key in (
                "index", "optimal_index", "face_index", "candidate_index",
                "global_index", "kind")}
            for record in metadata.get("source_pose_records", records)
        ]
        return command_records, metadata
    metadata = {
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
    metadata["path_signature"] = compute_path_signature(records, state.optimal_path, metadata)
    metadata["path_provenance"] = [
        {key: record.get(key) for key in (
            "index", "optimal_index", "face_index", "candidate_index",
            "global_index", "kind")}
        for record in records
    ]
    return records, metadata


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
        _record_operation("import_path_to_robodk", settings,
                          summary=summary.get("program", ""))
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


def analyze_planned_path_reachability(event=None):
    """Check ordered path poses against the live RoboDK UR10 model."""
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
    if (not pose_metadata.get("extrinsic_validated", False) or
            pose_metadata.get("T_base_workpiece") is None):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.reachability_calibration_required"),
            type="error")
        return

    settings = {
        "robot_name": pose_metadata.get("expected_robodk_robot_name") or "UR10",
        "frame_name": pose_metadata.get("expected_robodk_frame_name") or "Frame 2",
        "tool_name": pose_metadata.get("expected_robodk_tool_name") or
                     "Creaform MetraSCAN",
    }
    initial_signature = pose_metadata.get("path_signature") or _current_path_signature(
        records, pose_metadata)

    def on_success(report):
        try:
            current_records, current_metadata = _ordered_pose_records()
            if _current_path_signature(current_records, current_metadata) != initial_signature:
                show_topmost_message(
                    tr("common.warning"),
                    tr("message.robodk.reachability_stale_abort"),
                    type="warning")
                return
        except Exception as exc:
            show_topmost_message(
                tr("common.error"),
                tr("message.path.input_invalid", error=exc),
                type="error")
            return
        report = dict(report)
        report["path_signature"] = pose_metadata.get(
            "path_signature", initial_signature)
        report["path_provenance"] = pose_metadata.get("path_provenance", [])
        report["source_pose_records"] = pose_metadata.get(
            "source_pose_records", records)
        state.last_reachability_report = report
        _record_operation("analyze_robodk_reachability", settings,
                          summary="{} reachable, {} unreachable".format(
                              report["reachable_count"], report["unreachable_count"]))
        available = ", ".join(
            str(index) for index in report["reachable_indices"]
        ) or tr("common.none")
        unavailable = ", ".join(
            "{} ({})".format(
                point["index"],
                tr("message.robodk.reachability_reason.{}".format(
                    point["reason"])))
            for point in report["points"] if not point["reachable"]
        ) or tr("common.none")
        message = tr(
            "message.robodk.reachability_complete",
            total=report["pose_count"],
            reachable=report["reachable_count"],
            unreachable=report["unreachable_count"],
            reachable_indices=available,
            unreachable_details=unavailable)
        message += "\n\n" + tr("message.robodk.reachability_boundary")
        _update_workflow(message)
        show_topmost_message(
            tr("dialog.robodk.reachability_title"), message,
            type="info" if report["all_reachable"] else "warning")

    def on_error(error_text):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.reachability_failed", error=error_text),
            type="error")

    run_background_task(
        tr("dialog.robodk.reachability_title"),
        tr("message.robodk.reachability_running"),
        lambda: analyze_ur10_reachability(
            records, pose_metadata, **settings),
        on_success, on_error)


def plan_path_speeds(event=None):
    if not state.optimal_path or not state.optimal_viewpoints_with_pose:
        show_topmost_message(tr("common.prompt"), tr("message.require.path"))
        return
    settings = get_speed_planning_settings(
        get_main_window(), current=state.algorithm_parameters.get("speed"))
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
        state.algorithm_parameters["speed"] = dict(
            vars(profile), algorithm=algorithm)
        _record_operation("plan_speed", state.algorithm_parameters["speed"],
                          summary="{} points, {:.3f} s".format(
                              len(result.points), result.total_time))
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


def repair_ur10_reachability_failures(event=None):
    """Replace unreachable ordered path points with same-face reachable candidates."""
    if not state.optimal_path or not state.optimal_viewpoints_with_pose:
        show_topmost_message(tr("common.prompt"), tr("message.require.path"))
        return
    if not state.view_points_with_pose:
        show_topmost_message(
            tr("common.prompt"),
            tr("message.robodk.reachability_repair_no_candidates"),
            type="warning")
        return
    try:
        if len(state.viewpoint_records) != len(state.view_points_with_pose):
            state.viewpoint_records = build_viewpoint_records(
                state.view_points_with_pose, len(state.current_faces),
                NUM_CANDIDATES_PER_FACE)
            state.center_viewpoint_records = list(
                state.viewpoint_records[:len(state.current_faces)])
        if len(state.optimal_viewpoint_records) != len(state.optimal_viewpoints_with_pose):
            state.optimal_viewpoint_records = _resolve_optimal_viewpoint_records()
        records, pose_metadata = _ordered_pose_records()
    except Exception as exc:
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.reachability_repair_missing_provenance", error=exc),
            type="error")
        return
    if (not pose_metadata.get("extrinsic_validated", False) or
            pose_metadata.get("T_base_workpiece") is None):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.reachability_calibration_required"),
            type="error")
        return

    settings = _repair_settings_from_metadata(pose_metadata)
    initial_signature = pose_metadata.get("path_signature") or _current_path_signature(
        records, pose_metadata)
    source_records = pose_metadata.get("source_pose_records", records)
    last_report = state.last_reachability_report or {}
    use_cached_report = last_report.get("path_signature") == initial_signature

    def evaluate_source_records(source):
        command_records, metadata = transform_pose_records(source, state.extrinsic_config)
        metadata["extrinsic_config_path"] = state.extrinsic_config_path
        metadata["extrinsic_config_sha256"] = state.extrinsic_config_sha256
        report = dict(analyze_ur10_reachability(
            command_records, metadata, **settings))
        report["source_pose_records"] = [dict(record) for record in source]
        report["path_signature"] = compute_path_signature(
            source, state.optimal_path, metadata)
        report["path_provenance"] = [
            {key: record.get(key) for key in (
                "index", "optimal_index", "face_index", "candidate_index",
                "global_index", "kind")}
            for record in source
        ]
        return report

    def work():
        initial_report = last_report if use_cached_report else evaluate_source_records(source_records)
        if initial_report.get("all_reachable", False):
            return {"status": "all_reachable", "report": initial_report}
        repair_result = repair_unreachable_points(
            source_records=source_records,
            pose_metadata=pose_metadata,
            optimal_path=state.optimal_path,
            optimal_viewpoints=state.optimal_viewpoints,
            optimal_viewpoints_with_pose=state.optimal_viewpoints_with_pose,
            optimal_viewpoint_records=state.optimal_viewpoint_records,
            candidate_records=state.viewpoint_records,
            evaluate_fn=evaluate_source_records,
            face_count=len(state.current_faces),
            candidates_per_face=NUM_CANDIDATES_PER_FACE)
        return {"status": "repaired", "result": repair_result}

    def on_success(payload):
        try:
            current_records, current_metadata = _ordered_pose_records()
            if _current_path_signature(current_records, current_metadata) != initial_signature:
                show_topmost_message(
                    tr("common.warning"),
                    tr("message.robodk.reachability_repair_stale_abort"),
                    type="warning")
                return
        except Exception as exc:
            show_topmost_message(
                tr("common.error"),
                tr("message.path.input_invalid", error=exc),
                type="error")
            return

        if payload["status"] == "all_reachable":
            report = dict(payload["report"])
            report["path_signature"] = initial_signature
            state.last_reachability_report = report
            _record_operation("repair_robodk_reachability", settings,
                              summary="no failures")
            show_topmost_message(
                tr("dialog.robodk.reachability_repair_title"),
                tr("message.robodk.reachability_repair_no_failures"),
                type="info")
            return

        repair_result = payload["result"]
        if not repair_result.replacements:
            state.last_reachability_report = dict(repair_result.final_report)
            _record_operation("repair_robodk_reachability", settings,
                              summary="no replacement found")
            show_topmost_message(
                tr("dialog.robodk.reachability_repair_title"),
                tr("message.robodk.reachability_repair_none",
                   unrepaired=", ".join(str(i) for i in repair_result.unrepaired_indices) or tr("common.none")),
                type="warning")
            return

        previous_algorithm = state.last_path_algorithm or "greedy"
        state.optimal_viewpoints = list(repair_result.updated_optimal_viewpoints)
        state.optimal_viewpoints_with_pose = list(
            repair_result.updated_optimal_viewpoints_with_pose)
        state.optimal_viewpoint_records = list(
            repair_result.updated_optimal_viewpoint_records)
        _invalidate_after_path_mutation(keep_path=True)
        final_report = dict(repair_result.final_report)
        final_report["source_pose_records"] = repair_result.final_source_records
        final_report["path_signature"] = compute_path_signature(
            repair_result.final_source_records, state.optimal_path, pose_metadata)
        state.last_reachability_report = final_report
        _record_operation("repair_robodk_reachability", settings,
                          summary="{} replacements".format(
                              len(repair_result.replacements)))

        replaced = len(repair_result.replacements)
        unrepaired = ", ".join(str(i) for i in repair_result.unrepaired_indices) or tr("common.none")
        message = tr(
            "message.robodk.reachability_repair_complete",
            replaced=replaced,
            unrepaired=unrepaired,
            remaining=final_report.get("unreachable_count", 0))
        _update_workflow(message)
        answer = show_topmost_message(
            tr("dialog.robodk.reachability_repair_title"),
            message + "\n\n" + tr(
                "message.robodk.reachability_repair_replan_prompt",
                algorithm=previous_algorithm),
            type="question")
        if answer == "yes":
            _run_path_algorithm_after_repair(previous_algorithm)
        else:
            _update_workflow(tr(
                "message.robodk.reachability_repair_keep_order_complete",
                length=state.last_path_length))

    def on_error(error_text):
        show_topmost_message(
            tr("common.error"),
            tr("message.robodk.reachability_repair_failed", error=error_text),
            type="error")

    run_background_task(
        tr("dialog.robodk.reachability_repair_title"),
        tr("message.robodk.reachability_repair_running"),
        work, on_success, on_error)


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
        _record_operation("export_speed_plan", {"path": file_path},
                          summary="{} rows".format(count))
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
        _record_operation("import_speed_plan_to_robodk", settings,
                          summary=summary.get("program", ""))
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
    _record_operation("set_layer_visibility", {
        "layer": layer_key, "visible": not current})
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

def create_operation_panel_ui(event=None):
    create_operation_panel(state)


def _toggle_panel(name, show_panel):
    dock = get_main_window().findChild(QtWidgets.QDockWidget, name)
    if dock is None or not dock.isVisible():
        show_panel()
    else:
        dock.hide()


# ---------------------------------------------------------------------------
# 14. Entry point
# ---------------------------------------------------------------------------

def _run_ui_command(callback):
    try:
        callback()
    except Exception as exc:
        show_topmost_message(tr("common.error"), str(exc), type="error")


def _show_ui_text(title_key, body):
    dialog = QtWidgets.QDialog(get_main_window())
    dialog.setWindowTitle(tr(title_key))
    dialog.resize(640, 440)
    layout = QtWidgets.QVBoxLayout(dialog)
    editor = QtWidgets.QPlainTextEdit(dialog)
    editor.setReadOnly(True)
    editor.setPlainText(body)
    layout.addWidget(editor)
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Close)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)
    (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))()


def _show_shortcuts():
    lines = ["Ctrl+Shift+0: " + tr("action.default_layout")]
    lines += ["{}: {}".format(action.shortcut().toString(), action.text())
             for action in _workspace_ui.actions.values() if not action.shortcut().isEmpty()]
    _show_ui_text("action.show_shortcuts", "\n".join(sorted(lines)))


def _show_diagnostics():
    import platform
    lines = ["Python: " + sys.version.split()[0],
             "pythonOCC: " + str(PYTHONOCC_VERSION),
             "Qt: " + QtCore.QT_VERSION_STR,
             "OS: " + platform.platform(),
             "Model: " + (state.model_source_path or "-")]
    _show_ui_text("action.show_diagnostics", "\n".join(lines))


def _configure_shortcuts():
    dialog = QtWidgets.QDialog(get_main_window())
    dialog.setWindowTitle(tr("action.configure_shortcuts"))
    dialog.resize(600, 500)
    layout = QtWidgets.QVBoxLayout(dialog)
    table = QtWidgets.QTableWidget(dialog)
    actions = list(_workspace_ui.actions.values())
    table.setColumnCount(2)
    table.setHorizontalHeaderLabels([tr("ui.shortcut.command"), tr("ui.shortcut.keys")])
    table.setRowCount(len(actions))
    for row, action in enumerate(actions):
        name = QtWidgets.QTableWidgetItem(action.text())
        name.setFlags(name.flags() & ~QtCore.Qt.ItemIsEditable)
        table.setItem(row, 0, name)
        table.setItem(row, 1, QtWidgets.QTableWidgetItem(action.shortcut().toString()))
    table.horizontalHeader().setStretchLastSection(True)
    layout.addWidget(table)
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok |
                                          QtWidgets.QDialogButtonBox.Cancel)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)
    if (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))() != QtWidgets.QDialog.Accepted:
        return
    values = [table.item(row, 1).text().strip() for row in range(len(actions))]
    sequences = [QtGui.QKeySequence(value) for value in values]
    if any(value and sequence.isEmpty()
           for value, sequence in zip(values, sequences)):
        raise ValueError(tr("ui.shortcut.invalid"))
    normalized = [sequence.toString() for sequence in sequences if not sequence.isEmpty()]
    if len(normalized) != len(set(normalized)) or "Ctrl+Shift+0" in normalized:
        raise ValueError(tr("ui.shortcut.conflict"))
    for action, sequence in zip(actions, sequences):
        action.setShortcut(sequence)
        key = action.property("i18n_key")
        if key:
            _workspace_ui.settings.setValue("shortcuts/" + str(key), sequence.toString())


USAGE_TEXT_INLINE = """\
Usage Instructions:
  1. Use the mouse in the 3D viewer to rotate and zoom the model
  2. Please use functions in order:
     Import Model -> Segment Faces -> Get Centers -> Generate Viewpoints
     -> Filter Optimal Viewpoints -> Path Planning
  3. Use File for workstation and CAD model operations
  4. Use Path planning for segmentation, viewpoints, path and speed planning
  5. Use Collision detection for sensor and bounding-box checks
  6. Use Calibration and export for RoboDK, reachability and CSV
  7. Use View, Settings and Help for the camera, panels and preferences
  8. Use File -> Archives -> Save Workstation to preserve the session
  9. Load a validated T_tool_scanner from 'Calibration' before production import
  10. Check UR10 reachability using the current RoboDK robot model
  11. After path ordering, use 'Speed Planning' to plan constrained per-pose speeds
  12. Export Pose+Speed CSV or import Set Speed -> Move pairs to RoboDK
"""


def run():
    global _language_subscription, _workspace_ui, _recent_workstations_menu
    print("=== 3D Model Processing and Path Planning System ===")
    # Print usage to console instead of showing a blocking dialog at startup.
    # Users can access it later via Help -> show_usage_instructions.
    print(USAGE_TEXT_INLINE)

    apply_application_theme(QtCore, QtGui, QtWidgets, get_main_window())
    if get_main_window() is not None:
        get_main_window().setDockNestingEnabled(True)
    configure_viewer(display)

    # Menu IDs are stable; visible labels are read from the active JSON catalog.
    _add_translated_menu("File", "menu.file")
    action = _add_translated_action("File", new_workstation,
                                    "action.new_workstation")
    if action is not None:
        action.setShortcut("Ctrl+N")
    action = _add_translated_action("File", open_workstation,
                                    "action.open_workstation")
    if action is not None:
        action.setShortcut("Ctrl+O")
    action = _add_translated_action("File", save_workstation_ui,
                                    "action.save_workstation")
    if action is not None:
        action.setShortcut("Ctrl+S")
    action = _add_translated_action("File", save_workstation_as_ui,
                                    "action.save_workstation_as")
    if action is not None:
        action.setShortcut("Ctrl+Shift+S")
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
    _add_translated_action("View", lambda: _toggle_panel(
        "WorkflowStatusDock", create_workflow_panel_ui),
                           "action.create_workflow_panel")
    _add_translated_action("View", lambda: _toggle_panel(
        "LayerControlDock", create_layer_panel_ui),
                           "action.create_layer_panel")
    _add_translated_action("View", lambda: _toggle_panel(
        "OperationHistoryDock", create_operation_panel_ui),
                           "action.create_operation_panel")

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
    _add_translated_action("Path Planning", analyze_planned_path_reachability,
                           "action.analyze_ur10_reachability")
    _add_translated_action("Path Planning", repair_ur10_reachability_failures,
                           "action.repair_ur10_reachability")
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

    # Calibration menu. Capture is read-only; saved station mappings still
    # require independent physical calibration before production execution.
    _add_translated_menu("Calibration", "menu.calibration")
    _add_translated_action("Calibration", capture_current_robodk_station_mapping,
                           "action.capture_robodk_mapping")
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
    _add_translated_action("Help", lambda: _toggle_panel(
        "WorkflowStatusDock", create_workflow_panel_ui),
                           "action.create_workflow_panel")
    _add_translated_action("Help", lambda: _toggle_panel(
        "LayerControlDock", create_layer_panel_ui),
                           "action.create_layer_panel")
    _add_translated_action("Help", lambda: _toggle_panel(
        "OperationHistoryDock", create_operation_panel_ui),
                           "action.create_operation_panel")
    _add_translated_action("Help", show_usage_instructions, "action.show_usage")

    # Keep a single QAction for each command; the menu is hidden and the
    # ribbon, search and favorites all trigger these same actions.
    for command in ("fit", "iso", "front", "back", "left", "right",
                    "top", "bottom", "orthographic", "perspective",
                    "shaded", "wireframe", "edges", "screenshot"):
        _add_translated_action(
            "View", lambda checked=False, name=command:
            _run_ui_command(lambda: _workspace_ui.view(name)),
            "action.view_" + command)
    for key, method in (("action.default_layout", "default_layout"),
                        ("action.focus_layout", "focus_layout"),
                        ("action.toggle_fullscreen", "toggle_fullscreen"),
                        ("action.save_named_layout", "save_named_layout"),
                        ("action.restore_named_layout", "restore_named_layout"),
                        ("action.delete_named_layout", "delete_named_layout")):
        _add_translated_action("View", lambda checked=False, name=method:
                               _run_ui_command(getattr(_workspace_ui, name)), key)
    _add_translated_action(
        "View", lambda: _run_ui_command(lambda:
            _workspace_ui.configure_local_frames(lambda: _do_render(fit_all=False))),
        "action.configure_local_frames")
    _add_translated_menu("Settings", "ribbon.settings")
    _add_translated_action("Settings", lambda: _workspace_ui.configure_favorites(),
                           "action.configure_favorites")
    _add_translated_action("Settings", lambda: _run_ui_command(_configure_shortcuts),
                           "action.configure_shortcuts")
    _add_translated_action("Settings", lambda: _run_ui_command(
        _workspace_ui.configure_robodk),
        "action.configure_robodk")
    _add_translated_action("Settings", lambda: _run_ui_command(
        _workspace_ui.configure_colors), "action.configure_colors")
    for key, method in (("action.export_ui_config", "export_config"),
                        ("action.import_ui_config", "import_config"),
                        ("action.reset_ui_config", "reset_config")):
        _add_translated_action("Settings", lambda checked=False, name=method:
                               _run_ui_command(getattr(_workspace_ui, name)), key)
    _add_translated_action("Help", lambda: _show_ui_text(
        "action.show_mouse_guide", tr("help.mouse_guide")),
        "action.show_mouse_guide")
    _add_translated_action("Help", _show_shortcuts, "action.show_shortcuts")
    _add_translated_action("Help", _show_diagnostics, "action.show_diagnostics")
    _add_translated_action("Help", lambda: _workspace_ui.show_help(),
                           "action.search_help")
    _add_translated_action("Help", lambda: _show_ui_text(
        "action.show_about", tr("help.about")), "action.show_about")

    actions_by_key = {
        str(action.property("i18n_key")): action
        for action in _translated_actions
        if action.property("i18n_key")
    }
    create_primary_toolbar(QtCore, QtWidgets, get_main_window(), actions_by_key)
    _workspace_ui = WorkspaceUI(QtCore, QtGui, QtWidgets, get_main_window(),
                                display, actions_by_key)
    _workspace_ui.redraw_scene = lambda: _do_render(fit_all=False)
    for key, action in actions_by_key.items():
        saved_shortcut = _workspace_ui.settings.value("shortcuts/" + key)
        if saved_shortcut is not None:
            action.setShortcut(str(saved_shortcut))
    ribbon = get_main_window().findChild(QtWidgets.QTabWidget, "CommandRibbon")
    _workspace_ui.attach_header(ribbon)
    archive_button = ribbon.findChild(QtWidgets.QToolButton,
                                     "RibbonDropdown_ribbon.group.workstation")
    if archive_button is not None:
        recent_menu = archive_button.menu().addMenu(tr("workstation.recent"))
        recent_menu.aboutToShow.connect(lambda: _populate_recent_workstations(recent_menu))
        _recent_workstations_menu = recent_menu

    if _language_subscription is None:
        _language_subscription = subscribe_language_changed(_retranslate_main_ui)

    # Install close-event interceptor
    close_filter = MainWindowCloseEvent(QtWidgets.QApplication.instance())
    QtWidgets.QApplication.instance().installEventFilter(close_filter)

    # Create panels by default
    create_workflow_panel_ui()
    create_layer_panel_ui()
    create_operation_panel_ui()
    if not _workspace_ui.settings.contains("dockState"):
        get_main_window().findChild(QtWidgets.QDockWidget,
                                    "OperationHistoryDock").hide()
    _workspace_ui.remember_default_layout()
    for key, dock_name in (("action.create_workflow_panel", "WorkflowStatusDock"),
                           ("action.create_layer_panel", "LayerControlDock"),
                           ("action.create_operation_panel", "OperationHistoryDock")):
        dock = get_main_window().findChild(QtWidgets.QDockWidget, dock_name)
        related = [item for item in _translated_actions
                   if item.property("i18n_key") == key]
        for item in related:
            item.setCheckable(True)
            item.setChecked(dock.isVisible())
        dock.visibilityChanged.connect(
            lambda visible, group=related: [item.setChecked(visible) for item in group])
    _retranslate_main_ui()
    _update_window_title()

    # Accept one local STEP/IGES model or .swstation on the 3D canvas.
    window = get_main_window()
    viewer = getattr(window, "canva", None) if window is not None else None
    if viewer is not None:
        install_model_drop_support(
            QtCore, viewer, open_dropped_file,
            show_error=lambda message: show_topmost_message(
                tr("file.import_model.title"), message, type="error"),
            set_status=set_status_message,
            translate=tr)

    global _app_ready
    _app_ready = True

    start_display()


if __name__ == "__main__":
    run()
