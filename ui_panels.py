"""UI panels, dialogs, and message helpers.

All Qt widget creation is done *lazily* inside each function so the module
can be imported before ``init_display()`` is called.
"""

from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Workflow step status constants
# ---------------------------------------------------------------------------

STATUS_NOT_READY = "Not ready"
STATUS_READY = "Ready"
STATUS_RUNNING = "Running"
STATUS_DONE = "Done"
STATUS_ERROR = "Error"


@dataclass
class WorkflowStep:
    key: str
    label: str
    status: str = STATUS_NOT_READY


DEFAULT_WORKFLOW_STEPS = [
    WorkflowStep("import", "Import Model"),
    WorkflowStep("select_face", "Select Face"),
    WorkflowStep("segment", "Segment"),
    WorkflowStep("centers", "Compute Centers"),
    WorkflowStep("viewpoints", "Generate Viewpoints"),
    WorkflowStep("filter", "Filter Viewpoints"),
    WorkflowStep("sensor", "Create Sensor Volumes"),
    WorkflowStep("obb", "Generate OBB"),
    WorkflowStep("collision", "Collision Check"),
    WorkflowStep("path", "Path Planning"),
    WorkflowStep("speed", "Speed Planning"),
    WorkflowStep("robodk", "RoboDK Import"),
    WorkflowStep("export", "Export CSV"),
]


# ---------------------------------------------------------------------------
# Workflow snapshot
# ---------------------------------------------------------------------------

@dataclass
class WorkflowSnapshot:
    model_loaded: bool = False
    selected_face: bool = False
    patches: int = 0
    centers: int = 0
    center_viewpoints: int = 0
    all_viewpoints: int = 0
    optimal_viewpoints: int = 0
    sensor_volumes: int = 0
    obb_boxes: int = 0
    collisions_executed: bool = False
    path_points: int = 0
    path_length: float = 0.0
    speed_points: int = 0
    speed_total_time: float = 0.0
    speed_feasible: bool = False
    robodk_program: str = ""


def build_workflow_snapshot(state):
    """Collect a compact status snapshot from *state* (AppState or equivalent)."""
    speed_result = getattr(state, 'speed_plan_result', None)
    robodk_import = getattr(state, 'last_robodk_import', {}) or {}
    return WorkflowSnapshot(
        model_loaded=getattr(state, 'current_shape', None) is not None,
        selected_face=getattr(state, 'selected_face', None) is not None,
        patches=len(getattr(state, 'current_faces', [])),
        centers=len(getattr(state, 'face_centers', [])),
        center_viewpoints=len(getattr(state, 'center_view_points', [])),
        all_viewpoints=len(getattr(state, 'view_points', [])),
        optimal_viewpoints=len(getattr(state, 'optimal_viewpoints', [])),
        sensor_volumes=len(getattr(state, 'sensor_volumes_list', [])),
        obb_boxes=len(getattr(state, 'face_obbs', [])),
        collisions_executed=getattr(state, 'collision_detection_executed', False),
        path_points=len(getattr(state, 'optimal_path', [])),
        path_length=getattr(state, 'last_path_length', 0.0),
        speed_points=len(speed_result.points) if speed_result is not None else 0,
        speed_total_time=speed_result.total_time if speed_result is not None else 0.0,
        speed_feasible=bool(speed_result.feasible) if speed_result is not None else False,
        robodk_program=str(robodk_import.get('program', '')),
    )


def format_workflow_snapshot(snap):
    """Format a WorkflowSnapshot for display."""
    return (
        "Workflow Status\n"
        "Model loaded: {}\n"
        "Selected face: {}\n"
        "Segmented patches: {}\n"
        "Patch centers: {}\n"
        "Center viewpoints: {}\n"
        "All viewpoints: {}\n"
        "Optimal viewpoints: {}\n"
        "Sensor volumes: {}\n"
        "OBB boxes: {}\n"
        "Collision checked: {}\n"
        "Path points: {}\n"
        "Path length: {:.4f}\n"
        "Speed plan points: {}\n"
        "Planned scan time: {:.4f} s\n"
        "Speed constraints feasible: {}\n"
        "RoboDK program: {}"
    ).format(
        'Yes' if snap.model_loaded else 'No',
        'Yes' if snap.selected_face else 'No',
        snap.patches, snap.centers, snap.center_viewpoints,
        snap.all_viewpoints, snap.optimal_viewpoints,
        snap.sensor_volumes, snap.obb_boxes,
        'Yes' if snap.collisions_executed else 'No',
        snap.path_points, snap.path_length,
        snap.speed_points, snap.speed_total_time,
        'Yes' if snap.speed_feasible else 'No',
        snap.robodk_program or '-',
    )


# ---------------------------------------------------------------------------
# Qt helpers
# ---------------------------------------------------------------------------

def _get_qt():
    """Return (QtCore, QtWidgets) after OCC backend init."""
    from OCC.Display.backend import get_qt_modules
    QtCore, QtGui, QtWidgets, QtOpenGL = get_qt_modules()
    return QtCore, QtWidgets


def get_main_window():
    """Find the pythonOCC main window."""
    try:
        QtCore, QtWidgets = _get_qt()
        app = QtWidgets.QApplication.instance()
        if not app:
            return None
        titles = []
        for widget in app.topLevelWidgets():
            if hasattr(widget, "windowTitle"):
                title = widget.windowTitle()
                titles.append(title)
                if ("3D Model Processing" in title
                        or "pythonOCC" in title
                        or "3D Viewer" in title):
                    return widget
    except Exception as e:
        pass
    return None


# ---------------------------------------------------------------------------
# Message box
# ---------------------------------------------------------------------------

def show_topmost_message(title, message, type="info"):
    """Display a modal, topmost message box.

    Parameters
    ----------
    type : str
        One of 'info', 'error', 'warning', 'question'.

    Returns
    -------
    'yes' / 'no' for question type, otherwise None.
    """
    QtCore, QtWidgets = _get_qt()
    QMessageBox = QtWidgets.QMessageBox
    parent = get_main_window()

    msg = QMessageBox(parent)
    msg.setWindowTitle(title)
    msg.setText(message)
    # NOTE: WindowStaysOnTopHint removed — on Windows it can cause the dialog
    # to render behind the main window or block input, appearing as a freeze.

    if type == "info":
        msg.setIcon(QMessageBox.Information)
        msg.setStandardButtons(QMessageBox.Ok)
    elif type == "error":
        msg.setIcon(QMessageBox.Critical)
        msg.setStandardButtons(QMessageBox.Ok)
    elif type == "warning":
        msg.setIcon(QMessageBox.Warning)
        msg.setStandardButtons(QMessageBox.Ok)
    elif type == "question":
        msg.setIcon(QMessageBox.Question)
        msg.setStandardButtons(QMessageBox.Yes | QMessageBox.No)

    result = (getattr(msg, "exec", None) or getattr(msg, "exec_"))()
    if type == "question":
        return "yes" if result == QMessageBox.Yes else "no"
    return None


# ---------------------------------------------------------------------------
# Status bar
# ---------------------------------------------------------------------------

def set_status_message(message):
    """Write non-critical feedback to the main window status bar."""
    win = get_main_window()
    if win and hasattr(win, "statusBar"):
        try:
            win.statusBar().showMessage(message)
        except Exception:
            pass
    print(message)


# ---------------------------------------------------------------------------
# Workflow panel
# ---------------------------------------------------------------------------

_workflow_dock = None
_workflow_status_label = None


def create_workflow_panel():
    """Create (or show) a right-side workflow panel."""
    global _workflow_dock, _workflow_status_label

    if _workflow_dock is not None:
        update_workflow_status("Workflow panel already available")
        return

    QtCore, QtWidgets = _get_qt()
    main_window = get_main_window()
    if not main_window or not hasattr(main_window, "addDockWidget"):
        return

    _workflow_dock = QtWidgets.QDockWidget("Workflow", main_window)
    _workflow_dock.setObjectName("WorkflowStatusDock")
    _workflow_dock.setAllowedAreas(
        QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)

    _workflow_status_label = QtWidgets.QLabel()
    _workflow_status_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    _workflow_status_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
    _workflow_status_label.setMinimumWidth(230)
    _workflow_status_label.setMargin(8)

    _workflow_dock.setWidget(_workflow_status_label)
    main_window.addDockWidget(QtCore.Qt.RightDockWidgetArea, _workflow_dock)
    update_workflow_status("Ready")


def update_workflow_status(message=None, state=None):
    """Refresh the workflow panel text.  *state* is an AppState-like object."""
    if _workflow_status_label is not None and state is not None:
        snap = build_workflow_snapshot(state)
        _workflow_status_label.setText(format_workflow_snapshot(snap))
    if message:
        set_status_message(message)


# ---------------------------------------------------------------------------
# Layer panel
# ---------------------------------------------------------------------------

_layer_dock = None
_layer_checkboxes = {}


def create_layer_panel(toggle_callbacks):
    """Create a checkbox-based layer panel.

    Parameters
    ----------
    toggle_callbacks : dict mapping layer key -> callable
        Each callable is invoked when the corresponding checkbox is clicked.
    """
    global _layer_dock, _layer_checkboxes

    if _layer_dock is not None:
        return

    QtCore, QtWidgets = _get_qt()
    main_window = get_main_window()
    if not main_window or not hasattr(main_window, "addDockWidget"):
        return

    _layer_dock = QtWidgets.QDockWidget("Layers", main_window)
    _layer_dock.setObjectName("LayerControlDock")
    _layer_dock.setAllowedAreas(
        QtCore.Qt.LeftDockWidgetArea | QtCore.Qt.RightDockWidgetArea)

    panel = QtWidgets.QWidget()
    layout = QtWidgets.QVBoxLayout(panel)
    layout.setContentsMargins(8, 8, 8, 8)
    layout.setSpacing(6)

    _layer_checkboxes = {}
    for key, label in [
        ("model", "Model / patches"),
        ("face_centers", "Face centers"),
        ("normal_lines", "Normal lines"),
        ("all_viewpoints", "All viewpoints"),
        ("optimal_viewpoints", "Optimal viewpoints"),
        ("planned_path", "Planned path"),
        ("sensor_volumes", "Sensor volumes"),
        ("obb_boxes", "OBB boxes"),
    ]:
        cb = QtWidgets.QCheckBox(label)
        cb.setChecked(True)
        callback = toggle_callbacks.get(key)
        if callback:
            cb.clicked.connect(callback)
        _layer_checkboxes[key] = cb
        layout.addWidget(cb)

    layout.addStretch(1)
    _layer_dock.setWidget(panel)
    main_window.addDockWidget(QtCore.Qt.RightDockWidgetArea, _layer_dock)


def sync_layer_panel(vis_flags):
    """Synchronise checkbox states with *vis_flags* (dict of key->bool)."""
    for key, value in vis_flags.items():
        cb = _layer_checkboxes.get(key)
        if cb is None:
            continue
        prev = cb.blockSignals(True)
        cb.setChecked(value)
        cb.blockSignals(prev)


# ---------------------------------------------------------------------------
# Parameter dialogs
# ---------------------------------------------------------------------------

def get_user_segment_params(parent=None):
    """Show a single dialog for U/V segmentation parameters.

    Returns
    -------
    (u, v) tuple of ints, or None if cancelled
    """
    try:
        QtCore, QtWidgets = _get_qt()
        if parent is None:
            parent = get_main_window()

        dialog = QtWidgets.QDialog(parent)
        dialog.setWindowTitle("Segmentation Parameters")
        layout = QtWidgets.QFormLayout(dialog)

        u_spin = QtWidgets.QSpinBox()
        u_spin.setRange(1, 100)
        u_spin.setValue(6)
        v_spin = QtWidgets.QSpinBox()
        v_spin.setRange(1, 100)
        v_spin.setValue(4)

        preview_label = QtWidgets.QLabel()

        def update_preview():
            preview_label.setText(
                "Requested maximum grid per original face: {} "
                "(trimmed/periodic faces may produce fewer valid patches)".format(
                    u_spin.value() * v_spin.value()))

        u_spin.valueChanged.connect(update_preview)
        v_spin.valueChanged.connect(update_preview)
        update_preview()

        layout.addRow("U segments:", u_spin)
        layout.addRow("V segments:", v_spin)
        layout.addRow("Preview:", preview_label)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)

        if (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))() == QtWidgets.QDialog.Accepted:
            u, v = u_spin.value(), v_spin.value()
        else:
            return None

        print("Using segmentation parameters: u={}, v={}".format(u, v))
        return u, v
    except Exception as e:
        print("Error getting segmentation parameters: {}".format(str(e)))
        return 6, 4


def get_sensor_parameters_dialog(parent=None, current_config=None):
    """Show a single dialog for sensor width/height/depth.

    Parameters
    ----------
    current_config : dict with 'width', 'height', 'depth'

    Returns
    -------
    dict or None (if cancelled)
    """
    try:
        QtCore, QtWidgets = _get_qt()
        if parent is None:
            parent = get_main_window()
        if current_config is None:
            current_config = {'width': 80, 'height': 60, 'depth': 80}

        dialog = QtWidgets.QDialog(parent)
        dialog.setWindowTitle("Sensor Parameters")
        layout = QtWidgets.QFormLayout(dialog)

        spins = {}
        for key in ("width", "height", "depth"):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(10.0, 200.0)
            s.setDecimals(2)
            s.setSingleStep(5.0)
            s.setValue(float(current_config.get(key, 80)))
            layout.addRow(key.capitalize() + ":", s)
            spins[key] = s

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)

        if (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))() != QtWidgets.QDialog.Accepted:
            return None

        return {k: spins[k].value() for k in ("width", "height", "depth")}
    except Exception as e:
        print("Error setting sensor parameters: {}".format(str(e)))
        show_topmost_message("Error",
                             "Error setting sensor parameters: {}".format(str(e)),
                             type="error")
        return None


# ---------------------------------------------------------------------------
# Usage instructions
# ---------------------------------------------------------------------------

USAGE_TEXT = """=== 3D Model Processing and Path Planning System ===

Usage Instructions:
1. A 3D viewer window will be created upon startup
2. Use the mouse in the 3D viewer to rotate and zoom the model
3. Please use functions in order:
   Import Model -> Segment Faces -> Get Centers -> Generate Viewpoints
   -> Filter Optimal Viewpoints -> Path Planning
4. Click 'File' menu to import/clear models or exit
5. Click 'Model Processing' menu for segmentation and viewpoint workflow
6. Click 'Collision Detection' menu for sensor and OBB operations
7. Click 'View' menu to show/hide layers
8. Click 'Path Planning' menu for sequential/greedy/ABC planning
9. Click 'Export' menu to save path points as CSV
"""


def show_usage_instructions():
    show_topmost_message("Usage Instructions", USAGE_TEXT)
