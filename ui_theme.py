"""Shared visual theme, SVG icon loading, and the primary action toolbar."""

from pathlib import Path


ASSET_ROOT = Path(__file__).resolve().parent / "assets"
ICON_ROOT = ASSET_ROOT / "icons"

ACTION_ICONS = {
    "action.import_model": "import-model", "action.clear_model": "clear",
    "action.exit_program": "exit", "action.select_single_face": "select-face",
    "action.segment_faces": "segment", "action.get_centers": "centers",
    "action.generate_center_viewpoints": "viewpoints",
    "action.generate_candidate_viewpoints": "viewpoints",
    "action.filter_optimal_viewpoints": "optimize", "action.plan_path": "path",
    "action.toggle_collision_detection": "collision",
    "action.set_sensor_parameters": "sensor", "action.create_sensor_volumes": "sensor",
    "action.generate_min_bounding_boxes": "bounding-box",
    "action.execute_collision_detection": "collision",
    "action.demo_collision_detection": "collision", "action.toggle_model_layer": "layers",
    "action.toggle_workpiece_coordinate_system": "axes",
    "action.toggle_face_centers": "centers", "action.toggle_normal_lines": "normals",
    "action.toggle_all_viewpoints": "viewpoints",
    "action.toggle_optimal_viewpoints": "optimize", "action.toggle_planned_path": "path",
    "action.toggle_sensor_volumes": "sensor", "action.toggle_obb_boxes": "bounding-box",
    "action.create_workflow_panel": "workflow", "action.create_layer_panel": "layers",
    "action.connect_sequentially": "path", "action.solve_greedy": "optimize",
    "action.solve_abc": "optimize", "action.solve_mscga": "optimize",
    "action.analyze_ur10_reachability": "robot",
    "action.repair_ur10_reachability": "repair",
    "action.import_planned_path_robodk": "robot", "action.plan_path_speeds": "speed",
    "action.export_speed_plan_csv": "export", "action.import_speed_plan_robodk": "robot",
    "action.capture_robodk_mapping": "calibration", "action.load_extrinsic": "calibration",
    "action.clear_extrinsic": "clear", "action.export_path_csv": "export",
    "language.english": "language", "language.chinese": "language",
    "language.load_json": "language", "action.show_usage": "help",
}

TOOLBAR_GROUPS = (
    ("action.import_model", "action.clear_model"),
    ("action.segment_faces", "action.generate_candidate_viewpoints",
     "action.filter_optimal_viewpoints"),
    ("action.solve_mscga", "action.execute_collision_detection",
     "action.plan_path_speeds"),
    ("action.analyze_ur10_reachability", "action.export_path_csv"),
)


def load_icon(QtGui, name):
    path = ASSET_ROOT / "app_icon.svg" if name == "app" else ICON_ROOT / (name + ".svg")
    return QtGui.QIcon(str(path)) if path.is_file() else QtGui.QIcon()


def decorate_action(QtGui, action, translation_key):
    icon_name = ACTION_ICONS.get(translation_key)
    if icon_name:
        action.setIcon(load_icon(QtGui, icon_name))


def apply_application_theme(QtCore, QtGui, QtWidgets, window):
    """Apply a restrained engineering dark theme without styling the OCC canvas."""
    app = QtWidgets.QApplication.instance()
    if app is None:
        return
    app.setApplicationName("Scanning Measurement Path Planner")
    app.setWindowIcon(load_icon(QtGui, "app"))
    try:
        app.setStyle("Fusion")
    except Exception:
        pass
    app.setStyleSheet("""
        QMainWindow, QDialog { background: #102638; color: #DFF8FF; }
        QMenuBar { background: #163449; color: #DFF8FF; padding: 3px; }
        QMenuBar::item { padding: 6px 10px; border-radius: 4px; }
        QMenuBar::item:selected, QMenu::item:selected { background: #24506A; }
        QMenu { background: #163449; color: #DFF8FF; border: 1px solid #2C6078; padding: 5px; }
        QMenu::item { padding: 6px 26px 6px 8px; border-radius: 3px; }
        QToolBar { background: #132E42; border: 0; border-bottom: 1px solid #2C6078; spacing: 3px; padding: 4px; }
        QToolButton { color: #DFF8FF; border: 1px solid transparent; border-radius: 5px; padding: 5px; }
        QToolButton:hover { background: #214A62; border-color: #39738D; }
        QToolButton:pressed { background: #0D2131; }
        QDockWidget { color: #DFF8FF; font-weight: 600; }
        QDockWidget::title { background: #163449; padding: 7px; border-bottom: 1px solid #2C6078; }
        QDockWidget QWidget { background: #132E42; color: #DFF8FF; }
        QLabel, QCheckBox, QGroupBox { color: #DFF8FF; }
        QCheckBox { spacing: 7px; padding: 2px; }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
            background: #0D2131; color: #E9FBFF; border: 1px solid #39738D;
            border-radius: 4px; padding: 5px; selection-background-color: #2B7188;
        }
        QPushButton { background: #1F5269; color: #F2FDFF; border: 1px solid #4A91A8; border-radius: 5px; padding: 6px 14px; }
        QPushButton:hover { background: #276A81; }
        QStatusBar { background: #0D2131; color: #A9DDE8; border-top: 1px solid #2C6078; }
        QToolTip { background: #EAFBFF; color: #102638; border: 1px solid #55A9BF; padding: 4px; }
    """)
    if window is not None:
        window.setWindowIcon(load_icon(QtGui, "app"))
        window.setIconSize(QtCore.QSize(20, 20))


def create_primary_toolbar(QtCore, QtWidgets, window, actions):
    if window is None or not hasattr(window, "addToolBar"):
        return None
    existing = window.findChild(QtWidgets.QToolBar, "PrimaryWorkflowToolbar")
    if existing is not None:
        return existing
    toolbar = QtWidgets.QToolBar(window)
    toolbar.setObjectName("PrimaryWorkflowToolbar")
    toolbar.setWindowTitle("Workflow")
    toolbar.setMovable(True)
    toolbar.setFloatable(False)
    toolbar.setIconSize(QtCore.QSize(22, 22))
    for group_index, group in enumerate(TOOLBAR_GROUPS):
        if group_index:
            toolbar.addSeparator()
        for key in group:
            action = actions.get(key)
            if action is not None:
                toolbar.addAction(action)
    window.addToolBar(QtCore.Qt.TopToolBarArea, toolbar)
    return toolbar
