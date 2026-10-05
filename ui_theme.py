"""Qt palette, icons and integrated command ribbon."""
from pathlib import Path
import sys
import textwrap
from i18n import tr

ASSET_ROOT = Path(__file__).resolve().parent / "assets"
ICON_ROOT = ASSET_ROOT / "icons"
ACTION_ICONS = {
    "action.new_workstation": "clear", "action.open_workstation": "import-model",
    "action.save_workstation": "export", "action.save_workstation_as": "export",
    "action.create_operation_panel": "workflow",
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
RIBBON_PAGES = (
    ("ribbon.model", (
        ("ribbon.group.workstation", ("action.new_workstation", "action.open_workstation", "action.save_workstation", "action.save_workstation_as")),
        ("ribbon.group.file", ("action.import_model", "action.clear_model")),
        ("ribbon.group.surface", ("action.select_single_face", "action.segment_faces", "action.get_centers")))),
    ("ribbon.scan", (
        ("ribbon.group.viewpoints", ("action.generate_center_viewpoints", "action.generate_candidate_viewpoints", "action.filter_optimal_viewpoints")),
        ("ribbon.group.path", ("action.connect_sequentially", "action.solve_greedy", "action.solve_abc", "action.solve_mscga", "action.plan_path")))),
    ("ribbon.inspect", (
        ("ribbon.group.sensor", ("action.toggle_collision_detection", "action.set_sensor_parameters", "action.create_sensor_volumes", "action.generate_min_bounding_boxes", "action.execute_collision_detection", "action.demo_collision_detection")),
        ("ribbon.group.speed", ("action.plan_path_speeds",)))),
    ("ribbon.robodk", (
        ("ribbon.group.calibration", ("action.capture_robodk_mapping", "action.load_extrinsic", "action.clear_extrinsic")),
        ("ribbon.group.robot", ("action.analyze_ur10_reachability", "action.repair_ur10_reachability", "action.import_planned_path_robodk", "action.import_speed_plan_robodk")),
        ("ribbon.group.export", ("action.export_path_csv", "action.export_speed_plan_csv")))),
    ("ribbon.view", (
        ("ribbon.group.panels", ("action.create_workflow_panel", "action.create_layer_panel", "action.create_operation_panel")),
        ("ribbon.group.layers", ("action.toggle_model_layer", "action.toggle_workpiece_coordinate_system", "action.toggle_face_centers", "action.toggle_normal_lines", "action.toggle_all_viewpoints", "action.toggle_optimal_viewpoints", "action.toggle_planned_path", "action.toggle_sensor_volumes", "action.toggle_obb_boxes")),
        ("ribbon.group.language", ("language.chinese", "language.english", "language.load_json", "action.show_usage", "action.exit_program")))),
)
SIZES = {"compact": (24, 9, 94), "standard": (32, 10, 110), "large": (40, 11, 126)}
DROPDOWN_GROUPS = {
    "ribbon.group.workstation": "ribbon.dropdown.workstation",
    "ribbon.group.layers": "ribbon.dropdown.layers",
    "ribbon.group.language": "ribbon.dropdown.language",
}
_ribbon = None


def _button_label(label):
    label = label.replace("&", "")
    width = 4 if any("\u4e00" <= char <= "\u9fff" for char in label) else 9
    return textwrap.fill(label, width=width, max_lines=3, placeholder="…")


def load_icon(QtGui, name):
    path = ASSET_ROOT / "app_icon.svg" if name == "app" else ICON_ROOT / (name + ".svg")
    return QtGui.QIcon(str(path)) if path.is_file() else QtGui.QIcon()


def decorate_action(QtGui, action, translation_key):
    icon_name = ACTION_ICONS.get(translation_key)
    if icon_name:
        action.setIcon(load_icon(QtGui, icon_name))


def apply_application_theme(QtCore, QtGui, QtWidgets, window):
    app = QtWidgets.QApplication.instance()
    if app is None:
        return
    app.setApplicationName("Scanning Measurement Path Planner")
    app.setWindowIcon(load_icon(QtGui, "app"))
    app.setStyle("Fusion")
    app.setFont(QtGui.QFont("Microsoft YaHei UI" if sys.platform == "win32" else "Sans Serif", 10))
    app.setStyleSheet("""
        QMainWindow, QDialog { background: #F3F5F7; color: #263238; }
        QTabWidget::pane, QToolBar { background: #FFFFFF; border: 0; border-bottom: 1px solid #D4DCE3; }
        QTabBar::tab { background: #F3F5F7; padding: 8px 16px; border: 1px solid #D4DCE3; }
        QTabBar::tab:selected { background: #FFFFFF; color: #21699F; border-bottom: 2px solid #2878B8; }
        QToolButton { color: #263238; background: transparent; border: 1px solid transparent; border-radius: 5px; padding: 5px; }
        QToolButton:hover { background: #E7F2F9; border-color: #8BBEDC; }
        QToolButton:pressed { background: #D3E9F6; }
        QMenu { background: #FFFFFF; color: #263238; border: 1px solid #D4DCE3; }
        QMenu::item { padding: 6px 24px; }
        QMenu::item:selected { background: #E7F2F9; }
        QDockWidget { color: #263238; }
        QDockWidget::title { background: #E8EDF1; padding: 7px; }
        QScrollArea, QScrollArea > QWidget > QWidget { background: #FFFFFF; border: 0; }
        QGroupBox { color: #263238; border: 1px solid #D4DCE3; border-radius: 5px; margin-top: 10px; padding: 8px; font-weight: 600; }
        QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background: #FFFFFF; color: #263238; border: 1px solid #AEBCC7; border-radius: 4px; padding: 4px; selection-background-color: #9ACCE8; }
        QPushButton { background: #E7F2F9; color: #263238; border: 1px solid #8BBEDC; border-radius: 4px; padding: 5px 12px; }
        QPushButton:hover { background: #D3E9F6; }
        QStatusBar { background: #E8EDF1; color: #263238; border-top: 1px solid #D4DCE3; }
        QToolTip { background: #FFFFFF; color: #263238; border: 1px solid #8BBEDC; }
    """)
    if window is not None:
        window.setWindowIcon(load_icon(QtGui, "app"))


def create_primary_toolbar(QtCore, QtWidgets, window, actions):
    """Reuse menu QAction objects in one tabbed command area."""
    global _ribbon
    if window is None:
        return None
    old = window.findChild(QtWidgets.QToolBar, "PrimaryWorkflowToolbar")
    if old is not None:
        return old
    toolbar = QtWidgets.QToolBar(window)
    toolbar.setObjectName("PrimaryWorkflowToolbar")
    toolbar.setMovable(False)
    tabs = QtWidgets.QTabWidget(toolbar)
    tabs.setObjectName("CommandRibbon")
    tabs.setDocumentMode(True)
    tabs.setUsesScrollButtons(True)
    tabs.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    settings = QtCore.QSettings(
        QtCore.QSettings.IniFormat, QtCore.QSettings.UserScope,
        "ScanningPathPlanner", "UI")
    size = str(settings.value("ribbonSize", "standard"))
    if size not in SIZES:
        size = "standard"
    buttons, group_titles, dropdowns = [], [], []
    for page_key, groups in RIBBON_PAGES:
        scroll = QtWidgets.QScrollArea(tabs)
        scroll.setWidgetResizable(True)
        scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        page = QtWidgets.QWidget()
        row = QtWidgets.QHBoxLayout(page)
        row.setContentsMargins(8, 5, 8, 5)
        row.setSpacing(8)
        for group_key, keys in groups:
            box = QtWidgets.QGroupBox(tr(group_key))
            group_titles.append((box, group_key))
            box_row = QtWidgets.QHBoxLayout(box)
            box_row.setContentsMargins(5, 8, 5, 4)
            if group_key in DROPDOWN_GROUPS:
                menu_button = QtWidgets.QToolButton(box)
                menu_button.setText(tr(DROPDOWN_GROUPS[group_key]))
                menu_button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
                menu_button.setPopupMode(QtWidgets.QToolButton.InstantPopup)
                menu = QtWidgets.QMenu(menu_button)
                for key in keys:
                    action = actions.get(key)
                    if action is not None:
                        menu.addAction(action)
                if menu.actions():
                    menu_button.setIcon(menu.actions()[0].icon())
                menu_button.setMenu(menu)
                buttons.append(menu_button)
                dropdowns.append((menu_button, group_key))
                box_row.addWidget(menu_button)
                row.addWidget(box)
                continue
            for key in keys:
                action = actions.get(key)
                if action is None:
                    continue
                button = QtWidgets.QToolButton(box)
                button.setDefaultAction(action)
                button.setText(_button_label(action.text()))
                button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
                button.setToolTip(action.text())
                buttons.append(button)
                box_row.addWidget(button)
            row.addWidget(box)
        row.addStretch(1)
        scroll.setWidget(page)
        tabs.addTab(scroll, tr(page_key))
    view_page = tabs.widget(4).widget()
    view_row = view_page.layout()
    size_box = QtWidgets.QGroupBox(tr("ribbon.group.size"))
    group_titles.append((size_box, "ribbon.group.size"))
    size_row = QtWidgets.QHBoxLayout(size_box)
    for mode in SIZES:
        choice = QtWidgets.QToolButton(size_box)
        choice.setObjectName("RibbonSize_" + mode)
        choice.setText(tr("ribbon.size." + mode))
        choice.setToolButtonStyle(QtCore.Qt.ToolButtonTextOnly)
        choice.clicked.connect(lambda checked=False, selected=mode: set_ribbon_size(selected))
        size_row.addWidget(choice)
    view_row.insertWidget(view_row.count() - 1, size_box)
    collapse = QtWidgets.QToolButton(tabs)
    collapse.setObjectName("RibbonCollapse")
    collapse.setText("⌃")
    collapse.setToolTip(tr("ribbon.collapse"))
    def toggle_collapsed():
        collapsed = tabs.property("collapsed") == True
        tabs.setProperty("collapsed", not collapsed)
        if collapsed:
            tabs.setFixedHeight(_ribbon["expandedHeight"])
        else:
            tabs.setFixedHeight(tabs.tabBar().sizeHint().height() + 5)
        collapse.setText("⌃" if collapsed else "⌄")
        collapse.setToolTip(tr("ribbon.collapse" if collapsed else "ribbon.expand"))
    collapse.clicked.connect(toggle_collapsed)
    tabs.setCornerWidget(collapse, QtCore.Qt.TopRightCorner)
    toolbar.addWidget(tabs)
    toolbar.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
    window.addToolBar(QtCore.Qt.TopToolBarArea, toolbar)
    window.menuBar().hide()
    _ribbon = {"tabs": tabs, "groups": group_titles, "buttons": buttons,
               "dropdowns": dropdowns,
               "settings": settings, "QtCore": QtCore}
    set_ribbon_size(size)
    return toolbar


def set_ribbon_size(size):
    if _ribbon is None or size not in SIZES:
        return
    icon_size, font_size, width = SIZES[size]
    for button in _ribbon["buttons"]:
        button.setIconSize(_ribbon["QtCore"].QSize(icon_size, icon_size))
        button.setFixedWidth(width)
        button.setMinimumHeight(icon_size + font_size * 6 + 24)
        font = button.font()
        font.setPointSize(font_size)
        button.setFont(font)
    expanded_height = icon_size + font_size * 8 + 140
    _ribbon["expandedHeight"] = expanded_height
    if not (_ribbon["tabs"].property("collapsed") == True):
        _ribbon["tabs"].setFixedHeight(expanded_height)
    _ribbon["settings"].setValue("ribbonSize", size)
    _ribbon["settings"].sync()


def ribbon_page(index):
    if _ribbon is not None:
        _ribbon["tabs"].setCurrentIndex(index)


def retranslate_ribbon():
    if _ribbon is None:
        return
    for index, (key, _) in enumerate(RIBBON_PAGES):
        _ribbon["tabs"].setTabText(index, tr(key))
    for box, key in _ribbon["groups"]:
        box.setTitle(tr(key))
    for button in _ribbon["buttons"]:
        if button.defaultAction() is not None:
            button.setText(_button_label(button.defaultAction().text()))
            button.setToolTip(button.defaultAction().text())
    for button, key in _ribbon["dropdowns"]:
        button.setText(tr(DROPDOWN_GROUPS[key]))
        button.setToolTip(tr(key))
    tabs = _ribbon["tabs"]
    for mode in SIZES:
        tabs.findChild(type(tabs.cornerWidget()), "RibbonSize_" + mode).setText(tr("ribbon.size." + mode))
    collapsed = tabs.property("collapsed") == True
    tabs.cornerWidget().setToolTip(tr("ribbon.expand" if collapsed else "ribbon.collapse"))
