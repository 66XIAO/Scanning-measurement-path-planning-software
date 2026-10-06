"""Qt palette, icons and integrated command ribbon."""
from pathlib import Path
import sys
from i18n import tr

ASSET_ROOT = Path(__file__).resolve().parent / "assets"
ICON_ROOT = ASSET_ROOT / "icons"
ACTION_ICONS = {
    "action.new_workstation": "workstation-new", "action.open_workstation": "workstation-open",
    "action.save_workstation": "workstation-save", "action.save_workstation_as": "workstation-save",
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
    "action.view_fit": "camera", "action.view_iso": "camera",
    "action.view_front": "camera", "action.view_back": "camera",
    "action.view_left": "camera", "action.view_right": "camera",
    "action.view_top": "camera", "action.view_bottom": "camera",
    "action.view_orthographic": "camera", "action.view_perspective": "camera",
    "action.view_shaded": "layers", "action.view_wireframe": "bounding-box",
    "action.view_edges": "bounding-box",
    "action.view_screenshot": "camera",
    "action.configure_favorites": "settings", "action.configure_shortcuts": "settings",
    "action.configure_colors": "settings",
    "action.configure_robodk": "robot", "action.export_ui_config": "export",
    "action.import_ui_config": "import-model", "action.reset_ui_config": "clear",
    "action.show_mouse_guide": "help", "action.show_shortcuts": "help",
    "action.show_diagnostics": "help", "action.show_about": "help",
    "action.search_help": "help",
    "action.default_layout": "workflow", "action.focus_layout": "workflow",
    "action.toggle_fullscreen": "camera", "action.save_named_layout": "workstation-save",
    "action.restore_named_layout": "workstation-open",
    "action.delete_named_layout": "clear",
}
RIBBON_PAGES = (
    ("ribbon.file", (
        ("ribbon.group.workstation", ("action.new_workstation", "action.open_workstation", "action.save_workstation", "action.save_workstation_as")),
        ("ribbon.group.file", ("action.import_model", "action.clear_model")),
        ("ribbon.group.application", ("action.exit_program",)))),
    ("ribbon.path_planning", (
        ("ribbon.group.surface", ("action.select_single_face", "action.segment_faces", "action.get_centers")),
        ("ribbon.group.viewpoints", ("action.generate_center_viewpoints", "action.generate_candidate_viewpoints", "action.filter_optimal_viewpoints")),
        ("ribbon.group.path", ("action.connect_sequentially", "action.solve_greedy", "action.solve_abc", "action.solve_mscga", "action.plan_path")),
        ("ribbon.group.speed", ("action.plan_path_speeds",)))),
    ("ribbon.collision", (
        ("ribbon.group.sensor", ("action.toggle_collision_detection", "action.set_sensor_parameters", "action.create_sensor_volumes")),
        ("ribbon.group.bounds", ("action.generate_min_bounding_boxes", "action.execute_collision_detection", "action.demo_collision_detection")))),
    ("ribbon.calibration_export", (
        ("ribbon.group.calibration", ("action.capture_robodk_mapping", "action.load_extrinsic", "action.clear_extrinsic")),
        ("ribbon.group.robot", ("action.analyze_ur10_reachability", "action.repair_ur10_reachability", "action.import_planned_path_robodk", "action.import_speed_plan_robodk")),
        ("ribbon.group.export", ("action.export_path_csv", "action.export_speed_plan_csv")))),
    ("ribbon.view", (
        ("ribbon.group.camera", ("action.view_fit", "action.view_iso", "action.view_front", "action.view_back", "action.view_left", "action.view_right", "action.view_top", "action.view_bottom")),
        ("ribbon.group.display", ("action.view_orthographic", "action.view_perspective", "action.view_shaded", "action.view_wireframe", "action.view_edges", "action.view_screenshot")),
        ("ribbon.group.panels", ("action.create_workflow_panel", "action.create_layer_panel", "action.create_operation_panel")),
        ("ribbon.group.layout", ("action.default_layout", "action.focus_layout", "action.toggle_fullscreen", "action.save_named_layout", "action.restore_named_layout", "action.delete_named_layout")),
        ("ribbon.group.layers", ("action.toggle_model_layer", "action.toggle_workpiece_coordinate_system", "action.toggle_face_centers", "action.toggle_normal_lines", "action.toggle_all_viewpoints", "action.toggle_optimal_viewpoints", "action.toggle_planned_path", "action.toggle_sensor_volumes", "action.toggle_obb_boxes")))),
    ("ribbon.settings", (
        ("ribbon.group.language", ("language.chinese", "language.english", "language.load_json")),
        ("ribbon.group.preferences", ("action.configure_favorites", "action.configure_shortcuts", "action.configure_colors", "action.configure_robodk", "action.export_ui_config", "action.import_ui_config", "action.reset_ui_config")))),
    ("ribbon.help", (
        ("ribbon.group.support", ("action.search_help", "action.show_usage", "action.show_mouse_guide", "action.show_shortcuts", "action.show_diagnostics", "action.show_about")),)),
)
SIZES = {"compact": (24, 9, 86), "standard": (32, 10, 102), "large": (40, 11, 118)}
DROPDOWN_GROUPS = {
    "ribbon.group.workstation": "ribbon.dropdown.workstation",
    "ribbon.group.layers": "ribbon.dropdown.layers",
    "ribbon.group.camera": "ribbon.dropdown.camera",
    "ribbon.group.layout": "ribbon.dropdown.layout",
    "ribbon.group.bounds": "ribbon.dropdown.collision",
}
_ribbon = None
SHORT_LABELS = {
    "action.save_workstation_as": "save_as",
    "action.select_single_face": "select_face",
    "action.generate_center_viewpoints": "center_viewpoints",
    "action.generate_candidate_viewpoints": "candidate_viewpoints",
    "action.filter_optimal_viewpoints": "filter_viewpoints",
    "action.connect_sequentially": "sequential",
    "action.solve_greedy": "greedy",
    "action.solve_abc": "abc",
    "action.solve_mscga": "mscga",
    "action.plan_path_speeds": "speed",
    "action.capture_robodk_mapping": "capture_mapping",
    "action.analyze_ur10_reachability": "reachability",
    "action.repair_ur10_reachability": "repair",
    "action.import_planned_path_robodk": "import_path",
    "action.import_speed_plan_robodk": "import_speed",
    "action.export_speed_plan_csv": "export_speed",
    "action.configure_favorites": "favorites",
    "action.configure_shortcuts": "shortcuts",
    "action.configure_colors": "colors",
    "action.configure_robodk": "robodk",
    "action.export_ui_config": "export_config",
    "action.import_ui_config": "import_config",
    "action.reset_ui_config": "reset_config",
    "action.view_screenshot": "screenshot",
}


def _button_label(action):
    key = SHORT_LABELS.get(str(action.property("i18n_key")))
    return tr("ribbon.short." + key) if key else action.text().replace("&", "")


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
    settings = QtCore.QSettings(QtCore.QSettings.IniFormat,
                                QtCore.QSettings.UserScope,
                                "ScanningPathPlanner", "UI")
    scheme = str(settings.value("colorScheme", "light"))
    stylesheet = """
        QMainWindow, QDialog { background: #F3F5F7; color: #263238; }
        QTabWidget::pane, QToolBar { background: #FFFFFF; border: 0; border-bottom: 1px solid #D4DCE3; }
        QTabBar::tab { background: #F3F5F7; color: #263238; padding: 8px 16px; border: 1px solid #D4DCE3; }
        QTabBar::tab:selected { background: #FFFFFF; color: #21699F; border-bottom: 2px solid #2878B8; }
        QToolButton { color: #263238; background: transparent; border: 1px solid transparent; border-radius: 5px; padding: 5px; }
        QToolButton:hover { background: #E7F2F9; border-color: #8BBEDC; }
        QToolButton:pressed { background: #D3E9F6; }
        QMenu { background: #FFFFFF; color: #263238; border: 1px solid #D4DCE3; }
        QMenu::item { padding: 6px 24px; }
        QMenu::item:selected { background: #E7F2F9; }
        QDockWidget { color: #263238; }
        QLabel, QCheckBox, QPlainTextEdit, QTreeWidget { color: #263238; }
        QDockWidget::title { background: #E8EDF1; padding: 7px; }
        QScrollArea, QScrollArea > QWidget > QWidget { background: #FFFFFF; border: 0; }
        QGroupBox { color: #263238; border: 1px solid #D4DCE3; border-radius: 5px; margin-top: 10px; padding: 8px; font-weight: 600; }
        QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }
        QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background: #FFFFFF; color: #263238; border: 1px solid #AEBCC7; border-radius: 4px; padding: 4px; selection-background-color: #9ACCE8; }
        QPushButton { background: #E7F2F9; color: #263238; border: 1px solid #8BBEDC; border-radius: 4px; padding: 5px 12px; }
        QPushButton:hover { background: #D3E9F6; }
        QStatusBar { background: #E8EDF1; color: #263238; border-top: 1px solid #D4DCE3; }
        QToolTip { background: #FFFFFF; color: #263238; border: 1px solid #8BBEDC; }
    """
    palettes = {
        "dark": {"#F3F5F7": "#20272E", "#FFFFFF": "#29333C",
                 "#263238": "#EBF3F8", "#D4DCE3": "#526371",
                 "#E8EDF1": "#34424C", "#E7F2F9": "#35536A",
                 "#D3E9F6": "#45667C", "#AEBCC7": "#7D93A3",
                 "#8BBEDC": "#61A7D0", "#9ACCE8": "#3774A0",
                 "#21699F": "#8DC9ED"},
        "high_contrast": {"#F3F5F7": "#000000", "#FFFFFF": "#000000",
                          "#263238": "#FFFFFF", "#D4DCE3": "#FFFFFF",
                          "#E8EDF1": "#000000", "#E7F2F9": "#202020",
                          "#D3E9F6": "#404040", "#AEBCC7": "#FFFFFF",
                          "#8BBEDC": "#FFFF00", "#9ACCE8": "#FFFF00",
                          "#21699F": "#FFFF00", "#2878B8": "#FFFF00"},
    }
    if scheme in palettes:
        import re
        mapping = palettes[scheme]
        stylesheet = re.sub("|".join(mapping), lambda match: mapping[match.group()],
                            stylesheet)
    for key, old in (("accentColor", "#2878B8"),
                     ("surfaceColor", palettes.get(scheme, {}).get("#F3F5F7", "#F3F5F7")),
                     ("textColor", palettes.get(scheme, {}).get("#263238", "#263238"))):
        chosen = str(settings.value(key, ""))
        if QtGui.QColor(chosen).isValid():
            stylesheet = stylesheet.replace(old, chosen)
    app.setStyleSheet(stylesheet)
    from ui_panels import apply_panel_scheme
    apply_panel_scheme(scheme,
                       str(settings.value("surfaceColor", "")),
                       str(settings.value("textColor", "")))
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
                menu_button.setObjectName("RibbonDropdown_" + group_key)
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
                button.setText(_button_label(action))
                button.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
                button.setToolTip(action.text())
                buttons.append(button)
                box_row.addWidget(button)
            row.addWidget(box)
        row.addStretch(1)
        scroll.setWidget(page)
        tabs.addTab(scroll, tr(page_key))
    settings_page = tabs.widget(5).widget()
    settings_row = settings_page.layout()
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
    settings_row.insertWidget(settings_row.count() - 1, size_box)
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
               "settings": settings, "QtCore": QtCore, "collapse": collapse}
    set_ribbon_size(size)
    return toolbar


def set_ribbon_size(size):
    if _ribbon is None or size not in SIZES:
        return
    icon_size, font_size, minimum_width = SIZES[size]
    for button in _ribbon["buttons"]:
        button.setIconSize(_ribbon["QtCore"].QSize(icon_size, icon_size))
        font = button.font()
        font.setPointSize(font_size)
        button.setFont(font)
        metrics = button.fontMetrics()
        label_width = metrics.horizontalAdvance(button.text()) + 24
        button.setFixedWidth(max(minimum_width, label_width))
        button.setMinimumHeight(icon_size + metrics.height() + 22)
    expanded_height = icon_size + font_size * 6 + 110
    _ribbon["expandedHeight"] = expanded_height
    if not (_ribbon["tabs"].property("collapsed") == True):
        _ribbon["tabs"].setFixedHeight(expanded_height)
    _ribbon["settings"].setValue("ribbonSize", size)
    _ribbon["settings"].sync()


def ribbon_page(index):
    if _ribbon is not None:
        if isinstance(index, str):
            index = next((i for i, page in enumerate(RIBBON_PAGES)
                          if page[0] == index), 0)
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
            button.setText(_button_label(button.defaultAction()))
            button.setToolTip(button.defaultAction().text())
    for button, key in _ribbon["dropdowns"]:
        button.setText(tr(DROPDOWN_GROUPS[key]))
        button.setToolTip(tr(key))
    tabs = _ribbon["tabs"]
    for mode in SIZES:
        size_button = tabs.findChild(type(_ribbon["collapse"]), "RibbonSize_" + mode)
        size_button.setText(tr("ribbon.size." + mode))
    collapsed = tabs.property("collapsed") == True
    _ribbon["collapse"].setToolTip(tr("ribbon.expand" if collapsed else "ribbon.collapse"))
    current_size = str(_ribbon["settings"].value("ribbonSize", "standard"))
    set_ribbon_size(current_size if current_size in SIZES else "standard")
