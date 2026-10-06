"""Local UI preferences and viewer commands; no workstation data is stored here."""
import json
import os
import tempfile
from pathlib import Path

from i18n import tr
from local_frame_style import (DEFAULT_LOCAL_FRAME_STYLE,
                               normalize_local_frame_style,
                               read_local_frame_style, write_local_frame_style)

CONFIG_VERSION = 1
DEFAULT_FAVORITES = ["action.import_model", "action.save_workstation",
                     "action.segment_faces", "action.filter_optimal_viewpoints"]


class WorkspaceUI:
    def __init__(self, QtCore, QtGui, QtWidgets, window, display, actions):
        self.QtCore, self.QtGui, self.QtWidgets = QtCore, QtGui, QtWidgets
        self.window, self.display, self.actions = window, display, actions
        self.default_shortcuts = {key: action.shortcut().toString()
                                  for key, action in actions.items()}
        self.search_aliases = {}
        from ui_theme import RIBBON_PAGES
        self.categories = {key: (page, group)
                           for page, groups in RIBBON_PAGES
                           for group, keys in groups for key in keys}
        for locale in ("en", "zh_CN"):
            catalog = Path(__file__).resolve().parent / "locales" / (locale + ".json")
            try:
                labels = json.loads(catalog.read_text(encoding="utf-8"))["strings"]
                for key in actions:
                    self.search_aliases.setdefault(key, []).append(labels.get(key, ""))
            except (OSError, ValueError, KeyError):
                pass
        self.settings = QtCore.QSettings(QtCore.QSettings.IniFormat,
                                         QtCore.QSettings.UserScope,
                                         "ScanningPathPlanner", "UI")
        self.search = None
        self.favorites_button = None
        self.redraw_scene = None
        self.recovery_shortcut = QtWidgets.QShortcut(
            QtGui.QKeySequence("Ctrl+Shift+0"), window)
        self.recovery_shortcut.setContext(QtCore.Qt.ApplicationShortcut)
        self.recovery_shortcut.activated.connect(self.default_layout)

    def favorites(self):
        keys = self.settings.value("favorites", DEFAULT_FAVORITES)
        if isinstance(keys, str):
            keys = [keys]
        return [key for key in keys if key in self.actions]

    def attach_header(self, ribbon):
        box = self.QtWidgets.QWidget(ribbon)
        row = self.QtWidgets.QHBoxLayout(box)
        row.setContentsMargins(0, 0, 4, 0)
        row.setSpacing(3)
        self.favorites_button = self.QtWidgets.QToolButton(box)
        self.favorites_button.setObjectName("FavoriteCommands")
        self.favorites_button.setText("★")
        self.favorites_button.setPopupMode(self.QtWidgets.QToolButton.InstantPopup)
        row.addWidget(self.favorites_button)
        search = self.QtWidgets.QLineEdit(box)
        search.setObjectName("CommandSearch")
        search.setPlaceholderText(tr("ui.search.placeholder"))
        search.setFixedWidth(180)
        search.returnPressed.connect(self.show_search)
        row.addWidget(search)
        self.search = search
        collapse = ribbon.cornerWidget()
        collapse.setParent(box)
        row.addWidget(collapse)
        ribbon.setCornerWidget(box, self.QtCore.Qt.TopRightCorner)
        self.refresh_favorites()

    def refresh_favorites(self):
        if self.favorites_button is None:
            return
        menu = self.QtWidgets.QMenu(self.favorites_button)
        for key in self.favorites():
            menu.addAction(self.actions[key])
        if menu.actions():
            menu.addSeparator()
        configure = menu.addAction(tr("action.configure_favorites"))
        configure.triggered.connect(self.configure_favorites)
        self.favorites_button.setMenu(menu)
        self.favorites_button.setToolTip(tr("ui.favorites.title"))

    def show_search(self):
        query = self.search.text().strip().casefold() if self.search else ""
        dialog = self.QtWidgets.QDialog(self.window)
        dialog.setWindowTitle(tr("ui.search.title"))
        dialog.resize(570, 420)
        layout = self.QtWidgets.QVBoxLayout(dialog)
        entry = self.QtWidgets.QLineEdit(dialog)
        entry.setText(query)
        entry.setPlaceholderText(tr("ui.search.placeholder"))
        results = self.QtWidgets.QListWidget(dialog)
        layout.addWidget(entry)
        layout.addWidget(results)

        def refresh():
            results.clear()
            needle = entry.text().strip().casefold()
            for key, action in self.actions.items():
                haystack = " ".join((key, action.text(), action.toolTip(),
                                     action.shortcut().toString(),
                                     *self.search_aliases.get(key, ()))).casefold()
                if needle and needle not in haystack:
                    continue
                page, group = self.categories.get(key, ("ribbon.help", "ribbon.group.support"))
                label = "{}  ·  {} / {}".format(action.text(), tr(page), tr(group))
                if not action.shortcut().isEmpty():
                    label += "  [" + action.shortcut().toString() + "]"
                if not action.isEnabled():
                    label += "  (" + tr("ui.search.disabled") + ")"
                item = self.QtWidgets.QListWidgetItem(action.icon(), label)
                item.setData(self.QtCore.Qt.UserRole, key)
                item.setToolTip(action.toolTip() or action.text())
                results.addItem(item)

        def activate(item):
            action = self.actions[item.data(self.QtCore.Qt.UserRole)]
            if action.isEnabled():
                dialog.accept()
                action.trigger()

        entry.textChanged.connect(refresh)
        results.itemActivated.connect(activate)
        results.itemDoubleClicked.connect(activate)
        refresh()
        (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))()

    def show_help(self):
        dialog = self.QtWidgets.QDialog(self.window)
        dialog.setWindowTitle(tr("action.search_help"))
        dialog.resize(700, 560)
        layout = self.QtWidgets.QVBoxLayout(dialog)
        query = self.QtWidgets.QLineEdit(dialog)
        query.setPlaceholderText(tr("ui.help.search"))
        body = self.QtWidgets.QPlainTextEdit(dialog)
        body.setReadOnly(True)
        content = "\n\n".join((tr("help.usage_text"),
                                  tr("help.mouse_guide"),
                                  tr("help.about"),
                                  "Ctrl+Shift+0 — " + tr("action.default_layout"),
                                  "\n".join("{} — {}".format(
                                      action.shortcut().toString(), action.text())
                                      for action in self.actions.values()
                                      if not action.shortcut().isEmpty())))
        body.setPlainText(content)
        layout.addWidget(query)
        layout.addWidget(body)

        def find_next():
            if not query.text():
                return
            if not body.find(query.text()):
                cursor = body.textCursor()
                cursor.movePosition(self.QtGui.QTextCursor.Start)
                body.setTextCursor(cursor)
                body.find(query.text())

        query.returnPressed.connect(find_next)
        query.textChanged.connect(find_next)
        (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))()

    def configure_favorites(self):
        dialog = self.QtWidgets.QDialog(self.window)
        dialog.setWindowTitle(tr("ui.favorites.title"))
        dialog.resize(450, 500)
        layout = self.QtWidgets.QVBoxLayout(dialog)
        available = self.QtWidgets.QListWidget(dialog)
        available.setDragDropMode(self.QtWidgets.QAbstractItemView.InternalMove)
        available.setSelectionMode(self.QtWidgets.QAbstractItemView.MultiSelection)
        chosen = set(self.favorites())
        for key in self.favorites() + [k for k in self.actions if k not in chosen]:
            item = self.QtWidgets.QListWidgetItem(self.actions[key].text())
            item.setData(self.QtCore.Qt.UserRole, key)
            item.setFlags(item.flags() | self.QtCore.Qt.ItemIsUserCheckable |
                          self.QtCore.Qt.ItemIsDragEnabled)
            item.setCheckState(self.QtCore.Qt.Checked if key in chosen else self.QtCore.Qt.Unchecked)
            available.addItem(item)
        layout.addWidget(available)
        buttons = self.QtWidgets.QDialogButtonBox(
            self.QtWidgets.QDialogButtonBox.Ok | self.QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        if (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))() == self.QtWidgets.QDialog.Accepted:
            keys = [available.item(i).data(self.QtCore.Qt.UserRole)
                    for i in range(available.count())
                    if available.item(i).checkState() == self.QtCore.Qt.Checked]
            self.settings.setValue("favorites", keys)
            self.refresh_favorites()

    def configure_robodk(self):
        dialog = self.QtWidgets.QDialog(self.window)
        dialog.setWindowTitle(tr("action.configure_robodk"))
        form = self.QtWidgets.QFormLayout(dialog)
        entries = {}
        for key, default in (("robot_name", "UR10"), ("frame_name", "Frame 2"),
                             ("tool_name", "Creaform MetraSCAN")):
            entry = self.QtWidgets.QLineEdit(str(self.settings.value("robodk/" + key, default)))
            entries[key] = entry
            form.addRow(tr("dialog.robodk." + key.split("_")[0]), entry)
        buttons = self.QtWidgets.QDialogButtonBox(
            self.QtWidgets.QDialogButtonBox.Ok | self.QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))() == self.QtWidgets.QDialog.Accepted:
            values = {key: entry.text().strip() for key, entry in entries.items()}
            if not all(values.values()):
                raise ValueError(tr("ui.robodk.required"))
            for key, value in values.items():
                self.settings.setValue("robodk/" + key, value)

    def configure_colors(self):
        from ui_theme import apply_application_theme
        keys = ("colorScheme", "accentColor", "surfaceColor", "textColor")
        original = {key: self.settings.value(key, "") for key in keys}
        dialog = self.QtWidgets.QDialog(self.window)
        dialog.setWindowTitle(tr("action.configure_colors"))
        form = self.QtWidgets.QFormLayout(dialog)
        preset = self.QtWidgets.QComboBox(dialog)
        for value, label in (("light", tr("ui.color.light")),
                             ("dark", tr("ui.color.dark")),
                             ("high_contrast", tr("ui.color.high_contrast"))):
            preset.addItem(label, value)
        current = preset.findData(str(original["colorScheme"] or "light"))
        preset.setCurrentIndex(max(current, 0))
        form.addRow(tr("ui.color.preset"), preset)
        colors = {}

        def preview():
            self.settings.setValue("colorScheme", preset.currentData())
            for key, entry in colors.items():
                self.settings.setValue(key, entry.text().strip())
            apply_application_theme(self.QtCore, self.QtGui, self.QtWidgets, self.window)

        preset.currentIndexChanged.connect(preview)
        for key in keys[1:]:
            line = self.QtWidgets.QLineEdit(str(original[key] or ""), dialog)
            line.setPlaceholderText(tr("ui.color.default"))
            line.editingFinished.connect(preview)
            choose = self.QtWidgets.QPushButton("…", dialog)
            choose.clicked.connect(lambda checked=False, field=line:
                self._pick_color(field, preview))
            row = self.QtWidgets.QWidget(dialog)
            inside = self.QtWidgets.QHBoxLayout(row)
            inside.setContentsMargins(0, 0, 0, 0)
            inside.addWidget(line)
            inside.addWidget(choose)
            form.addRow(tr("ui.color." + key), row)
            colors[key] = line
        buttons = self.QtWidgets.QDialogButtonBox(
            self.QtWidgets.QDialogButtonBox.Ok | self.QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        accepted = (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))() == self.QtWidgets.QDialog.Accepted
        if accepted:
            for entry in colors.values():
                value = entry.text().strip()
                if value and not self.QtGui.QColor(value).isValid():
                    for key, previous in original.items():
                        if previous:
                            self.settings.setValue(key, previous)
                        else:
                            self.settings.remove(key)
                    apply_application_theme(self.QtCore, self.QtGui,
                                            self.QtWidgets, self.window)
                    raise ValueError(tr("ui.color.invalid"))
            preview()
        else:
            for key, value in original.items():
                if value:
                    self.settings.setValue(key, value)
                else:
                    self.settings.remove(key)
            apply_application_theme(self.QtCore, self.QtGui, self.QtWidgets, self.window)

    def _pick_color(self, field, preview):
        color = self.QtWidgets.QColorDialog.getColor(self.QtGui.QColor(field.text()),
                                                     self.window)
        if color.isValid():
            field.setText(color.name())
            preview()

    def configure_local_frames(self, redraw):
        original = read_local_frame_style(self.settings)
        dialog = self.QtWidgets.QDialog(self.window)
        dialog.setWindowTitle(tr("action.configure_local_frames"))
        form = self.QtWidgets.QFormLayout(dialog)
        sizes = {}
        for key in ("face_center_size", "center_view_size", "candidate_view_size"):
            spin = self.QtWidgets.QDoubleSpinBox(dialog)
            spin.setRange(0.1, 100000.0)
            spin.setDecimals(1)
            spin.setSuffix(" " + tr("ui.local_frames.model_units"))
            spin.setValue(original[key])
            form.addRow(tr("ui.local_frames." + key), spin)
            sizes[key] = spin
        labels = self.QtWidgets.QCheckBox(tr("ui.local_frames.labels_visible"), dialog)
        labels.setChecked(original["labels_visible"])
        form.addRow(labels)
        height = self.QtWidgets.QDoubleSpinBox(dialog)
        height.setRange(6.0, 48.0)
        height.setDecimals(1)
        height.setValue(original["label_height"])
        form.addRow(tr("ui.local_frames.label_height"), height)
        color = self.QtWidgets.QLineEdit(original["label_color"], dialog)
        choose = self.QtWidgets.QPushButton("…", dialog)
        choose.clicked.connect(lambda: self._pick_color(color, lambda: None))
        row = self.QtWidgets.QWidget(dialog)
        line = self.QtWidgets.QHBoxLayout(row)
        line.setContentsMargins(0, 0, 0, 0)
        line.addWidget(color)
        line.addWidget(choose)
        form.addRow(tr("ui.local_frames.label_color"), row)
        font = self.QtWidgets.QLineEdit(original["label_font"], dialog)
        font.setPlaceholderText(tr("ui.local_frames.default_font"))
        form.addRow(tr("ui.local_frames.label_font"), font)
        buttons = self.QtWidgets.QDialogButtonBox(
            self.QtWidgets.QDialogButtonBox.Ok |
            self.QtWidgets.QDialogButtonBox.Cancel |
            self.QtWidgets.QDialogButtonBox.Apply |
            self.QtWidgets.QDialogButtonBox.RestoreDefaults, dialog)
        form.addRow(buttons)

        def values():
            return normalize_local_frame_style({
                **{key: widget.value() for key, widget in sizes.items()},
                "labels_visible": labels.isChecked(),
                "label_height": height.value(),
                "label_color": color.text().strip(),
                "label_font": font.text().strip(),
            })

        def apply():
            try:
                write_local_frame_style(self.settings, values())
            except (TypeError, ValueError):
                self.QtWidgets.QMessageBox.warning(
                    dialog, tr("action.configure_local_frames"),
                    tr("ui.local_frames.invalid_color"))
                return False
            redraw()
            return True

        def accept():
            if apply():
                dialog.accept()

        def restore_defaults():
            for key, widget in sizes.items():
                widget.setValue(DEFAULT_LOCAL_FRAME_STYLE[key])
            labels.setChecked(DEFAULT_LOCAL_FRAME_STYLE["labels_visible"])
            height.setValue(DEFAULT_LOCAL_FRAME_STYLE["label_height"])
            color.setText(DEFAULT_LOCAL_FRAME_STYLE["label_color"])
            font.clear()

        buttons.accepted.connect(accept)
        buttons.rejected.connect(dialog.reject)
        buttons.button(self.QtWidgets.QDialogButtonBox.Apply).clicked.connect(apply)
        buttons.button(self.QtWidgets.QDialogButtonBox.RestoreDefaults).clicked.connect(
            restore_defaults)
        (getattr(dialog, "exec", None) or getattr(dialog, "exec_"))()
        if dialog.result() != self.QtWidgets.QDialog.Accepted:
            write_local_frame_style(self.settings, original)
            redraw()

    def view(self, command):
        if command == "fit":
            self.display.FitAll()
        elif command in ("front", "back", "left", "right", "top", "bottom", "iso"):
            method = getattr(self.display, "View_" +
                             ("Rear" if command == "back" else command.capitalize()), None)
            if method is None:
                raise RuntimeError("Viewer does not support " + command)
            method()
            self.display.FitAll()
        elif command == "orthographic":
            camera = self.display.View.Camera()
            camera.SetProjectionType(camera.Projection_Orthographic)
            self.display.FitAll()
        elif command == "perspective":
            camera = self.display.View.Camera()
            camera.SetProjectionType(camera.Projection_Perspective)
            self.display.FitAll()
        elif command == "shaded":
            self.display.SetModeShaded()
        elif command == "wireframe":
            self.display.SetModeWireFrame()
        elif command == "edges":
            drawer = self.display.Context.DefaultDrawer()
            drawer.SetFaceBoundaryDraw(not drawer.FaceBoundaryDraw())
            self.display.Context.UpdateCurrentViewer()
        elif command == "screenshot":
            path, _ = self.QtWidgets.QFileDialog.getSaveFileName(
                self.window, tr("action.view_screenshot"), "view.png", "PNG (*.png)")
            if path:
                self.display.View.Dump(path)
        self.display.Repaint()

    def remember_default_layout(self):
        from ui_theme import ensure_primary_ribbon_visible
        geometry = self.settings.value("windowGeometry")
        state = self.settings.value("dockState")
        if geometry:
            self.window.restoreGeometry(geometry)
        if state:
            if not self.window.restoreState(state):
                self.default_layout()
        ensure_primary_ribbon_visible(self.window)
        screens = self.QtWidgets.QApplication.screens()
        if screens and not any(screen.availableGeometry().intersects(self.window.frameGeometry())
                               for screen in screens):
            available = screens[0].availableGeometry()
            self.window.resize(min(self.window.width(), available.width()),
                               min(self.window.height(), available.height()))
            self.window.move(available.topLeft())

    def save_layout(self):
        from ui_theme import ensure_primary_ribbon_visible
        ensure_primary_ribbon_visible(self.window)
        self.settings.setValue("windowGeometry", self.window.saveGeometry())
        self.settings.setValue("dockState", self.window.saveState())
        self.settings.sync()

    def default_layout(self):
        from ui_theme import ensure_primary_ribbon_visible
        if self.window.isFullScreen():
            self.window.showNormal()
        docks = (("WorkflowStatusDock", self.QtCore.Qt.LeftDockWidgetArea, True),
                 ("LayerControlDock", self.QtCore.Qt.RightDockWidgetArea, True),
                 ("OperationHistoryDock", self.QtCore.Qt.BottomDockWidgetArea, False))
        for name, area, visible in docks:
            dock = self.window.findChild(self.QtWidgets.QDockWidget, name)
            if dock is None:
                continue
            if dock.isFloating():
                dock.setFloating(False)
            self.window.addDockWidget(area, dock)
            dock.setVisible(visible)
        left = self.window.findChild(self.QtWidgets.QDockWidget, "WorkflowStatusDock")
        right = self.window.findChild(self.QtWidgets.QDockWidget, "LayerControlDock")
        if left is not None:
            self.window.resizeDocks([left], [240], self.QtCore.Qt.Horizontal)
        if right is not None:
            self.window.resizeDocks([right], [260], self.QtCore.Qt.Horizontal)
        ensure_primary_ribbon_visible(self.window, expand=True)

    def focus_layout(self):
        for name in ("WorkflowStatusDock", "LayerControlDock", "OperationHistoryDock"):
            dock = self.window.findChild(self.QtWidgets.QDockWidget, name)
            if dock is not None:
                dock.hide()

    def toggle_fullscreen(self):
        if self.window.isFullScreen():
            self.window.showNormal()
        else:
            self.window.showFullScreen()

    def save_named_layout(self):
        name, accepted = self.QtWidgets.QInputDialog.getText(
            self.window, tr("action.save_named_layout"), tr("ui.layout.name"))
        if accepted and name.strip():
            self.settings.setValue("layouts/" + name.strip(), self.window.saveState())

    def restore_named_layout(self):
        self.settings.beginGroup("layouts")
        names = self.settings.childKeys()
        self.settings.endGroup()
        if not names:
            return
        name, accepted = self.QtWidgets.QInputDialog.getItem(
            self.window, tr("action.restore_named_layout"), tr("ui.layout.name"),
            names, 0, False)
        if accepted and name:
            from ui_theme import ensure_primary_ribbon_visible
            self.window.restoreState(self.settings.value("layouts/" + name))
            ensure_primary_ribbon_visible(self.window)

    def delete_named_layout(self):
        self.settings.beginGroup("layouts")
        names = self.settings.childKeys()
        self.settings.endGroup()
        if not names:
            return
        name, accepted = self.QtWidgets.QInputDialog.getItem(
            self.window, tr("action.delete_named_layout"), tr("ui.layout.name"),
            names, 0, False)
        if accepted and name:
            self.settings.remove("layouts/" + name)

    def export_config(self):
        path, _ = self.QtWidgets.QFileDialog.getSaveFileName(
            self.window, tr("action.export_ui_config"), "ui-preferences.json", "JSON (*.json)")
        if path:
            data = {"version": CONFIG_VERSION,
                    "ribbonSize": str(self.settings.value("ribbonSize", "standard")),
                    "favorites": self.favorites(),
                    "colorScheme": str(self.settings.value("colorScheme", "light")),
                    "colors": {key: str(self.settings.value(key, ""))
                               for key in ("accentColor", "surfaceColor", "textColor")},
                    "localFrames": read_local_frame_style(self.settings),
                    "shortcuts": {key: action.shortcut().toString()
                                  for key, action in self.actions.items()},
                    "robodk": {key: str(self.settings.value("robodk/" + key, default))
                               for key, default in (("robot_name", "UR10"),
                                                    ("frame_name", "Frame 2"),
                                                    ("tool_name", "Creaform MetraSCAN"))}}
            target = Path(path)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile("w", encoding="utf-8", delete=False,
                                                 dir=str(target.parent), suffix=".tmp") as stream:
                    temporary = stream.name
                    json.dump(data, stream, ensure_ascii=False, indent=2)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, str(target))
            finally:
                if temporary and os.path.exists(temporary):
                    os.unlink(temporary)

    def import_config(self):
        path, _ = self.QtWidgets.QFileDialog.getOpenFileName(
            self.window, tr("action.import_ui_config"), "", "JSON (*.json)")
        if not path:
            return
        source = Path(path)
        if source.stat().st_size > 1024 * 1024:
            raise ValueError("UI configuration is too large")
        data = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("version") != CONFIG_VERSION:
            raise ValueError("Unsupported UI configuration version")
        size = data.get("ribbonSize", "standard")
        keys = data.get("favorites", [])
        scheme = data.get("colorScheme", "light")
        colors = data.get("colors", {})
        shortcuts = data.get("shortcuts", {})
        robodk = data.get("robodk", {})
        local_frames = normalize_local_frame_style(data.get("localFrames", {}))
        if (size not in ("compact", "standard", "large") or
                scheme not in ("light", "dark", "high_contrast") or
                not isinstance(keys, list) or not isinstance(colors, dict) or
                not isinstance(shortcuts, dict) or not isinstance(robodk, dict)):
            raise ValueError("Invalid UI configuration")
        if not all(isinstance(key, str) for key in keys):
            raise ValueError("Invalid favorite command")
        for key in ("accentColor", "surfaceColor", "textColor"):
            value = colors.get(key, "")
            if not isinstance(value, str) or (value and not self.QtGui.QColor(value).isValid()):
                raise ValueError("Invalid UI color")
        if not all(isinstance(key, str) and isinstance(value, str)
                   for key, value in shortcuts.items()):
            raise ValueError("Invalid shortcuts")
        known_shortcuts = {key: self.QtGui.QKeySequence(value).toString()
                           for key, value in shortcuts.items() if key in self.actions}
        if any(shortcuts[key] and not sequence
               for key, sequence in known_shortcuts.items()):
            raise ValueError("Invalid keyboard shortcut")
        effective_shortcuts = {key: action.shortcut().toString()
                               for key, action in self.actions.items()}
        effective_shortcuts.update(known_shortcuts)
        values = [value for value in effective_shortcuts.values() if value]
        if len(values) != len(set(values)) or "Ctrl+Shift+0" in values:
            raise ValueError(tr("ui.shortcut.conflict"))
        if not all(isinstance(robodk.get(key, default), str) and
                   robodk.get(key, default).strip()
                   for key, default in (("robot_name", "UR10"),
                                        ("frame_name", "Frame 2"),
                                        ("tool_name", "Creaform MetraSCAN"))):
            raise ValueError("Invalid RoboDK defaults")
        preview = tr("ui.config.preview", size=size, scheme=scheme,
                     favorites=len([key for key in keys if key in self.actions]),
                     shortcuts=len(known_shortcuts))
        if self.QtWidgets.QMessageBox.question(
                self.window, tr("action.import_ui_config"), preview,
                self.QtWidgets.QMessageBox.Ok | self.QtWidgets.QMessageBox.Cancel
                ) != self.QtWidgets.QMessageBox.Ok:
            return
        from ui_theme import set_ribbon_size
        from ui_theme import apply_application_theme
        changes = {"favorites": [key for key in keys if key in self.actions],
                   "colorScheme": scheme}
        changes.update({key: colors.get(key, "")
                        for key in ("accentColor", "surfaceColor", "textColor")})
        changes.update({"shortcuts/" + key: value
                        for key, value in known_shortcuts.items()})
        changes.update({"robodk/" + key: robodk[key]
                        for key in ("robot_name", "frame_name", "tool_name")
                        if key in robodk})
        changes.update({"localFrames/" + key: value
                        for key, value in local_frames.items()})
        old = {key: self.settings.value(key) for key in changes}
        old_shortcuts = {key: action.shortcut() for key, action in self.actions.items()}
        old_size = str(self.settings.value("ribbonSize", "standard"))
        try:
            for key, value in changes.items():
                self.settings.setValue(key, value)
            set_ribbon_size(size)
            for key, sequence in known_shortcuts.items():
                self.actions[key].setShortcut(self.QtGui.QKeySequence(sequence))
            apply_application_theme(self.QtCore, self.QtGui, self.QtWidgets, self.window)
            self.refresh_favorites()
            if self.redraw_scene:
                self.redraw_scene()
            self.settings.sync()
        except Exception:
            for key, value in old.items():
                if value is None:
                    self.settings.remove(key)
                else:
                    self.settings.setValue(key, value)
            set_ribbon_size(old_size if old_size in ("compact", "standard", "large")
                            else "standard")
            for key, sequence in old_shortcuts.items():
                self.actions[key].setShortcut(sequence)
            apply_application_theme(self.QtCore, self.QtGui, self.QtWidgets, self.window)
            self.refresh_favorites()
            raise

    def reset_config(self):
        for key in ("favorites", "ribbonSize", "windowGeometry", "dockState",
                    "colorScheme", "accentColor", "surfaceColor", "textColor",
                    "shortcuts", "robodk", "layouts", "localFrames"):
            self.settings.remove(key)
        from ui_theme import set_ribbon_size, apply_application_theme
        set_ribbon_size("standard")
        for key, sequence in self.default_shortcuts.items():
            self.actions[key].setShortcut(self.QtGui.QKeySequence(sequence))
        apply_application_theme(self.QtCore, self.QtGui, self.QtWidgets, self.window)
        self.default_layout()
        self.refresh_favorites()
        if self.redraw_scene:
            self.redraw_scene()
