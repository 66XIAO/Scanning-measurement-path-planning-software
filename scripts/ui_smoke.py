"""Visual smoke capture for the integrated command UI with a real CAD model."""
import os
import sys
import json
from unittest import mock
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

if len(sys.argv) > 3:
    os.environ["QT_SCALE_FACTOR"] = sys.argv[3]

import main
from ui_theme import set_ribbon_size


def main_smoke():
    model = Path(sys.argv[1])
    output = Path(sys.argv[2])
    output.mkdir(parents=True, exist_ok=True)
    main.QtCore.QSettings.setDefaultFormat(main.QtCore.QSettings.IniFormat)
    main.QtCore.QSettings.setPath(
        main.QtCore.QSettings.IniFormat,
        main.QtCore.QSettings.UserScope, str(output.resolve()))
    main.start_display = lambda: None
    errors = []
    def show_message(title, message, type="info"):
        if type == "error":
            errors.append(str(message))
        return "yes" if type == "question" else None
    main.show_topmost_message = show_message
    # The smoke exercises scene transitions without interactive save prompts.
    main._confirm_discard_or_save = lambda: True
    main.run()
    app = main.QtWidgets.QApplication.instance()
    window = main.get_main_window()
    window.show()
    app.processEvents()
    primary_toolbar = window.findChild(main.QtWidgets.QToolBar, "PrimaryWorkflowToolbar")
    assert primary_toolbar.isVisible()
    main._workspace_ui.focus_layout()
    app.processEvents()
    main._workspace_ui.default_layout()
    app.processEvents()
    assert primary_toolbar.isVisible(), "Default layout hid the main Ribbon"
    assert window.findChild(main.QtWidgets.QDockWidget, "WorkflowStatusDock").isVisible()
    assert window.findChild(main.QtWidgets.QDockWidget, "LayerControlDock").isVisible()
    # Simulate a legacy UI.ini state whose serialized toolbar visibility is off.
    primary_toolbar.hide()
    hidden_layout = window.saveState()
    primary_toolbar.show()
    main._workspace_ui.settings.setValue("dockState", hidden_layout)
    main._workspace_ui.remember_default_layout()
    app.processEvents()
    assert primary_toolbar.isVisible(), "Restoring a saved layout hid the main Ribbon"
    # Closing with a hidden toolbar must not persist the unusable state.
    primary_toolbar.hide()
    main._workspace_ui.save_layout()
    assert primary_toolbar.isVisible()
    window.restoreState(main._workspace_ui.settings.value("dockState"))
    app.processEvents()
    assert primary_toolbar.isVisible()
    # Named layouts created by older versions need the same recovery guard.
    main._workspace_ui.settings.setValue("layouts/legacy-hidden", hidden_layout)
    with mock.patch.object(main.QtWidgets.QInputDialog, "getItem",
                           return_value=("legacy-hidden", True)):
        main._workspace_ui.restore_named_layout()
    app.processEvents()
    assert primary_toolbar.isVisible()
    main._workspace_ui.default_layout()
    primary_toolbar.hide()
    main._workspace_ui.focus_layout()
    from PyQt5.QtTest import QTest
    QTest.keyClick(window, main.QtCore.Qt.Key_0,
                   main.QtCore.Qt.ControlModifier | main.QtCore.Qt.ShiftModifier)
    app.processEvents()
    assert primary_toolbar.isVisible(), "Recovery shortcut did not restore the Ribbon"
    assert window.findChild(main.QtWidgets.QDockWidget, "WorkflowStatusDock").isVisible()
    ribbon = window.findChild(main.QtWidgets.QTabWidget, "CommandRibbon")
    collapse = ribbon.findChild(main.QtWidgets.QToolButton, "RibbonCollapse")
    collapse.click()
    assert ribbon.property("collapsed") is True
    main._workspace_ui.default_layout()
    app.processEvents()
    assert ribbon.property("collapsed") is False
    workflow = window.findChild(main.QtWidgets.QDockWidget, "WorkflowStatusDock")
    workflow.setFloating(True)
    main._workspace_ui.default_layout()
    app.processEvents()
    assert not workflow.isFloating()
    assert window.dockWidgetArea(workflow) == main.QtCore.Qt.LeftDockWidgetArea
    main._workspace_ui.settings.setValue("dockState", b"invalid-dock-state")
    main._workspace_ui.remember_default_layout()
    app.processEvents()
    assert primary_toolbar.isVisible() and workflow.isVisible()
    assert window.findChild(main.QtWidgets.QDockWidget, "LayerControlDock").isVisible()
    main.import_model_from_path(str(model))
    assert main.state.current_shape is not None
    shape = main.state.current_shape
    assert ribbon is not None and ribbon.count() == 7
    assert window.menuBar().isHidden()
    assert len(window.findChildren(main.QtWidgets.QToolButton)) >= 30
    exposed = set()
    for button in ribbon.findChildren(main.QtWidgets.QToolButton):
        if button.defaultAction() is not None:
            exposed.add(button.defaultAction())
        if button.menu() is not None:
            exposed.update(button.menu().actions())
    menu_keys = {str(action.property("i18n_key")) for action in main._translated_actions}
    exposed_keys = {str(action.property("i18n_key")) for action in exposed}
    assert menu_keys <= exposed_keys
    assert {
        "action.new_workstation", "action.open_workstation",
        "action.save_workstation", "action.save_workstation_as",
        "action.create_operation_panel",
    } <= exposed_keys
    assert [ribbon.tabText(i) for i in range(7)] == [
        "File", "Path planning", "Collision detection",
        "Calibration and export", "View", "Settings", "Help"]
    for size_name in ("compact", "standard", "large"):
        set_ribbon_size(size_name)
        for language, switch in (("en", main.switch_to_english),
                                 ("zh", main.switch_to_chinese)):
            switch()
            window.resize(1366, 768)
            for index in range(7):
                ribbon.setCurrentIndex(index)
                app.processEvents()
                for button in ribbon.widget(index).findChildren(main.QtWidgets.QToolButton):
                    if button.defaultAction() is not None:
                        assert button.width() >= button.fontMetrics().horizontalAdvance(
                            button.text()) + 20, button.text()
                window.grab().save(str(output / ("tab_{}_{}_{}.png".format(
                    size_name, language, index))))
    main.switch_to_english()
    for scheme in ("light", "dark", "high_contrast"):
        main._workspace_ui.settings.setValue("colorScheme", scheme)
        main.apply_application_theme(main.QtCore, main.QtGui, main.QtWidgets, window)
        app.processEvents()
        window.grab().save(str(output / ("scheme_" + scheme + ".png")))
    main._workspace_ui.settings.setValue("colorScheme", "light")
    main.apply_application_theme(main.QtCore, main.QtGui, main.QtWidgets, window)
    workflow = window.findChild(main.QtWidgets.QDockWidget, "WorkflowStatusDock")
    workflow.hide()
    app.processEvents()
    assert not workflow.isVisible()
    main._workspace_ui.actions["action.create_workflow_panel"].trigger()
    app.processEvents()
    assert workflow.isVisible()
    preferences_file = output / "preferences.json"
    with mock.patch.object(main.QtWidgets.QFileDialog, "getSaveFileName",
                           return_value=(str(preferences_file), "JSON (*.json)")):
        main._workspace_ui.export_config()
    exported = json.loads(preferences_file.read_text(encoding="utf-8"))
    assert exported["version"] == 1
    assert "recentWorkstations" not in exported and "model_source_path" not in exported
    main._workspace_ui.settings.setValue("favorites", [])
    with mock.patch.object(main.QtWidgets.QFileDialog, "getOpenFileName",
                           return_value=(str(preferences_file), "JSON (*.json)")), \
         mock.patch.object(main.QtWidgets.QMessageBox, "question",
                           return_value=main.QtWidgets.QMessageBox.Ok):
        main._workspace_ui.import_config()
    assert main._workspace_ui.favorites() == exported["favorites"]
    assert main.state.current_shape is shape
    invalid = output / "invalid-preferences.json"
    invalid.write_text(json.dumps({"version": 99, "favorites": []}), encoding="utf-8")
    with mock.patch.object(main.QtWidgets.QFileDialog, "getOpenFileName",
                           return_value=(str(invalid), "JSON (*.json)")):
        try:
            main._workspace_ui.import_config()
        except ValueError:
            pass
        else:
            raise AssertionError("Unsupported UI config version was accepted")
    assert main._workspace_ui.favorites() == exported["favorites"]
    for command in ("fit", "iso", "front", "back", "left", "right",
                    "top", "bottom", "orthographic", "perspective",
                    "wireframe", "shaded", "edges"):
        main._workspace_ui.view(command)
        main.display.View.Dump(str(output / ("view_command_" + command + ".png")))
    main._workspace_ui.view("iso")
    search = window.findChild(main.QtWidgets.QLineEdit, "CommandSearch")
    assert search is not None
    search.setText("segment")
    assert "segment" in search.text()
    search.clear()
    window.resize(700, 600)
    ribbon.setCurrentIndex(4)
    app.processEvents()
    assert ribbon.widget(4).horizontalScrollBar().maximum() > 0
    collapse = ribbon.findChild(main.QtWidgets.QToolButton, "RibbonCollapse")
    collapse.click()
    app.processEvents()
    assert ribbon.property("collapsed") == True
    collapse.click()
    app.processEvents()
    assert ribbon.property("collapsed") == False
    window.findChild(main.QtWidgets.QToolButton, "WorkflowCard_model").click()
    assert ribbon.currentIndex() == 0
    for size, width, height in (("standard", 1366, 768), ("large", 1920, 1080)):
        set_ribbon_size(size)
        stored_size = str(main.QtCore.QSettings(
            main.QtCore.QSettings.IniFormat, main.QtCore.QSettings.UserScope,
            "ScanningPathPlanner", "UI").value("ribbonSize"))
        assert stored_size == size
        window.showNormal()
        window.resize(width, height)
        app.processEvents()
        main.display.FitAll()
        main.display.Repaint()
        app.processEvents()
        viewport = output / ("viewport_" + size + ".png")
        main.display.View.Dump(str(viewport))
        image = output / ("ui_" + size + ".png")
        capture = window.grab()
        capture.save(str(output / ("ui_raw_" + size + ".png")))
        canvas = getattr(window, "canva", None)
        if canvas is not None:
            painter = main.QtGui.QPainter(capture)
            origin = canvas.mapTo(window, main.QtCore.QPoint(0, 0))
            painter.drawPixmap(main.QtCore.QRect(origin, canvas.size()),
                               main.QtGui.QPixmap(str(viewport)))
            painter.end()
        if not capture.save(str(image)):
            raise RuntimeError("Could not save " + str(image))
        print(image)
    ribbon.setCurrentIndex(4)
    app.processEvents()
    window.grab().save(str(output / "ui_view_controls.png"))
    ribbon.setCurrentIndex(0)
    main.switch_to_chinese()
    assert ribbon.tabText(0) == "文件"
    assert ribbon.tabText(6) == "帮助"
    app.processEvents()
    chinese_capture = window.grab()
    canvas = getattr(window, "canva", None)
    if canvas is not None:
        painter = main.QtGui.QPainter(chinese_capture)
        origin = canvas.mapTo(window, main.QtCore.QPoint(0, 0))
        painter.drawPixmap(main.QtCore.QRect(origin, canvas.size()),
                           main.QtGui.QPixmap(str(output / "viewport_large.png")))
        painter.end()
    chinese_capture.save(str(output / "ui_chinese.png"))
    main.switch_to_english()
    assert ribbon.tabText(0) == "File"
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopAbs import TopAbs_FACE
    faces = TopExp_Explorer(shape, TopAbs_FACE)
    assert faces.More()
    main.select_single_face()
    app.processEvents()
    center = canvas.rect().center()
    main.display.Context.MoveTo(center.x(), center.y(), main.display.View, True)
    main.display.Select(center.x(), center.y())
    app.processEvents()
    assert main.state.selected_face is not None
    main.select_single_face()
    assert len(main.display._select_callbacks) == 1
    main.display.Context.MoveTo(center.x(), center.y(), main.display.View, True)
    main.display.Select(center.x(), center.y())
    app.processEvents()
    assert main.state.selected_face is not None
    assert not main._awaiting_face_selection
    assert main.state.selected_face_source_index >= 0
    native_selected_face = main.state.selected_face
    selected = output / "viewport_selected.png"
    main.display.View.Dump(str(selected))
    print(selected)
    main.state.selected_face = None
    main._do_render(fit_all=False)
    main.display.View.Dump(str(output / "viewport_unselected.png"))
    main.select_single_face()
    main.select_face_clicked([native_selected_face], 0, 0)
    u = int(sys.argv[4]) if len(sys.argv) > 4 else 2
    v = int(sys.argv[5]) if len(sys.argv) > 5 else 2
    main.get_user_segment_params = lambda **kwargs: (u, v)
    main.run_background_task = (
        lambda title, message, fn, on_success, on_error=None: on_success(fn()))
    main.segment_faces()
    assert main.state.current_faces and main.state.selected_face is None
    print("Smoke segmentation: {} patches".format(len(main.state.current_faces)))
    main.display.View.Dump(str(output / "viewport_segmented.png"))
    main.get_centers()
    main.generate_center_viewpoints()
    from local_frame_style import write_local_frame_style
    write_local_frame_style(main._workspace_ui.settings, {
        "face_center_size": 14.0,
        "center_view_size": 9.0,
        "candidate_view_size": 4.0,
        "labels_visible": False,
        "label_height": 14.0,
        "label_color": "#40C0FF",
        "label_font": "Arial",
    })
    main._do_render(fit_all=False)
    frames = main.state.coordinate_systems
    assert len(frames) == 2 * len(main.state.current_faces), len(frames)
    assert all(frame.Size() == 14.0 for frame in frames[:len(main.state.current_faces)])
    assert all(frame.Size() == 9.0 for frame in frames[len(main.state.current_faces):])
    assert all(not frame.Attributes().DatumAspect().ToDrawLabels() for frame in frames)
    assert all(frame.Attributes().DatumAspect().TextAspect().Height() == 14.0
               for frame in frames)
    main.display.View.Dump(str(output / "viewport_local_frames_no_labels.png"))
    main.generate_candidate_viewpoints()
    candidate_count = max(0, len(main.state.view_points) - len(main.state.face_centers))
    assert candidate_count > 0
    assert len(main.state.coordinate_systems) == 2 * len(main.state.face_centers) + candidate_count
    assert all(frame.Size() == 4.0 for frame in main.state.coordinate_systems[-candidate_count:])
    def edit_local_frame_dialog():
        dialog = app.activeModalWidget()
        assert dialog is not None
        spins = dialog.findChildren(main.QtWidgets.QDoubleSpinBox)
        spins[0].setValue(12.0)
        dialog.findChild(main.QtWidgets.QCheckBox).setChecked(True)
        box = dialog.findChild(main.QtWidgets.QDialogButtonBox)
        box.button(main.QtWidgets.QDialogButtonBox.Ok).click()
    main.QtCore.QTimer.singleShot(0, edit_local_frame_dialog)
    main._workspace_ui.configure_local_frames(lambda: main._do_render(fit_all=False))
    assert main.state.coordinate_systems[0].Size() == 12.0
    assert main.state.coordinate_systems[0].Attributes().DatumAspect().ToDrawLabels()
    main.filter_optimal_viewpoints()
    assert len(main.state.face_centers) == len(main.state.current_faces)
    assert len(main.state.optimal_viewpoints) == len(main.state.current_faces)
    archive = output / "smoke.swstation"
    assert main._save_workstation_to(str(archive))
    assert not main.state.workstation_dirty
    patch_count = len(main.state.current_faces)
    # Clearing an armed selection must also cancel its pending callback state.
    main.select_single_face()
    assert main._awaiting_face_selection
    assert main.new_workstation()
    assert main.state.current_shape is None and not main._awaiting_face_selection
    assert main.open_dropped_file(str(archive))
    assert len(main.state.current_faces) == patch_count
    assert len(main.state.optimal_viewpoints) == patch_count
    assert not main._awaiting_face_selection
    assert main.state.model_source_path == str(model.resolve())
    assert len(main.display._select_callbacks) == 1
    main.create_operation_panel_ui()
    app.processEvents()
    from ui_panels import _clear_operation_display
    history_count = len(main.state.operation_history)
    _clear_operation_display()
    assert len(main.state.operation_history) == history_count
    main.update_operation_panel(main.state)
    window.grab().save(str(output / "ui_workstation.png"))
    main.clear_model()
    assert main.state.current_shape is None and main.state.selected_face is None
    main.display.View.Dump(str(output / "viewport_cleared.png"))
    assert not errors, errors
    print("Smoke passed: layout recovery, local frame styles, native selection, segmentation, viewpoints, workstation round trip and clear")
    main._app_ready = False
    window.close()


if __name__ == "__main__":
    main_smoke()
