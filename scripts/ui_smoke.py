"""Visual smoke capture for the integrated command UI with a real CAD model."""
import os
import sys
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
    main.run()
    app = main.QtWidgets.QApplication.instance()
    window = main.get_main_window()
    window.show()
    app.processEvents()
    shape = main.load_cad_shape(str(model))
    main.state.current_shape = shape
    main.state.workpiece_coordinate_system_size = main.calculate_workpiece_coordinate_size(shape)
    main._do_render()
    main._update_workflow()
    ribbon = window.findChild(main.QtWidgets.QTabWidget, "CommandRibbon")
    assert ribbon is not None and ribbon.count() == 5
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
    window.resize(700, 600)
    ribbon.setCurrentIndex(4)
    app.processEvents()
    assert ribbon.widget(4).horizontalScrollBar().maximum() > 0
    ribbon.cornerWidget().click()
    app.processEvents()
    assert ribbon.property("collapsed") == True
    ribbon.cornerWidget().click()
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
        if size == "large":
            window.showMaximized()
        else:
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
    assert ribbon.tabText(0) == "模型"
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
    assert ribbon.tabText(0) == "Model"
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
    main._awaiting_face_selection = False
    selected = output / "viewport_selected.png"
    main.display.View.Dump(str(selected))
    print(selected)
    main.state.selected_face = None
    main._do_render(fit_all=False)
    main.display.View.Dump(str(output / "viewport_unselected.png"))
    main.select_single_face()
    main.select_face_clicked([faces.Current()], 0, 0)
    main.get_user_segment_params = lambda: (2, 2)
    main.show_topmost_message = lambda title, message, type="info": "yes" if type == "question" else None
    main.run_background_task = (
        lambda title, message, fn, on_success, on_error=None: on_success(fn()))
    main.segment_faces()
    assert main.state.current_faces and main.state.selected_face is None
    main.display.View.Dump(str(output / "viewport_segmented.png"))
    main.clear_model()
    assert main.state.current_shape is None and main.state.selected_face is None
    main.display.View.Dump(str(output / "viewport_cleared.png"))
    main._app_ready = False
    window.close()


if __name__ == "__main__":
    main_smoke()
