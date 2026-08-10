"""Qt dialogs used by the integrated speed-planning workflow."""

from i18n import tr


def _qt():
    from OCC.Display.backend import get_qt_modules
    QtCore, QtGui, QtWidgets, QtOpenGL = get_qt_modules()
    return QtCore, QtWidgets


def _exec(dialog):
    method = getattr(dialog, "exec", None) or getattr(dialog, "exec_")
    return method()


def _double_spin(QtWidgets, minimum, maximum, value, decimals=2, suffix=""):
    box = QtWidgets.QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setValue(value)
    box.setDecimals(decimals)
    if suffix:
        box.setSuffix(" " + suffix)
    return box


def _translate_dialog_buttons(QtWidgets, buttons):
    for standard_button, key in (
            (QtWidgets.QDialogButtonBox.Ok, "common.ok"),
            (QtWidgets.QDialogButtonBox.Cancel, "common.cancel")):
        button = buttons.button(standard_button)
        if button is not None:
            button.setText(tr(key))


def get_speed_planning_settings(parent=None):
    QtCore, QtWidgets = _qt()
    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle(tr("dialog.speed.title"))
    dialog.setMinimumWidth(500)
    layout = QtWidgets.QVBoxLayout(dialog)

    explanation = QtWidgets.QLabel(tr("dialog.speed.explanation"))
    explanation.setWordWrap(True)
    layout.addWidget(explanation)

    form = QtWidgets.QFormLayout()
    algorithm = QtWidgets.QComboBox()
    algorithm.addItem(tr("dialog.speed.algorithm_deterministic"), "deterministic")
    algorithm.addItem(tr("dialog.speed.algorithm_double_q"), "double_q")
    form.addRow(tr("dialog.speed.algorithm"), algorithm)

    widgets = {
        "min_linear_speed": _double_spin(QtWidgets, 0.1, 5000, 20, suffix="mm/s"),
        "max_linear_speed": _double_spin(QtWidgets, 0.1, 5000, 250, suffix="mm/s"),
        "max_linear_accel": _double_spin(QtWidgets, 0.1, 50000, 500, suffix="mm/s^2"),
        "max_lateral_accel": _double_spin(QtWidgets, 0.1, 50000, 300, suffix="mm/s^2"),
        "max_angular_speed": _double_spin(QtWidgets, 0.1, 1000, 60, suffix="deg/s"),
        "max_angular_accel": _double_spin(QtWidgets, 0.1, 10000, 180, suffix="deg/s^2"),
        "command_joint_speed": _double_spin(QtWidgets, 0.1, 1000, 60, suffix="deg/s"),
        "command_joint_accel": _double_spin(QtWidgets, 0.1, 10000, 180, suffix="deg/s^2"),
        "start_speed": _double_spin(QtWidgets, 0.1, 5000, 20, suffix="mm/s"),
        "end_speed": _double_spin(QtWidgets, 0.1, 5000, 20, suffix="mm/s"),
        "safety_factor": _double_spin(QtWidgets, 0.1, 1.0, 0.90, decimals=3),
        "accel_margin_factor": _double_spin(QtWidgets, 1.0, 2.0, 1.10, decimals=3),
    }
    for key, widget in widgets.items():
        form.addRow(tr("dialog.speed." + key), widget)

    speed_levels = QtWidgets.QSpinBox()
    speed_levels.setRange(3, 100)
    speed_levels.setValue(15)
    episodes = QtWidgets.QSpinBox()
    episodes.setRange(10, 1000000)
    episodes.setValue(5000)
    seed = QtWidgets.QSpinBox()
    seed.setRange(0, 2147483647)
    seed.setValue(7)
    form.addRow(tr("dialog.speed.speed_levels"), speed_levels)
    form.addRow(tr("dialog.speed.episodes"), episodes)
    form.addRow(tr("dialog.speed.seed"), seed)
    layout.addLayout(form)

    warning = QtWidgets.QLabel(tr("dialog.speed.warning"))
    warning.setWordWrap(True)
    layout.addWidget(warning)

    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
    _translate_dialog_buttons(QtWidgets, buttons)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout.addWidget(buttons)
    if _exec(dialog) != QtWidgets.QDialog.Accepted:
        return None
    result = {key: widget.value() for key, widget in widgets.items()}
    result.update({
        "algorithm": algorithm.currentData(),
        "speed_levels": speed_levels.value(),
        "training_episodes": episodes.value(),
        "random_seed": seed.value(),
    })
    return result


def get_robodk_import_settings(parent=None, import_kind="speed"):
    if import_kind not in ("speed", "path"):
        raise ValueError("import_kind must be 'speed' or 'path'")
    QtCore, QtWidgets = _qt()
    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle(tr("dialog.robodk.{}_title".format(import_kind)))
    dialog.setMinimumWidth(440)
    layout = QtWidgets.QFormLayout(dialog)
    robot = QtWidgets.QLineEdit("UR10")
    frame = QtWidgets.QLineEdit("Frame 2")
    tool = QtWidgets.QLineEdit("Creaform MetraSCAN")
    program = QtWidgets.QLineEdit(
        "IntegratedSpeedPlan" if import_kind == "speed" else "IntegratedPlannedPath")
    namespace = QtWidgets.QLineEdit("")
    first_move = QtWidgets.QComboBox()
    first_move.addItem(tr("dialog.robodk.movej_then_movel"), "movej")
    first_move.addItem(tr("dialog.robodk.all_movel"), "movel")
    replace = QtWidgets.QCheckBox(tr("dialog.robodk.replace_items"))
    layout.addRow(tr("dialog.robodk.robot"), robot)
    layout.addRow(tr("dialog.robodk.frame"), frame)
    layout.addRow(tr("dialog.robodk.tool"), tool)
    layout.addRow(tr("dialog.robodk.program"), program)
    layout.addRow(tr("dialog.robodk.namespace"), namespace)
    layout.addRow(tr("dialog.robodk.first_move"), first_move)
    layout.addRow(tr("dialog.robodk.replace"), replace)
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
    _translate_dialog_buttons(QtWidgets, buttons)
    buttons.accepted.connect(dialog.accept)
    buttons.rejected.connect(dialog.reject)
    layout.addRow(buttons)
    if _exec(dialog) != QtWidgets.QDialog.Accepted:
        return None
    return {
        "robot_name": robot.text().strip(),
        "frame_name": frame.text().strip(),
        "tool_name": tool.text().strip(),
        "program_name": program.text().strip(),
        "target_namespace": namespace.text().strip() or None,
        "first_move": first_move.currentData(),
        "replace": replace.isChecked(),
    }
