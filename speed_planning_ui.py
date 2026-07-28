"""Qt dialogs used by the integrated speed-planning workflow."""


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


def get_speed_planning_settings(parent=None):
    QtCore, QtWidgets = _qt()
    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle("Constrained Speed Planning")
    dialog.setMinimumWidth(500)
    layout = QtWidgets.QVBoxLayout(dialog)

    explanation = QtWidgets.QLabel(
        "Plan speed after path ordering. Deterministic mode is the production baseline; "
        "Double-Q is the thesis-compatible research mode. Units: mm, s, deg.")
    explanation.setWordWrap(True)
    layout.addWidget(explanation)

    form = QtWidgets.QFormLayout()
    algorithm = QtWidgets.QComboBox()
    algorithm.addItem("Deterministic forward/backward (recommended)", "deterministic")
    algorithm.addItem("Double Q-learning (research)", "double_q")
    form.addRow("Algorithm", algorithm)

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
    labels = {
        "min_linear_speed": "Minimum scanner speed",
        "max_linear_speed": "Maximum scanner speed",
        "max_linear_accel": "Maximum linear acceleration",
        "max_lateral_accel": "Maximum lateral acceleration",
        "max_angular_speed": "Maximum TCP orientation rate",
        "max_angular_accel": "Maximum TCP orientation acceleration",
        "command_joint_speed": "RoboDK joint speed command limit",
        "command_joint_accel": "RoboDK joint acceleration command limit",
        "start_speed": "Start speed",
        "end_speed": "End speed",
        "safety_factor": "Safety factor",
        "accel_margin_factor": "Exported acceleration margin",
    }
    for key, widget in widgets.items():
        form.addRow(labels[key], widget)

    speed_levels = QtWidgets.QSpinBox()
    speed_levels.setRange(3, 100)
    speed_levels.setValue(15)
    episodes = QtWidgets.QSpinBox()
    episodes.setRange(10, 1000000)
    episodes.setValue(5000)
    seed = QtWidgets.QSpinBox()
    seed.setRange(0, 2147483647)
    seed.setValue(7)
    form.addRow("Discrete speed levels", speed_levels)
    form.addRow("Double-Q episodes", episodes)
    form.addRow("Random seed", seed)
    layout.addLayout(form)

    warning = QtWidgets.QLabel(
        "Important: this stage enforces geometric/TCP constraints. Joint velocity, joint "
        "acceleration and torque require robot-specific validation before production execution.")
    warning.setWordWrap(True)
    layout.addWidget(warning)

    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
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


def get_robodk_import_settings(parent=None):
    QtCore, QtWidgets = _qt()
    dialog = QtWidgets.QDialog(parent)
    dialog.setWindowTitle("Import Speed Plan to RoboDK")
    dialog.setMinimumWidth(440)
    layout = QtWidgets.QFormLayout(dialog)
    robot = QtWidgets.QLineEdit("UR10")
    frame = QtWidgets.QLineEdit("Frame 2")
    tool = QtWidgets.QLineEdit("Creaform MetraSCAN")
    program = QtWidgets.QLineEdit("IntegratedSpeedPlan")
    namespace = QtWidgets.QLineEdit("")
    first_move = QtWidgets.QComboBox()
    first_move.addItem("MoveJ to first point, then MoveL", "movej")
    first_move.addItem("MoveL for every point", "movel")
    replace = QtWidgets.QCheckBox("Delete and rebuild same-name generated items")
    layout.addRow("Robot", robot)
    layout.addRow("Reference frame", frame)
    layout.addRow("Tool", tool)
    layout.addRow("Program name", program)
    layout.addRow("Target namespace (optional)", namespace)
    layout.addRow("First movement", first_move)
    layout.addRow("Replace", replace)
    buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
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
