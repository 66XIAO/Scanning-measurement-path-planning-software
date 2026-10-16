"""Regression for WorkspaceUI initialization on both supported Qt layouts."""
import types
import unittest
from unittest import mock

from ui_workspace import WorkspaceUI


class FakeSettings:
    IniFormat = 1
    UserScope = 0

    def __init__(self, *args):
        pass


class WorkspaceQtCompatibilityTests(unittest.TestCase):
    def _initialize(self, qt6):
        shortcut = mock.Mock()
        factory = mock.Mock(return_value=shortcut)
        core = types.SimpleNamespace(
            QSettings=FakeSettings, Qt=types.SimpleNamespace(ApplicationShortcut=7))
        gui = types.SimpleNamespace(QKeySequence=lambda text: ('sequence', text))
        widgets = types.SimpleNamespace()
        if qt6:
            gui.QShortcut = factory
        else:
            widgets.QShortcut = factory
        window = object()
        workspace = WorkspaceUI(core, gui, widgets, window, object(), {})
        factory.assert_called_once_with(('sequence', 'Ctrl+Shift+0'), window)
        shortcut.setContext.assert_called_once_with(core.Qt.ApplicationShortcut)
        shortcut.activated.connect.assert_called_once_with(workspace.default_layout)
        self.assertIs(workspace.recovery_shortcut, shortcut)

    def test_qt6_initializes_without_qtwidgets_shortcut(self):
        self._initialize(qt6=True)

    def test_qt5_keeps_qtwidgets_shortcut_support(self):
        self._initialize(qt6=False)


if __name__ == '__main__':
    unittest.main()
