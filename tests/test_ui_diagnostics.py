import ast
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


class UIDiagnosticsCompatibilityTests(unittest.TestCase):
    def test_diagnostics_uses_runtime_qt_version_without_pyqt_only_constant(self):
        # Extract the real handler without opening the viewer at import time.
        path = Path(__file__).resolve().parents[1] / 'main.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        function = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == '_show_diagnostics')
        module = ast.Module(body=[function], type_ignores=[])
        namespace = {'sys': sys, 'PYTHONOCC_VERSION': 'test-occ',
                     'QtCore': types.SimpleNamespace(qVersion=lambda: '6.7.3'),
                     'state': types.SimpleNamespace(model_source_path=''),
                     '_show_ui_text': mock.Mock()}
        exec(compile(module, str(path), 'exec'), namespace)
        namespace['_show_diagnostics']()
        title, text = namespace['_show_ui_text'].call_args[0]
        self.assertEqual(title, 'action.show_diagnostics')
        self.assertIn('Qt: 6.7.3', text)
        self.assertIn('Model: -', text)


if __name__ == '__main__':
    unittest.main()
