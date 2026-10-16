import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import runtime_environment as runtime


class RuntimeEnvironmentTests(unittest.TestCase):
    def _prefix(self, directory, version=None):
        prefix = Path(directory)
        (prefix / 'Lib/site-packages/OCC').mkdir(parents=True)
        (prefix / 'Library/bin').mkdir(parents=True)
        (prefix / 'conda-meta').mkdir()
        version = version or '{}.{}.0'.format(*sys.version_info[:2])
        (prefix / 'conda-meta/python-test.json').write_text(json.dumps(
            {'name': 'python', 'version': version}), encoding='utf-8')
        return prefix

    def test_default_runtime_leaves_import_paths_unchanged(self):
        before = list(sys.path)
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertIsNone(runtime.configure_runtime())
        self.assertEqual(sys.path, before)

    def test_explicit_prefix_precedes_interpreter_packages_and_retains_handles(self):
        with tempfile.TemporaryDirectory() as directory:
            prefix = self._prefix(directory)
            with mock.patch.dict(os.environ, {'SCANNING_APP_OCC_PREFIX': directory}, clear=True), \
                    mock.patch.dict(sys.modules), \
                    mock.patch.object(sys, 'path', list(sys.path)), \
                    mock.patch.object(runtime, '_DLL_HANDLES', []), \
                    mock.patch.object(os, 'add_dll_directory', create=True) as add_dll:
                sys.modules.pop('OCC', None)
                self.assertEqual(runtime.configure_runtime(), str(prefix.resolve()))
                self.assertEqual(sys.path[0], str(prefix.resolve() / 'Lib/site-packages'))
                self.assertEqual(add_dll.call_count, 2)
                self.assertEqual(len(runtime._DLL_HANDLES), 2)

    def test_mismatched_python_abi_is_rejected_before_native_loading(self):
        with tempfile.TemporaryDirectory() as directory:
            self._prefix(directory, '0.0.0')
            with mock.patch.dict(os.environ, {'SCANNING_APP_OCC_PREFIX': directory}, clear=True), \
                    mock.patch.object(os, 'add_dll_directory', create=True) as add_dll:
                with self.assertRaisesRegex(RuntimeError, 'does not match interpreter'):
                    runtime.configure_runtime()
                add_dll.assert_not_called()

    def test_incomplete_prefix_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(os.environ, {'SCANNING_APP_OCC_PREFIX': directory}, clear=True):
                with self.assertRaisesRegex(RuntimeError, 'lacks OCC'):
                    runtime.configure_runtime()

    def test_prefix_cannot_replace_already_loaded_occ(self):
        with tempfile.TemporaryDirectory() as directory:
            self._prefix(directory)
            with mock.patch.dict(os.environ, {'SCANNING_APP_OCC_PREFIX': directory}, clear=True), \
                    mock.patch.dict(sys.modules, {'OCC': object()}):
                with self.assertRaisesRegex(RuntimeError, 'before importing OCC'):
                    runtime.configure_runtime()

    def test_explicit_qt_binding_does_not_silently_choose_another(self):
        with mock.patch.dict(os.environ, {'SCANNING_APP_QT_BACKEND': 'pyqt5'}, clear=True), \
                mock.patch.object(runtime.importlib, 'import_module', side_effect=ImportError) as importer:
            with self.assertRaisesRegex(RuntimeError, 'PyQt5'):
                runtime.select_qt_binding()
            importer.assert_called_once_with('PyQt5.QtWidgets')

    def test_automatic_qt_falls_back_to_pyside6(self):
        with mock.patch.dict(os.environ, {}, clear=True), \
                mock.patch.object(runtime.importlib, 'import_module', side_effect=[ImportError(), object()]):
            self.assertEqual(runtime.select_qt_binding(), 'PySide6')
            self.assertEqual(os.environ['QT_API'], 'pyside6')

    def test_local_qt_prefix_controls_platform_plugins(self):
        with tempfile.TemporaryDirectory() as directory:
            plugins = Path(directory) / 'Library/lib/qt6/plugins'
            (plugins / 'platforms').mkdir(parents=True)
            values = {'SCANNING_APP_OCC_PREFIX': directory,
                      'SCANNING_APP_QT_BACKEND': 'pyside6',
                      'QT_QPA_PLATFORM_PLUGIN_PATH': 'wrong-machine-path'}
            with mock.patch.dict(os.environ, values, clear=True), \
                    mock.patch.object(runtime.importlib, 'import_module', return_value=object()):
                self.assertEqual(runtime.select_qt_binding(), 'PySide6')
                self.assertEqual(os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'], str(plugins / 'platforms'))


if __name__ == '__main__':
    unittest.main()
