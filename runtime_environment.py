"""Machine-local native runtime settings shared by preflight and the GUI."""

import importlib
import json
import os
from pathlib import Path
import sys


_DLL_HANDLES = []  # DLL search registrations must outlive native OCC objects.


def configure_runtime():
    """Apply an explicitly configured OCC prefix before importing native modules.

    Normally the selected interpreter owns the entire stack. An optional prefix
    supports the existing same-Python-ABI interpreter workaround on Windows.
    """
    configured = os.environ.get("SCANNING_APP_OCC_PREFIX", "").strip()
    if not configured:
        return None
    prefix = Path(configured).expanduser().resolve()
    packages = prefix / "Lib" / "site-packages"
    dlls = prefix / "Library" / "bin"
    if not (packages / "OCC").is_dir() or not dlls.is_dir():
        raise RuntimeError("SCANNING_APP_OCC_PREFIX lacks OCC packages or DLLs: {}".format(prefix))
    for record_path in (prefix / "conda-meta").glob("python-*.json"):
        record = json.loads(record_path.read_text(encoding="utf-8"))
        if record.get("name") != "python":
            continue
        version = tuple(int(part) for part in record["version"].split(".")[:2])
        if version != sys.version_info[:2]:
            raise RuntimeError(
                "OCC prefix Python {}.{} does not match interpreter {}.{}".format(
                    *version, *sys.version_info[:2]))
    if "OCC" in sys.modules:
        raise RuntimeError("Configure SCANNING_APP_OCC_PREFIX before importing OCC")
    if hasattr(os, "add_dll_directory"):
        for directory in (dlls, prefix):
            _DLL_HANDLES.append(os.add_dll_directory(str(directory)))
    # An explicit local override takes precedence over the interpreter's packages.
    sys.path.insert(0, str(packages))
    return str(prefix)


def select_qt_binding():
    """Use one binding consistently for preflight, plugins and OCC's viewer."""
    requested = os.environ.get("SCANNING_APP_QT_BACKEND", "").strip().lower()
    bindings = {"pyqt5": "PyQt5", "pyside6": "PySide6"}
    if requested and requested not in bindings:
        raise RuntimeError("SCANNING_APP_QT_BACKEND must be pyqt5 or pyside6")
    candidates = [bindings[requested]] if requested else ["PyQt5", "PySide6"]
    for binding in candidates:
        try:
            importlib.import_module(binding + ".QtWidgets")
        except (ImportError, OSError):
            continue
        prefix = os.environ.get("SCANNING_APP_OCC_PREFIX", "").strip()
        if prefix:
            qt_directory = "qt6" if binding == "PySide6" else "qt"
            plugins = Path(prefix) / "Library" / "lib" / qt_directory / "plugins"
            if (plugins / "platforms").is_dir():
                os.environ["QT_PLUGIN_PATH"] = str(plugins)
                os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = str(plugins / "platforms")
        os.environ["QT_API"] = binding.lower()
        os.environ["PYTHONOCC_BACKEND"] = binding.lower()
        return binding
    raise RuntimeError("Configured Qt binding cannot load: {}".format(
        ", ".join(candidates)))
