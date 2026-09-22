"""Portable launcher and dependency preflight for the integrated application.

The PowerShell launcher selects a Conda-managed interpreter. This module then
verifies that the selected environment contains the native GUI/CAD stack before
``main`` is imported, so environment problems produce an actionable message.
"""

from __future__ import print_function

import argparse
import importlib
import os
from pathlib import Path
import sys

from robodk_discovery import discover_robodk_api_path


PROJECT_ROOT = Path(__file__).resolve().parent


def _configure_optional_robodk_api():
    """Configure a discovered RoboDK API without embedding a machine path."""
    discovered = discover_robodk_api_path()
    if discovered:
        os.environ["ROBODK_API_PATH"] = discovered
    return discovered


def preflight():
    """Return an environment summary or raise for a missing requirement."""
    versions = {"python": sys.version.split()[0], "executable": sys.executable}
    failures = []
    for module_name in (
            "numpy", "OCC", "OCC.Core.gp", "OCC.Extend.DataExchange",
            "OCC.Display.OCCViewer"):
        try:
            module = importlib.import_module(module_name)
            root_name = module_name.split(".")[0]
            module_version = getattr(
                module, "VERSION", getattr(module, "__version__", "available"))
            versions.setdefault(root_name, module_version)
        except Exception as exc:
            failures.append("{}: {}".format(module_name, exc))

    qt_backend = None
    for backend, module_name in (("PyQt5", "PyQt5.QtWidgets"),
                                 ("PySide6", "PySide6.QtWidgets")):
        try:
            importlib.import_module(module_name)
            qt_backend = backend
            break
        except Exception:
            pass
    if qt_backend is None:
        failures.append("Qt: neither PyQt5 nor PySide6 can load")
    else:
        versions["qt"] = qt_backend

    versions["robodk_api"] = _configure_optional_robodk_api() or "not found (optional)"
    if failures:
        message = "\n  - ".join(["Selected Conda environment is not usable:"] + failures)
        raise RuntimeError(
            message +
            "\nChoose another environment with SCANNING_APP_CONDA_ENV, for example:\n"
            "  $env:SCANNING_APP_CONDA_ENV = 'my-pythonocc-env'")
    return versions


def _print_summary(summary):
    print("Environment preflight passed")
    print("  Python: {} ({})".format(summary["python"], summary["executable"]))
    print("  Qt: {}".format(summary["qt"]))
    print("  PythonOCC: {}".format(summary.get("OCC", "available")))
    print("  NumPy: {}".format(summary.get("numpy", "available")))
    print("  RoboDK API: {}".format(summary["robodk_api"]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true",
                        help="validate the selected environment without opening the GUI")
    parser.add_argument("--skip-check", action="store_true",
                        help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.check and args.skip_check:
        parser.error("--check and --skip-check cannot be used together")
    if not args.skip_check:
        summary = preflight()
        _print_summary(summary)
    if args.check:
        return 0

    os.chdir(str(PROJECT_ROOT))
    from main import run
    run()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RuntimeError as exc:
        print("[ERROR] {}".format(exc), file=sys.stderr)
        raise SystemExit(2)
