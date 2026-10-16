"""Portable launcher and dependency preflight for the integrated application.

The PowerShell launcher selects a Conda-managed interpreter. This module then
verifies that the selected environment contains the native GUI/CAD stack before
``main`` is imported, so environment problems produce an actionable message.
"""

from __future__ import print_function

import argparse
import importlib
import json
import os
from pathlib import Path
import sys

from robodk_discovery import discover_robodk_api_path
from runtime_environment import configure_runtime, select_qt_binding


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

    try:
        versions["qt"] = select_qt_binding()
    except RuntimeError as exc:
        failures.append("Qt: {}".format(exc))
    occ = sys.modules.get("OCC")
    versions["occ_path"] = getattr(occ, "__file__", "unavailable")
    versions["occ_prefix"] = os.environ.get("SCANNING_APP_OCC_PREFIX", "")
    if occ is not None:
        try:
            from OCC.Core.BRepCheck import BRepCheck_ListOfStatus
            from surface_segmentation import _iter_brep_statuses
            list(_iter_brep_statuses(BRepCheck_ListOfStatus()))
        except Exception as exc:
            failures.append("OCC status-list compatibility: {}".format(exc))

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
    print("  OCC module: {}".format(summary["occ_path"]))
    if summary["occ_prefix"]:
        print("  Explicit OCC prefix: {}".format(summary["occ_prefix"]))
    print("  NumPy: {}".format(summary.get("numpy", "available")))
    print("  RoboDK API: {}".format(summary["robodk_api"]))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    checks = parser.add_mutually_exclusive_group()
    checks.add_argument("--check", action="store_true",
                        help="validate the selected environment without opening the GUI")
    checks.add_argument("--check-model", metavar="STEP_OR_IGES",
                        help="load and segment a model without opening the GUI")
    parser.add_argument("--u", type=int, default=5)
    parser.add_argument("--v", type=int, default=7)
    parser.add_argument("--skip-check", action="store_true",
                        help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.check and args.skip_check:
        parser.error("--check and --skip-check cannot be used together")
    configure_runtime()
    if not args.skip_check:
        summary = preflight()
        _print_summary(summary)
    if args.check:
        return 0
    if args.check_model:
        from cad_io import load_cad_shape
        from geometry import segment_model
        patches, diagnostics = segment_model(
            load_cad_shape(args.check_model), args.u, args.v,
            strategy="auto", return_diagnostics=True)
        print(json.dumps({"file": str(Path(args.check_model).resolve()),
                          "patch_count": len(patches),
                          "diagnostics": diagnostics}, ensure_ascii=False, indent=2))
        return 0 if patches and not diagnostics.get("warnings") else 1

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
