"""Headless regression for complex trimmed STEP mesh-grid segmentation.

Generated JSON belongs under ``diagnostics/cardoor0808/``.  That directory is
ignored by Git so real model paths and machine-specific timings do not pollute
the source tree.
"""

import argparse
import json
import os
import sys
import time


PYTHONOCC_PREFIX = os.environ.get(
    "PYTHONOCC_PREFIX", r"D:\Env\conda\2024\envs\Pythonocc")
SITE_PACKAGES = os.path.join(PYTHONOCC_PREFIX, "Lib", "site-packages")
DLL_DIRECTORY = os.path.join(PYTHONOCC_PREFIX, "Library", "bin")

if hasattr(os, "add_dll_directory"):
    _dll_handle = os.add_dll_directory(DLL_DIRECTORY)
if SITE_PACKAGES not in sys.path:
    sys.path.insert(0, SITE_PACKAGES)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from cad_io import load_cad_shape
from config import DEFAULT_SEGMENT_U, DEFAULT_SEGMENT_V
from geometry import segment_model
from surface_segmentation import inspect_shape_topology, is_surface_patch
from OCC.Core.BRepClass import BRepClass_FaceClassifier
from OCC.Core.TopAbs import TopAbs_IN, TopAbs_ON
from OCC.Core.gp import gp_Pnt2d


def validate_model(file_path, u, v, linear_deflection=None,
                   angular_deflection=0.5, validate_centers=True):
    started = time.perf_counter()
    shape = load_cad_shape(file_path)
    load_seconds = time.perf_counter() - started
    topology = inspect_shape_topology(shape)

    started = time.perf_counter()
    patches, diagnostics = segment_model(
        shape, u, v, return_diagnostics=True, strategy="auto",
        mesh_linear_deflection=linear_deflection,
        mesh_angular_deflection=angular_deflection)
    segmentation_seconds = time.perf_counter() - started
    patch_summaries = [
        patch.summary() for patch in patches if is_surface_patch(patch)]
    invalid_center_indices = []
    invalid_normal_indices = []
    if validate_centers:
        for index, patch in enumerate(patches):
            if not is_surface_patch(patch):
                continue
            classifier = BRepClass_FaceClassifier(
                patch.source_face,
                gp_Pnt2d(*patch.representative_uv), 1e-6, True)
            if classifier.State() not in (TopAbs_IN, TopAbs_ON):
                invalid_center_indices.append(index)
            if abs(patch.normal.Magnitude() - 1.0) > 1e-6:
                invalid_normal_indices.append(index)
    passed = bool(patches) and not diagnostics.get("warnings")
    if diagnostics.get("strategy") == "mesh_grid":
        passed = passed and all(is_surface_patch(patch) for patch in patches)
        passed = passed and abs(diagnostics.get("area_ratio", 0.0) - 1.0) <= 0.02
        if validate_centers:
            passed = passed and not invalid_center_indices and not invalid_normal_indices

    return {
        "schema_version": "1.0",
        "file": os.path.abspath(file_path),
        "file_size_bytes": os.path.getsize(file_path),
        "u": u,
        "v": v,
        "load_seconds": load_seconds,
        "segmentation_seconds": segmentation_seconds,
        "topology": topology,
        "diagnostics": diagnostics,
        "center_normal_validation": {
            "performed": validate_centers,
            "invalid_center_count": len(invalid_center_indices),
            "invalid_center_indices": invalid_center_indices,
            "invalid_normal_count": len(invalid_normal_indices),
            "invalid_normal_indices": invalid_normal_indices,
        },
        "patches": patch_summaries,
        "passed": passed,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate trim-aware car-door STEP segmentation")
    parser.add_argument("file")
    parser.add_argument("--u", type=int, default=DEFAULT_SEGMENT_U)
    parser.add_argument("--v", type=int, default=DEFAULT_SEGMENT_V)
    parser.add_argument("--linear-deflection", type=float)
    parser.add_argument("--angular-deflection", type=float, default=0.5)
    parser.add_argument("--json-output")
    parser.add_argument(
        "--summary-only", action="store_true",
        help="Print diagnostics without the per-patch records")
    parser.add_argument(
        "--skip-center-validation", action="store_true",
        help="Skip the slower per-patch OCC centre classifier for parameter sweeps")
    args = parser.parse_args(argv)

    result = validate_model(
        args.file, args.u, args.v,
        linear_deflection=args.linear_deflection,
        angular_deflection=args.angular_deflection,
        validate_centers=not args.skip_center_validation)
    rendered = json.dumps(result, ensure_ascii=False, indent=2)
    if args.summary_only:
        summary = dict(result)
        summary["patches"] = "{} records written to JSON output".format(
            len(result["patches"]))
        print(json.dumps(summary, ensure_ascii=False, indent=2))
    else:
        print(rendered)
    if args.json_output:
        output_path = os.path.abspath(args.json_output)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
