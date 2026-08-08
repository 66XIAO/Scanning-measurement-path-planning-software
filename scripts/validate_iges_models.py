"""Headless regression for real IGES model loading and face segmentation.

Run with the existing ``test`` Conda interpreter. The script reuses the
existing Pythonocc environment and does not install packages.
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

from OCC.Core.TopAbs import TopAbs_FACE
from OCC.Core.TopExp import TopExp_Explorer

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from geometry import segment_model
from cad_io import load_cad_shape


def count_faces(shape):
    count = 0
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    while explorer.More():
        count += 1
        explorer.Next()
    return count


def validate_model(file_path, u, v):
    started = time.perf_counter()
    shape = load_cad_shape(file_path)
    load_seconds = time.perf_counter() - started
    if shape is None or shape.IsNull():
        raise RuntimeError("IGES reader returned a null shape")
    face_count = count_faces(shape)
    started = time.perf_counter()
    patches, diagnostics = segment_model(
        shape, u, v, return_diagnostics=True)
    segmentation_seconds = time.perf_counter() - started
    return {
        "file": os.path.abspath(file_path),
        "file_size_bytes": os.path.getsize(file_path),
        "u": u,
        "v": v,
        "load_seconds": load_seconds,
        "segmentation_seconds": segmentation_seconds,
        "face_count": face_count,
        "patch_count": len(patches),
        "diagnostics": diagnostics,
        "passed": bool(patches) and abs(diagnostics["area_ratio"] - 1.0) <= 0.01,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate real IGES models")
    parser.add_argument("files", nargs="+")
    parser.add_argument("--u", type=int, default=6)
    parser.add_argument("--v", type=int, default=4)
    parser.add_argument("--json-output")
    args = parser.parse_args(argv)
    results = [validate_model(path, args.u, args.v) for path in args.files]
    output = {"schema_version": "1.0", "results": results,
              "all_passed": all(item["passed"] for item in results)}
    rendered = json.dumps(output, ensure_ascii=False, indent=2)
    print(rendered)
    if args.json_output:
        with open(args.json_output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    return 0 if output["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
