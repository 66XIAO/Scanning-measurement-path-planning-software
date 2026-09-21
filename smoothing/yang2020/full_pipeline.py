"""End-to-end Python pipeline for the paper's C3 corner smoother.

Reads a tool path JSON (schema 1.1) and produces a smoothed-path JSON
matching the schema described in REPRODUCTION_PURPOSE.md §2.2.

Usage
-----
python -m scripts.prototype.full_pipeline \\
       --input  data/test_paths/paper_fig9_path.json \\
       --output results/csharp_output/paper_fig9_path_smoothed.json \\
       --eps-pw 0.8 --eps-ow 0.01
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


from .orientation_decompose import decompose
from .length_solver import (
    solve_position_length, solve_orientation_length,
)
from .corner_spline import (
    build_corner_spline, KNOTS_CORNER, DEGREE_CORNER,
)
from .synchronization_proto import (
    synchronization_lengths,
    build_sync_position_cps, build_sync_orientation_cps,
    KNOTS_SYNC, DEGREE_SYNC,
)


SCHEMA_VERSION = "1.1"


def load_tool_path(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    """Return (positions [N,3], orientations_R [N,3,3], raw_dict)."""
    raw = json.loads(path.read_text(encoding="utf-8"))
    positions = np.array([wp["position"] for wp in raw["waypoints"]], dtype=float)
    orientations = np.array([wp["orientation"] for wp in raw["waypoints"]], dtype=float)
    return positions, orientations, raw


def smooth(positions: np.ndarray, orientations: np.ndarray,
           eps_pw: float, eps_ow: float, *, continuous_angles=None) -> dict:
    """Run the full Section 3-4-5 pipeline. Returns a result dict ready to dump."""
    n = positions.shape[0]
    if n < 3:
        raise ValueError("need at least 3 waypoints to form one corner")

    # 1. Decompose orientations -> rotary angles psi_i
    psi = (np.array([decompose(R) for R in orientations]) if continuous_angles is None
           else np.asarray(continuous_angles, dtype=float))
    if psi.shape != (n, 3) or not np.all(np.isfinite(psi)):
        raise ValueError('continuous_angles must be finite N x 3')

    # 2. For each interior corner i = 1..n-2, build position & orientation
    #    corner B-splines.
    pos_corner_cps = []   # list of (7,3) arrays
    ori_corner_cps = []
    l_p_list, l_o_list = [], []
    for i in range(1, n - 1):
        l_p = solve_position_length(positions[i - 1], positions[i], positions[i + 1], eps_pw)
        l_o = solve_orientation_length(psi[i - 1], psi[i], psi[i + 1], eps_ow)
        sp_pos = build_corner_spline(positions[i - 1], positions[i], positions[i + 1], l_p)
        sp_ori = build_corner_spline(psi[i - 1],       psi[i],       psi[i + 1],       l_o)
        pos_corner_cps.append(sp_pos.control_points.copy())
        ori_corner_cps.append(sp_ori.control_points.copy())
        l_p_list.append(l_p)
        l_o_list.append(l_o)

    # 3. For each residual line between consecutive corners, build sync splines.
    sync_pos_cps = []
    sync_ori_cps = []
    sync_lengths_all = []
    for i in range(1, len(pos_corner_cps)):
        Q6_prev = pos_corner_cps[i - 1][6]
        Q0_next = pos_corner_cps[i][0]
        Phi6_prev = ori_corner_cps[i - 1][6]
        Phi0_next = ori_corner_cps[i][0]
        P_mid     = 0.5 * (Q6_prev + Q0_next)
        Psi_mid   = 0.5 * (Phi6_prev + Phi0_next)

        L = synchronization_lengths(
            Q6_prev, Q0_next, P_mid,
            Phi6_prev, Phi0_next, Psi_mid,
            l_p_list[i - 1], l_p_list[i],
            l_o_list[i - 1], l_o_list[i],
        )
        sync_pos_cps.append(build_sync_position_cps(
            Q6_prev, Q0_next, P_mid, L["l_pla"], L["l_plb"]))
        sync_ori_cps.append(build_sync_orientation_cps(
            Phi6_prev, Phi0_next, Psi_mid, L["l_ola"], L["l_olb"]))
        sync_lengths_all.append(L)

    # 4. Linear pieces — first piece (start to first sync/corner) and last piece
    linear_pieces = [
        {"start": positions[0].tolist(),
         "end":   pos_corner_cps[0][0].tolist()},
        {"start": pos_corner_cps[-1][6].tolist(),
         "end":   positions[-1].tolist()},
    ]

    return {
        "schema_version": SCHEMA_VERSION,
        "tolerances": {"eps_pw": eps_pw, "eps_ow": eps_ow},
        "linear_pieces": linear_pieces,
        "position_corner_splines": [
            {"knots": KNOTS_CORNER.tolist(), "degree": DEGREE_CORNER,
             "control_points": cp.tolist()} for cp in pos_corner_cps
        ],
        "orientation_corner_splines": [
            {"knots": KNOTS_CORNER.tolist(), "degree": DEGREE_CORNER,
             "control_points": cp.tolist()} for cp in ori_corner_cps
        ],
        "sync_position_splines": [
            {"knots": KNOTS_SYNC.tolist(), "degree": DEGREE_SYNC,
             "control_points": cp.tolist()} for cp in sync_pos_cps
        ],
        "sync_orientation_splines": [
            {"knots": KNOTS_SYNC.tolist(), "degree": DEGREE_SYNC,
             "control_points": cp.tolist()} for cp in sync_ori_cps
        ],
        "transition_lengths": {
            "l_p": l_p_list,
            "l_o": l_o_list,
            "sync": sync_lengths_all,
        },
        "continuity": {"c0": 0.0, "c1": 0.0, "c2": 0.0, "c3": 0.0},
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input",  required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--eps-pw", type=float, default=0.8)
    parser.add_argument("--eps-ow", type=float, default=0.01)
    args = parser.parse_args()

    positions, orientations, raw = load_tool_path(Path(args.input))
    eps_pw = float(raw.get("tolerances", {}).get("eps_pw", args.eps_pw))
    eps_ow = float(raw.get("tolerances", {}).get("eps_ow", args.eps_ow))

    result = smooth(positions, orientations, eps_pw, eps_ow)
    result["input_path"] = args.input

    # Populate the continuity block by calling the validation routine on the
    # result we just built. Keeps the JSON self-describing for downstream tools.
    from scripts.analysis.c3_validation import validate
    metrics = validate(result, positions)
    result["continuity"] = {
        "c0": metrics["A1_C0"],
        "c1": metrics["A2_C1"],
        "c2": metrics["A3_C2"],
        "c3": metrics["A4_C3"],
    }

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(f"[OK] wrote {out}")
    print(f"     l_p = {result['transition_lengths']['l_p']}")
    print(f"     l_o = {result['transition_lengths']['l_o']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
