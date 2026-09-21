"""Synchronization spline construction (Eqs. 28–43).

For each residual linear segment between two consecutive corner splines,
we build a degree-5 B-spline with 9 control points whose end control points
coincide with the corner-spline endpoints, and whose intermediate control
points are placed so that

  - C^3 continuity holds at v=0 (junction with previous corner spline) and
    v=1 (junction with next corner spline);
  - the orientation spline is *synchronized* to the position spline up to the
    third derivative w.r.t. the tool-tip arclength.

Inputs from the corner-smoothing stage (per residual line i):
  Q_{6,i-1} : end control point of previous position corner spline
  Q_{0,i}   : start control point of next     position corner spline
  P_{m,i-1} : midpoint of residual position line  (== G_4)
  Phi_{6,i-1}: end control point of previous orientation corner spline
  Phi_{0,i} : start control point of next     orientation corner spline
  Psi_{m,i-1}: midpoint of residual orientation line  (== H_4)
  l_p_prev, l_p_next : adjacent position transition lengths
  l_o_prev, l_o_next : adjacent orientation transition lengths

Outputs: 9 position CPs (G_0..G_8) and 9 orientation CPs (H_0..H_8).
"""
from __future__ import annotations

import numpy as np

from .bspline import BSpline

# Knot vector V (Eq. 28) — degree 5, 9 control points => 15 knots
KNOTS_SYNC = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
)
DEGREE_SYNC = 5


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-15 else np.zeros_like(v)


def synchronization_lengths(
    Q6_prev: np.ndarray, Q0_next: np.ndarray, P_mid: np.ndarray,
    Phi6_prev: np.ndarray, Phi0_next: np.ndarray, Psi_mid: np.ndarray,
    l_p_prev: float, l_p_next: float,
    l_o_prev: float, l_o_next: float,
) -> dict:
    """Compute l_pla, l_plb, l_ola, l_olb per Eqs. 38–41."""
    # k_a, k_b — analytic synchronization ratios (Eq. 38, 40)
    k_a = l_p_prev / l_o_prev if l_o_prev > 1e-15 else 1.0
    k_b = l_p_next / l_o_next if l_o_next > 1e-15 else 1.0

    # Eq. 39: l_pla = min{ (1/4)||Q_{6,i-1} P_{m,i-1}||,  (1/4)||Phi_{6,i-1} Psi_{m,i-1}|| · k_a }
    pos_left  = 0.25 * np.linalg.norm(P_mid - Q6_prev)
    ori_left  = 0.25 * np.linalg.norm(Psi_mid - Phi6_prev) * k_a
    l_pla = min(pos_left, ori_left)

    pos_right = 0.25 * np.linalg.norm(P_mid - Q0_next)
    ori_right = 0.25 * np.linalg.norm(Psi_mid - Phi0_next) * k_b
    l_plb = min(pos_right, ori_right)

    # Eqs. 38, 40: l_ola = (l_o_prev / l_p_prev) · l_pla, similarly for l_olb
    l_ola = (l_o_prev / l_p_prev) * l_pla if l_p_prev > 1e-15 else 0.0
    l_olb = (l_o_next / l_p_next) * l_plb if l_p_next > 1e-15 else 0.0

    return {
        "l_pla": float(l_pla),
        "l_plb": float(l_plb),
        "l_ola": float(l_ola),
        "l_olb": float(l_olb),
    }


def build_sync_position_cps(Q6_prev: np.ndarray, Q0_next: np.ndarray,
                            P_mid: np.ndarray,
                            l_pla: float, l_plb: float) -> np.ndarray:
    """Return 9 position CPs G_0..G_8 per Eq. 42."""
    e_pl = _unit(Q0_next - Q6_prev)

    G0 = Q6_prev.copy()
    G1 = G0 + l_pla * e_pl
    G2 = G0 + 2.0 * l_pla * e_pl
    G3 = G0 + 3.0 * l_pla * e_pl
    G4 = P_mid.copy()
    G8 = Q0_next.copy()
    G7 = G8 - l_plb * e_pl
    G6 = G8 - 2.0 * l_plb * e_pl
    G5 = G8 - 3.0 * l_plb * e_pl
    return np.vstack([G0, G1, G2, G3, G4, G5, G6, G7, G8])


def build_sync_orientation_cps(Phi6_prev: np.ndarray, Phi0_next: np.ndarray,
                               Psi_mid: np.ndarray,
                               l_ola: float, l_olb: float) -> np.ndarray:
    """Return 9 orientation CPs H_0..H_8 per Eq. 43."""
    e_ol = _unit(Phi0_next - Phi6_prev)

    H0 = Phi6_prev.copy()
    H1 = H0 + l_ola * e_ol
    H2 = H0 + 2.0 * l_ola * e_ol
    H3 = H0 + 3.0 * l_ola * e_ol
    H4 = Psi_mid.copy()
    H8 = Phi0_next.copy()
    H7 = H8 - l_olb * e_ol
    H6 = H8 - 2.0 * l_olb * e_ol
    H5 = H8 - 3.0 * l_olb * e_ol
    return np.vstack([H0, H1, H2, H3, H4, H5, H6, H7, H8])


def build_sync_position_spline(*args) -> BSpline:
    return BSpline(build_sync_position_cps(*args), KNOTS_SYNC, DEGREE_SYNC)


def build_sync_orientation_spline(*args) -> BSpline:
    return BSpline(build_sync_orientation_cps(*args), KNOTS_SYNC, DEGREE_SYNC)
