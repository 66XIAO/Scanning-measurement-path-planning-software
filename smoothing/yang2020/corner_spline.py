"""Position / orientation corner spline construction (Eqs. 2–4 and 13–17).

Both share the same knot vector U (Eq. 3) and the same control-point structure;
the only difference is the underlying space (R^3 for tip position, R^3 for
rotary angles psi = (alpha, beta, gamma)).

Control points are computed in **closed form** — no iteration.
"""
from __future__ import annotations

import numpy as np

from .bspline import BSpline

# Knot vector U (Eq. 3) — degree 5, 7 control points => 13 knots
KNOTS_CORNER = np.array(
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
)
DEGREE_CORNER = 5


def _unit(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    return v / n if n > 1e-15 else np.zeros_like(v)


def build_corner_control_points(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray,
                                length: float) -> np.ndarray:
    """Return 7 control points Q_0..Q_6 (Eq. 4 / Eq. 17) given three waypoints
    and the transition length `length` (= l_p or l_o).

    p1, p2, p3 must live in the same Euclidean space (R^3). Works for both
    tip position and rotary-angle representations.
    """
    p1 = np.asarray(p1, dtype=float)
    p2 = np.asarray(p2, dtype=float)
    p3 = np.asarray(p3, dtype=float)

    e_a = _unit(p1 - p2)
    e_b = _unit(p3 - p2)

    Q3 = p2.copy()
    # Distances along e_pa / e_pb from Q3 are 1·l, 2·l, 2.5·l (and symmetric).
    # These are the unique distances that satisfy
    #   80 Q0 − 120 Q1 + 40 Q2 = 0   (P''(0) = 0)
    #  −480 Q0 + 840 Q1 − 480 Q2 + 120 Q3 = 0  (P'''(0) = 0)
    # (basis derivative coefficients verified numerically against scipy.BSpline).
    Q2 = Q3 + 1.0 * length * e_a
    Q1 = Q3 + 2.0 * length * e_a
    Q0 = Q3 + 2.5 * length * e_a
    Q4 = Q3 + 1.0 * length * e_b
    Q5 = Q3 + 2.0 * length * e_b
    Q6 = Q3 + 2.5 * length * e_b
    return np.vstack([Q0, Q1, Q2, Q3, Q4, Q5, Q6])


def build_corner_spline(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray,
                        length: float) -> BSpline:
    cps = build_corner_control_points(p1, p2, p3, length)
    return BSpline(cps, KNOTS_CORNER, DEGREE_CORNER)
