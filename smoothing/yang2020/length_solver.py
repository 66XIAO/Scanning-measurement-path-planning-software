"""Numpy reference of Eqs. 4 / 7 / 17 / 27 — corner-spline transition lengths.

l_p  : position transition length (Eq. 7)
l_o  : orientation transition length (Eq. 27)
"""
from __future__ import annotations

import numpy as np

from .orientation_decompose import orientation_jacobian


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    """Unsigned angle between two vectors in radians."""
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-15 or n2 < 1e-15:
        return 0.0
    cos_t = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
    return float(np.arccos(cos_t))


def solve_position_length(p1: np.ndarray, p2: np.ndarray, p3: np.ndarray, eps_pw: float) -> float:
    """Eq. 7: l_p = min{ (4/3)·eps_pw / cos(theta_p / 2),
                        (2/15)·||P1P2||,
                        (2/15)·||P2P3|| }

    theta_p is the angle between the two adjacent linear position segments,
    i.e. between vectors (P1−P2) and (P3−P2).
    """
    v_a = p1 - p2
    v_b = p3 - p2
    seg_len_in  = np.linalg.norm(p2 - p1)
    seg_len_out = np.linalg.norm(p3 - p2)
    theta_p = angle_between(v_a, v_b)

    chord_term = (4.0 / 3.0) * eps_pw / np.cos(theta_p / 2.0)
    geom_term  = (2.0 / 15.0) * min(seg_len_in, seg_len_out)
    return float(min(chord_term, geom_term))


def solve_orientation_length(psi1: np.ndarray, psi2: np.ndarray, psi3: np.ndarray,
                             eps_ow: float) -> float:
    """Eq. 27: l_o = min{ 16·sin(eps_ow/2) / (3·||J_o · (e_oa + e_ob)||),
                          (2/15)·||Psi1Psi2||,
                          (2/15)·||Psi2Psi3|| }

    Psi1, Psi2, Psi3 are 3-vectors of rotary angles (alpha, beta, gamma).
    """
    v_a = psi1 - psi2
    v_b = psi3 - psi2
    seg_len_in  = np.linalg.norm(v_a)
    seg_len_out = np.linalg.norm(v_b)
    e_oa = v_a / max(seg_len_in,  1e-15)
    e_ob = v_b / max(seg_len_out, 1e-15)

    Jo = orientation_jacobian(psi2[0], psi2[1], psi2[2])
    denom = np.linalg.norm(Jo @ (e_oa + e_ob))
    if denom < 1e-15:
        ortho_term = np.inf
    else:
        ortho_term = 16.0 * np.sin(eps_ow / 2.0) / (3.0 * denom)

    geom_term = (2.0 / 15.0) * min(seg_len_in, seg_len_out)
    return float(min(ortho_term, geom_term))
