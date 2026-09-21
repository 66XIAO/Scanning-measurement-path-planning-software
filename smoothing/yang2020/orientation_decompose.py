"""Numpy reference of Eqs. 8–12: ZYX fixed-axis decomposition R <-> (alpha, beta, gamma)."""
from __future__ import annotations

import numpy as np

EPS = 1e-9


def compose(alpha: float, beta: float, gamma: float) -> np.ndarray:
    """Build R = R_Z(alpha) R_Y(beta) R_X(gamma) per Eq. 8.

    The 3 angles are sequential rotations about the *fixed* global X, Y, Z axes,
    applied in the order X first (gamma), then Y (beta), then Z (alpha).
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta),  np.sin(beta)
    cg, sg = np.cos(gamma), np.sin(gamma)
    return np.array([
        [ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg],
        [sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg],
        [    -sb,                cb * sg,                cb * cg],
    ])


def decompose(R: np.ndarray) -> tuple[float, float, float]:
    """Extract (alpha, beta, gamma) from a rotation matrix R per Eqs. 10–12.

    Branch handling for the singular cases beta = +/- 90 deg.
    Returns (alpha, beta, gamma) in radians.
    """
    R = np.asarray(R, dtype=float)
    r31 = R[2, 0]
    if abs(r31 + 1.0) < EPS:        # beta = +90 deg
        beta  = np.pi / 2
        alpha = 0.0
        gamma = np.arctan2(R[0, 1], R[1, 1])
    elif abs(r31 - 1.0) < EPS:      # beta = -90 deg
        beta  = -np.pi / 2
        alpha = 0.0
        gamma = -np.arctan2(R[0, 1], R[1, 1])
    else:
        beta  = np.arctan2(-r31, np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2))
        cb    = np.cos(beta)
        alpha = np.arctan2(R[1, 0] / cb, R[0, 0] / cb)
        gamma = np.arctan2(R[2, 1] / cb, R[2, 2] / cb)
    return float(alpha), float(beta), float(gamma)


def orientation_jacobian(alpha: float, beta: float, gamma: float) -> np.ndarray:
    """Eq. 23: J_o relating the unit tool orientation vector O = R r_ot (with r_ot = e_z)
    to the rotary angles Psi = (alpha, beta, gamma).

    O = (Cα Sβ Cγ + Sα Sγ, Sα Sβ Cγ − Cα Sγ, Cβ Cγ)^T  (Eq. 20 with r_ot = e_z)
    """
    ca, sa = np.cos(alpha), np.sin(alpha)
    cb, sb = np.cos(beta),  np.sin(beta)
    cg, sg = np.cos(gamma), np.sin(gamma)
    return np.array([
        [-sa * sb * cg + ca * sg, ca * cb * cg, -ca * sb * sg + sa * cg],
        [ ca * sb * cg + sa * sg, sa * cb * cg, -sa * sb * sg - ca * cg],
        [                    0.0,        -sb * cg,                -cb * sg],
    ])
