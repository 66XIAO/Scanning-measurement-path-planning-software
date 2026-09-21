"""Generic non-uniform B-spline evaluator.

We use a simple but explicit Cox-de Boor recursion. For our needs (degree 5,
either 7 or 9 control points), performance is not critical — clarity is.

A B-spline curve r(u) = sum_i C_i * N_{i,p}(u), u ∈ [u_p, u_{n+1}].
For our paper:
  - corner spline: degree p = 5, 7 control points, knots U (length 13)
  - sync   spline: degree p = 5, 9 control points, knots V (length 15)

The evaluator returns r(u) and its first three derivatives w.r.t. u.
"""
from __future__ import annotations

import numpy as np


class BSpline:
    """Non-uniform B-spline of given degree with explicit knot vector."""

    def __init__(self, control_points: np.ndarray, knots: np.ndarray, degree: int):
        cp = np.asarray(control_points, dtype=float)
        kn = np.asarray(knots, dtype=float)
        if cp.ndim != 2:
            raise ValueError("control_points must be 2-D (n_cp, dim)")
        if len(kn) != len(cp) + degree + 1:
            raise ValueError(
                f"knot length {len(kn)} != cp + degree + 1 = {len(cp) + degree + 1}")
        self.control_points = cp
        self.knots = kn
        self.degree = int(degree)

    @property
    def n_cp(self) -> int:
        return self.control_points.shape[0]

    @property
    def dim(self) -> int:
        return self.control_points.shape[1]

    # ---- Cox-de Boor basis ---------------------------------------------------
    def _basis(self, u: float, p: int, knots: np.ndarray) -> np.ndarray:
        """Return the (n_cp_p)-dim vector of basis functions of degree p at u,
        where n_cp_p = len(knots) - p - 1.
        """
        # Degree-0 basis has length len(knots) - 1.
        N = np.zeros(len(knots) - 1)
        last = knots[-1]
        if u >= last - 1e-15:
            # Right-endpoint convention: activate the last non-empty span.
            for i in range(len(N) - 1, -1, -1):
                if knots[i + 1] - knots[i] > 1e-15 and knots[i] < last:
                    N[i] = 1.0
                    break
        else:
            for i in range(len(N)):
                if knots[i] <= u < knots[i + 1]:
                    N[i] = 1.0
                    break
        # Recurse to degree p
        for d in range(1, p + 1):
            new_N = np.zeros(len(knots) - d - 1)
            for i in range(len(new_N)):
                left  = (knots[i + d]     - knots[i])
                right = (knots[i + d + 1] - knots[i + 1])
                a = ((u - knots[i])     / left)  * N[i]      if left  > 1e-15 else 0.0
                b = ((knots[i + d + 1] - u) / right) * N[i + 1] if right > 1e-15 else 0.0
                new_N[i] = a + b
            N = new_N
        return N

    # ---- Evaluation ----------------------------------------------------------
    def evaluate(self, u: float) -> np.ndarray:
        """Return r(u)."""
        N = self._basis(u, self.degree, self.knots)
        return N @ self.control_points

    def derivative_control_points(self, order: int = 1):
        """Return (control_points, knots, degree) of the order-th derivative B-spline."""
        cp = self.control_points
        kn = self.knots
        p  = self.degree
        for k in range(order):
            # New CP: Q_i = p * (P_{i+1} - P_i) / (knots[i+p+1] - knots[i+1])
            new_cp = np.zeros((cp.shape[0] - 1, cp.shape[1]))
            for i in range(new_cp.shape[0]):
                denom = kn[i + p + 1] - kn[i + 1]
                if denom < 1e-15:
                    new_cp[i] = 0.0
                else:
                    new_cp[i] = p * (cp[i + 1] - cp[i]) / denom
            cp = new_cp
            kn = kn[1:-1]
            p -= 1
            if p < 0:
                return np.zeros((1, self.dim)), kn, 0
        return cp, kn, p

    def evaluate_derivative(self, u: float, order: int) -> np.ndarray:
        """Return d^order r / d u^order at u."""
        if order == 0:
            return self.evaluate(u)
        cp, kn, p = self.derivative_control_points(order)
        if p < 0:
            return np.zeros(self.dim)
        # Build a temporary BSpline-like basis evaluation
        N = self._basis(u, p, kn)
        return N @ cp

    def evaluate_all(self, u: float) -> dict:
        """Return r, r', r'', r''' at u in a dict."""
        return {
            "r":   self.evaluate(u),
            "r1":  self.evaluate_derivative(u, 1),
            "r2":  self.evaluate_derivative(u, 2),
            "r3":  self.evaluate_derivative(u, 3),
        }
