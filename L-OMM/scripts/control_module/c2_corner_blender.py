#!/usr/bin/env python3
"""Local quintic C² corner blending for xz-only reference (spec §5, §6, §8).

Per the deterministic-pitch spec rewrite, the blender no longer touches
pitch — it operates purely on xz.  Pitch is reconstructed downstream by
``GazePitch`` on the smoothed dense table (spec §10).

Curvature radius (spec §6):
  - geometric ceiling = min(r_max, α·ℓ_prev, α·ℓ_next)   (binding for nominal v_ref)
  - curvature  floor  = 2·v² · sin(Δφ/2) / a_lat_max     (monitor only at nominal v_ref)

The floor is recorded as ``curvature_lift`` / ``curvature_overflow`` flags
on the BlendSegment so it shows up in the reference summary metrics.  The
selected radius lifts to the floor only when geometric < floor.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np

from blend_segment import BlendSegment
from smoothing_params import SmoothingParams


class C2CornerBlender:
    """Quintic Hermite blend per corner (xz only, with R3 arclength reparam)."""

    def __init__(self, params: SmoothingParams | None = None) -> None:
        self.params = params if params is not None else SmoothingParams()

    def initial_radius(self,
                       p_prev: np.ndarray,
                       p_cur: np.ndarray,
                       p_next: np.ndarray,
                       v_ref: float = 0.04) -> Tuple[float, bool, bool]:
        """Return (r_i, curvature_lift, curvature_overflow) per spec §6.

        - curvature_lift: True iff r_geo < r_curv ≤ r_max (radius lifted to floor)
        - curvature_overflow: True iff r_curv > r_max (corner too tight for v_ref)

        At nominal v_ref both flags are typically False, so the chosen radius
        is identical to the pre-§6 geometric value.
        """
        l0 = float(np.linalg.norm(p_cur - p_prev))
        l1 = float(np.linalg.norm(p_next - p_cur))
        r_geo = min(self.params.r_max,
                    self.params.radius_alpha * l0,
                    self.params.radius_alpha * l1)
        if not self.params.enable_curvature_radius or l0 <= 1.0e-12 or l1 <= 1.0e-12:
            return float(r_geo), False, False
        t_minus = (p_cur - p_prev) / l0
        t_plus = (p_next - p_cur) / l1
        cos_phi = float(np.clip(np.dot(t_minus, t_plus), -1.0, 1.0))
        delta_phi = math.acos(cos_phi)
        sin_half = math.sin(0.5 * delta_phi)
        if sin_half <= 1.0e-9:
            return float(r_geo), False, False
        r_curv_floor = 2.0 * (float(v_ref) ** 2) * sin_half / max(self.params.a_lat_max, 1.0e-12)
        if r_geo >= r_curv_floor:
            return float(r_geo), False, False
        # Geometric < curvature floor — try to lift up to floor (capped by r_max).
        if r_curv_floor <= self.params.r_max:
            return float(r_curv_floor), True, False
        return float(self.params.r_max), True, True

    def corner_candidate(self,
                         p_prev: np.ndarray,
                         p_cur: np.ndarray,
                         p_next: np.ndarray,
                         *,
                         corner_index: int,
                         radius: float,
                         sample_ds: float,
                         curvature_lift: bool = False,
                         curvature_overflow: bool = False) -> Optional[BlendSegment]:
        p_prev = np.asarray(p_prev, dtype=np.float64).reshape(2)
        p_cur = np.asarray(p_cur, dtype=np.float64).reshape(2)
        p_next = np.asarray(p_next, dtype=np.float64).reshape(2)
        l_prev = float(np.linalg.norm(p_cur - p_prev))
        l_next = float(np.linalg.norm(p_next - p_cur))
        if l_prev <= 1.0e-9 or l_next <= 1.0e-9 or radius <= 0.0:
            return None

        t_minus = (p_cur - p_prev) / l_prev
        t_plus = (p_next - p_cur) / l_next
        if float(np.dot(t_minus, t_plus)) > 0.999:
            return None  # nearly straight — no blending needed (spec §5.4)

        r = float(min(radius, 0.49 * l_prev, 0.49 * l_next))
        if r < self.params.r_min:
            return None

        a_pt = p_cur - r * t_minus
        b_pt = p_cur + r * t_plus

        chord = float(np.linalg.norm(b_pt - a_pt))
        if chord <= 1.0e-12:
            return None

        # τ-domain BC for position quintic (γ′(0)=ℓ·t⁻, γ′(1)=ℓ·t⁺, γ″(0)=γ″(1)=0)
        u_len = max(chord, r, 1.0e-6)
        v0 = t_minus * u_len
        v1 = t_plus * u_len

        # First, sample uniformly in τ for arclength integration (spec §8.1 R3)
        tau_int = np.linspace(0.0, 1.0, 65, dtype=np.float64)
        xz_int = self._quintic_eval(a_pt, b_pt, v0, v1, tau_int)
        d_int = np.linalg.norm(np.diff(xz_int, axis=0), axis=1)
        s_int = np.concatenate(([0.0], np.cumsum(d_int)))
        L_blend = float(s_int[-1])
        if L_blend <= 1.0e-12:
            return None

        # R3 — arclength-uniform resample with monotone interpolation τ(s)
        n = max(5, int(math.ceil(L_blend / max(sample_ds, 1.0e-4))) + 1)
        s_uniform = np.linspace(0.0, L_blend, n, dtype=np.float64)
        tau_resampled = np.interp(s_uniform, s_int, tau_int)
        xz_resampled = self._quintic_eval(a_pt, b_pt, v0, v1, tau_resampled)

        return BlendSegment(
            kind="blend",
            xz=xz_resampled,
            s_local=s_uniform,
            corner_index=int(corner_index),
            radius=r,
            curvature_lift=bool(curvature_lift),
            curvature_overflow=bool(curvature_overflow),
        )

    def line_segment(self,
                     start_xz: np.ndarray,
                     end_xz: np.ndarray,
                     *,
                     sample_ds: float) -> BlendSegment:
        p0 = np.asarray(start_xz, dtype=np.float64).reshape(2)
        p1 = np.asarray(end_xz, dtype=np.float64).reshape(2)
        length = float(np.linalg.norm(p1 - p0))
        if length <= 1.0e-12:
            xz = p0.reshape(1, 2)
            s_local = np.zeros(1, dtype=np.float64)
        else:
            n = max(2, int(math.ceil(length / max(sample_ds, 1.0e-4))) + 1)
            a = np.linspace(0.0, 1.0, n, dtype=np.float64)
            xz = (1.0 - a[:, None]) * p0 + a[:, None] * p1
            s_local = a * length
        return BlendSegment(kind="line", xz=xz, s_local=s_local)

    @staticmethod
    def _quintic_eval(p0: np.ndarray,
                      p1: np.ndarray,
                      v0: np.ndarray,
                      v1: np.ndarray,
                      tau: np.ndarray) -> np.ndarray:
        # γ(τ) with γ(0)=p₀, γ(1)=p₁, γ′(0)=v₀, γ′(1)=v₁, γ″(0)=γ″(1)=0
        d = p1 - p0
        a3 = 10.0 * d - 6.0 * v0 - 4.0 * v1
        a4 = -15.0 * d + 8.0 * v0 + 7.0 * v1
        a5 = 6.0 * d - 3.0 * v0 - 3.0 * v1
        t = tau[:, None]
        return p0 + v0 * t + a3 * t**3 + a4 * t**4 + a5 * t**5
