#!/usr/bin/env python3
"""Reference-smoother hyperparameters (spec §4–§10).

Per the deterministic-pitch architecture (spec §10), pitch is *not* a
smoothing primary state — it is reconstructed deterministically from gaze
geometry on the smoothed dense xz table.  The two ``pitch_*_threshold_rad``
fields below are kept for backward-compatibility but are *ignored* by the
KeyframeExtractor.  Only ``pitch_slope_limit`` remains active, and it acts
as a *runtime feedforward guard* (spec §10.5), not as a smoother input.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SmoothingParams:
    # The A* skeleton is the experimental object.  Keep LOS shortcut disabled
    # by default so reference generation does not erase policy-dependent
    # corridor choices before DLS evaluation.
    los_lookahead: int = 1

    # Deprecated — see module docstring.  Kept so older instantiations keep
    # constructing without raising.  Not consulted by KeyframeExtractor.
    pitch_step_threshold_rad: float = math.radians(5.0)
    pitch_acc_threshold_rad: float = math.radians(10.0)

    # Geometric corner radius ceilings (spec §6).
    r_max: float = 0.04
    r_min: float = 0.006
    radius_alpha: float = 0.25
    radius_shrink: float = 0.5

    # Curvature-bounded radius *floor* (spec §6.1, monitored at nominal v_ref).
    a_lat_max: float = 4.0     # m/s^2
    enable_curvature_radius: bool = True

    # Sample density for feasibility validation (spec §7.1).
    validation_ds: float = 0.005

    # Runtime feedforward guard for the gaze-derived dpitch/ds (spec §10.5).
    # The previous default 6 rad/m was sized for the legacy quintic-pitch
    # path that only had to interpolate sparse keyframes.  With the
    # deterministic atan2 reconstruction the slope follows true gaze
    # geometry and can legitimately reach ~7–8 rad/m near the gaze
    # target.  At v_ref = 0.04 m/s, 30 rad/m maps to a joint5 feedforward
    # of 1.2 rad/s, still well below the joint5 velocity limit (±2 rad/s).
    pitch_slope_limit: float = 30.0

    # R3 — number of Gauss-Legendre nodes for blend arclength integration.
    arclen_quad_nodes: int = 16

    def __post_init__(self) -> None:
        if self.los_lookahead < 1:
            raise ValueError("los_lookahead must be >= 1")
        if not (0.0 < self.r_min <= self.r_max):
            raise ValueError("require 0 < r_min <= r_max")
        if not (0.0 < self.radius_alpha <= 0.5):
            raise ValueError("radius_alpha should be in (0, 0.5]")
        if not (0.0 < self.radius_shrink < 1.0):
            raise ValueError("radius_shrink must be in (0, 1)")
        if self.validation_ds <= 0.0 or self.pitch_slope_limit <= 0.0:
            raise ValueError("validation_ds and pitch_slope_limit must be positive")
        if self.a_lat_max <= 0.0:
            raise ValueError("a_lat_max must be positive")
        if self.arclen_quad_nodes < 4:
            raise ValueError("arclen_quad_nodes must be >= 4")
