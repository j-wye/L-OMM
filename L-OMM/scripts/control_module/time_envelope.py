#!/usr/bin/env python3
"""R2 — t-domain S-curve velocity envelope ṡ(t) = v_ref · σ(...)."""
from __future__ import annotations

import math
from typing import Tuple


class TimeEnvelope:
    """Quintic smoothstep S-curve envelope for start/cruise/end velocity scheduling.

    Ramp-up uses τ_a(t) = t/T_accel, ramp-down uses τ_d(t) = (T_total - t)/T_decel.
    The smoothstep is σ(τ) = 6τ⁵ - 15τ⁴ + 10τ³ — a C² function with σ(0)=σ̇(0)=σ̈(0)=0
    and σ(1)=1, σ̇(1)=σ̈(1)=0, so the resulting r̈(t) is continuous through the
    transition into and out of cruise.
    """

    def __init__(self,
                 length: float,
                 v_ref: float,
                 t_accel: float,
                 t_decel: float) -> None:
        if length < 0.0 or v_ref <= 0.0 or t_accel < 0.0 or t_decel < 0.0:
            raise ValueError("invalid envelope inputs")
        self.length = float(length)
        self.v_ref = float(v_ref)
        self.t_accel = float(t_accel)
        self.t_decel = float(t_decel)
        self.s_accel, self.s_decel, self.t_cruise, self.total_time = self._schedule()

    def _schedule(self) -> Tuple[float, float, float, float]:
        # Distance covered during a ramp = ∫₀^T v_ref·σ(t/T) dt = v_ref · T · 0.5
        # (since the average of the smoothstep over [0,1] is 0.5)
        s_acc_full = 0.5 * self.v_ref * self.t_accel
        s_dec_full = 0.5 * self.v_ref * self.t_decel
        if s_acc_full + s_dec_full <= self.length + 1.0e-12:
            s_acc = s_acc_full
            s_dec = s_dec_full
            cruise_len = max(0.0, self.length - s_acc - s_dec)
            t_cruise = cruise_len / max(self.v_ref, 1.0e-12)
            total = self.t_accel + t_cruise + self.t_decel
            return s_acc, s_dec, t_cruise, total
        # Triangular profile: shrink ramps proportionally so they fit inside `length`
        if (s_acc_full + s_dec_full) <= 1.0e-12:
            return 0.0, 0.0, self.length / max(self.v_ref, 1.0e-12), self.length / max(self.v_ref, 1.0e-12)
        scale = self.length / (s_acc_full + s_dec_full)
        s_acc = s_acc_full * scale
        s_dec = s_dec_full * scale
        t_acc_eff = self.t_accel * scale
        t_dec_eff = self.t_decel * scale
        return s_acc, s_dec, 0.0, t_acc_eff + t_dec_eff

    @staticmethod
    def smoothstep(x: float) -> float:
        x = max(0.0, min(1.0, float(x)))
        return x * x * x * (10.0 + x * (-15.0 + 6.0 * x))

    @staticmethod
    def smoothstep_integral(x: float) -> float:
        # ∫₀^x σ(u) du = (5/2)x⁴ - 3x⁵ + x⁶
        x = max(0.0, min(1.0, float(x)))
        return 2.5 * x**4 - 3.0 * x**5 + x**6

    def s_at_time(self, t: float) -> float:
        if self.length <= 0.0 or self.total_time <= 0.0:
            return 0.0
        t = max(0.0, float(t))
        if t >= self.total_time:
            return self.length
        # Effective ramp lengths after triangular scaling
        if self.t_cruise <= 0.0 and (self.t_accel + self.t_decel) > 0.0:
            # Triangular profile — we reused t_accel/t_decel scale already in schedule()
            scale = self.length / max(0.5 * self.v_ref * (self.t_accel + self.t_decel), 1.0e-12)
            t_acc_eff = self.t_accel * scale
            t_dec_eff = self.t_decel * scale
        else:
            t_acc_eff = self.t_accel
            t_dec_eff = self.t_decel
        # 1) accel
        if t < t_acc_eff:
            tau = t / max(t_acc_eff, 1.0e-12)
            return self.v_ref * t_acc_eff * self.smoothstep_integral(tau)
        # 2) cruise
        t1 = t_acc_eff
        if t < t1 + self.t_cruise:
            return self.s_accel + self.v_ref * (t - t1)
        # 3) decel
        t2 = t1 + self.t_cruise
        rem = t - t2
        if t_dec_eff <= 1.0e-12:
            return self.length
        tau_dec = rem / t_dec_eff
        # ∫₀^rem v_ref·σ((t_dec - u)/t_dec) du = v_ref·t_dec·(τ - I(1-τ) + I(1)) — simpler:
        # = v_ref · t_dec · ∫₀^τ σ(1-u) du = v_ref · t_dec · (τ - smoothstep_integral(1) + smoothstep_integral(1-τ))
        # but smoothstep_integral(1) = 0.5
        s_decel_so_far = self.v_ref * t_dec_eff * (tau_dec - 0.5 + self.smoothstep_integral(1.0 - tau_dec))
        return min(self.length, self.s_accel + self.v_ref * self.t_cruise + s_decel_so_far)

    def sdot_at_time(self, t: float) -> float:
        if self.length <= 0.0 or self.total_time <= 0.0:
            return 0.0
        t = max(0.0, float(t))
        if t >= self.total_time:
            return 0.0
        if self.t_cruise <= 0.0 and (self.t_accel + self.t_decel) > 0.0:
            scale = self.length / max(0.5 * self.v_ref * (self.t_accel + self.t_decel), 1.0e-12)
            t_acc_eff = self.t_accel * scale
            t_dec_eff = self.t_decel * scale
        else:
            t_acc_eff = self.t_accel
            t_dec_eff = self.t_decel
        if t < t_acc_eff:
            return self.v_ref * self.smoothstep(t / max(t_acc_eff, 1.0e-12))
        t1 = t_acc_eff
        if t < t1 + self.t_cruise:
            return self.v_ref
        t2 = t1 + self.t_cruise
        rem = t - t2
        if t_dec_eff <= 1.0e-12:
            return 0.0
        return self.v_ref * self.smoothstep((t_dec_eff - rem) / t_dec_eff)
