#!/usr/bin/env python3
"""Gap-based arclength progress regulator for DLS path following."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class PathFollowingScheduler:
    reference: object
    v_ref: float
    gap_target_m: float
    k_gap_per_s: float
    sdot_max_factor: float
    sdot_floor_factor: float
    accel_time_s: float = 0.30
    decel_time_s: float = 0.30
    s: float = 0.0
    sdot_prev: float = 0.0

    def update(self, ee_pos: np.ndarray, dt: float, hold: bool = False) -> Tuple[float, float, float, float]:
        length = float(getattr(self.reference, "length", 0.0))
        if hold or length <= 0.0:
            self.s = min(max(self.s, 0.0), length)
            self.sdot_prev = 0.0
            sample = self.reference.evaluate(self.s)
            return self.s, 0.0, 0.0, float(np.linalg.norm(sample.pos - ee_pos))

        s_proj, _ = self._project_ee_to_reference(ee_pos)
        s_target = min(length, s_proj + float(self.gap_target_m))
        sdot_max = max(float(self.v_ref) * float(self.sdot_max_factor), 0.0)
        sdot_min = max(float(self.v_ref) * float(self.sdot_floor_factor), 0.0)
        dt_f = max(float(dt), 1.0e-12)
        if self.s >= length - 1.0e-12:
            sdot = 0.0
        else:
            ds_cmd = max(0.0, s_target - self.s)
            if ds_cmd <= 1.0e-12 or sdot_max <= 0.0:
                sdot_des = 0.0
            else:
                sdot_des = min(sdot_max, float(self.k_gap_per_s) * ds_cmd)
                if sdot_des > 0.0 and ds_cmd > sdot_min * dt_f:
                    sdot_des = max(sdot_des, sdot_min)
            accel_time = max(float(self.accel_time_s), dt_f)
            decel_time = max(float(self.decel_time_s), dt_f)
            accel_lim = sdot_max / accel_time if sdot_max > 0.0 else 0.0
            decel_lim = sdot_max / decel_time if sdot_max > 0.0 else 0.0
            lo = max(0.0, self.sdot_prev - decel_lim * dt_f)
            hi = min(sdot_max, self.sdot_prev + accel_lim * dt_f)
            sdot = float(np.clip(sdot_des, lo, hi))
            sdot = float(min(sdot, ds_cmd / dt_f))
        self.sdot_prev = float(sdot)
        self.s = min(length, self.s + sdot * float(dt))
        sample = self.reference.evaluate(self.s)
        gap = float(np.linalg.norm(np.asarray(sample.pos, dtype=np.float64) - np.asarray(ee_pos, dtype=np.float64)))
        lookahead_gap = max(0.0, self.s - s_proj)
        return float(self.s), float(sdot), float(lookahead_gap), float(gap)

    def _project_ee_to_reference(self, ee_pos: np.ndarray) -> Tuple[float, float]:
        ee = np.asarray(ee_pos, dtype=np.float64).reshape(-1)
        if ee.shape[0] < 3:
            return 0.0, float("inf")
        ee_xz = np.array([ee[0], ee[2]], dtype=np.float64)
        try:
            s_nodes, xz_nodes, _ = self.reference.table_arrays()
        except Exception:
            sample = self.reference.evaluate(self.s)
            delta = np.asarray(sample.pos, dtype=np.float64) - ee
            return float(self.s), float(np.linalg.norm(delta))
        s_arr = np.asarray(s_nodes, dtype=np.float64).reshape(-1)
        xz = np.asarray(xz_nodes, dtype=np.float64).reshape(-1, 2)
        if s_arr.size == 0 or xz.shape[0] == 0:
            return 0.0, float("inf")
        if xz.shape[0] == 1:
            return 0.0, float(np.linalg.norm(xz[0] - ee_xz))

        best_s = 0.0
        best_d2 = float("inf")
        for i in range(xz.shape[0] - 1):
            p0 = xz[i]
            p1 = xz[i + 1]
            seg = p1 - p0
            seg2 = float(np.dot(seg, seg))
            if seg2 <= 1.0e-18:
                a = 0.0
            else:
                a = float(np.clip(np.dot(ee_xz - p0, seg) / seg2, 0.0, 1.0))
            proj = p0 + a * seg
            d2 = float(np.dot(ee_xz - proj, ee_xz - proj))
            if d2 < best_d2:
                s0 = float(s_arr[min(i, s_arr.size - 1)])
                s1 = float(s_arr[min(i + 1, s_arr.size - 1)])
                best_s = s0 + a * max(0.0, s1 - s0)
                best_d2 = d2
        length = float(getattr(self.reference, "length", 0.0))
        return float(np.clip(best_s, 0.0, length)), float(best_d2 ** 0.5)
