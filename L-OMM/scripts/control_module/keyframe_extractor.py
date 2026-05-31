#!/usr/bin/env python3
"""Keyframe extractor — direction-change + bounded LOS shortcut (xz only).

Per spec §4 the keyframe set is *xz-only*: we keep direction-change keyframes
plus the two endpoints, then run a bounded LOS Bresenham shortcut over the
feasibility mask.  Pitch keyframes have been removed because pitch is no
longer a smoothing primary state — it is reconstructed deterministically from
the gaze geometry on the smoothed dense xz table (see ``gaze_pitch.py`` and
spec §10).  Removing them eliminates the structural problem where LOS
shortcut would silently strip pitch anchors.
"""
from __future__ import annotations

from typing import List, Sequence

import numpy as np

from constants import Cell
from keyframe_result import KeyframeResult
from smoothing_params import SmoothingParams


class KeyframeExtractor:
    """Compress an A* grid path into geometric (xz-only) keyframes."""

    def __init__(self, params: SmoothingParams | None = None) -> None:
        self.params = params if params is not None else SmoothingParams()

    def extract(self,
                cells: Sequence[Cell],
                xz: np.ndarray,
                feasible_mask: np.ndarray) -> KeyframeResult:
        n = len(cells)
        if n == 0:
            return KeyframeResult([], {"original_waypoint_count": 0})
        if n == 1:
            return KeyframeResult([0], {
                "original_waypoint_count": 1,
                "candidate_keyframe_count": 1,
                "keyframe_count": 1,
                "direction_keyframe_count": 0,
                "los_removed_count": 0,
            })

        candidates = {0, n - 1}

        # 1. integer direction-change keyframes (spec §4.1 K1)
        direction_count = 0
        prev_dir = self._direction(cells[0], cells[1])
        for k in range(1, n - 1):
            cur_dir = self._direction(cells[k], cells[k + 1])
            if cur_dir != prev_dir:
                candidates.add(k)
                direction_count += 1
            prev_dir = cur_dir
        candidates.add(n - 1)

        ordered = sorted(candidates)
        shortcut = self._bounded_los_shortcut(ordered, cells, feasible_mask)
        metrics = {
            "original_waypoint_count": int(n),
            "candidate_keyframe_count": int(len(ordered)),
            "keyframe_count": int(len(shortcut)),
            "direction_keyframe_count": int(direction_count),
            "los_removed_count": int(len(ordered) - len(shortcut)),
        }
        return KeyframeResult(shortcut, metrics)

    @staticmethod
    def _direction(a: Cell, b: Cell) -> Cell:
        dx = int(np.sign(int(b[0]) - int(a[0])))
        dz = int(np.sign(int(b[1]) - int(a[1])))
        return dx, dz

    def _bounded_los_shortcut(self,
                              ordered: List[int],
                              cells: Sequence[Cell],
                              feasible_mask: np.ndarray) -> List[int]:
        if len(ordered) <= 2:
            return ordered
        out = [ordered[0]]
        cur_pos = 0
        while cur_pos < len(ordered) - 1:
            best = cur_pos + 1
            max_pos = min(len(ordered) - 1, cur_pos + self.params.los_lookahead)
            for cand_pos in range(max_pos, cur_pos, -1):
                if self._line_feasible(cells[ordered[cur_pos]], cells[ordered[cand_pos]], feasible_mask):
                    best = cand_pos
                    break
            out.append(ordered[best])
            cur_pos = best
        return out

    def _line_feasible(self, a: Cell, b: Cell, feasible_mask: np.ndarray) -> bool:
        n_x, n_z = feasible_mask.shape
        for ix, iz in self._bresenham(a, b):
            if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z or not feasible_mask[ix, iz]:
                return False
        return True

    @staticmethod
    def _bresenham(a: Cell, b: Cell) -> List[Cell]:
        x0, z0 = int(a[0]), int(a[1])
        x1, z1 = int(b[0]), int(b[1])
        dx = abs(x1 - x0)
        dz = abs(z1 - z0)
        sx = 1 if x0 < x1 else -1
        sz = 1 if z0 < z1 else -1
        err = dx - dz
        points: List[Cell] = []
        while True:
            points.append((x0, z0))
            if x0 == x1 and z0 == z1:
                break
            e2 = 2 * err
            if e2 > -dz:
                err -= dz
                x0 += sx
            if e2 < dx:
                err += dx
                z0 += sz
        return points
