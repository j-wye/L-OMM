#!/usr/bin/env python3
"""Dry-run classifier for snapshot-induced replanning events."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from collections.abc import Mapping
from typing import Any, Dict, Optional, Tuple

import numpy as np

from .active_map_snapshot import ActiveMapSnapshot
from .map_update_layer import MapUpdateLayer
from .reference_snapshot_diff import DiffReport
from .runtime_contract import RuntimeEventClass


@dataclass(frozen=True)
class BlockageReport:
    """Classification output for one synthetic frame."""

    event_class: str
    replanning_triggered: bool
    reason: str
    current_s_m: float
    current_pose_blocked_count: int
    reference_blocked_count: int
    goal_blocked_count: int
    mask_changed_cell_count: int
    lookahead_start_s_m: float
    lookahead_end_s_m: float
    goal_window_start_s_m: float
    target_changed_event: bool = False
    goal_feasible_neighborhood_count: int = 0
    goal_connectivity_ok: bool = True
    reference_diff_enabled: bool = False
    new_obstacle_cells: int = 0
    vanished_obstacle_cells: int = 0
    fov_observed_cells: int = 0
    fov_coverage_ratio: float = 0.0
    largest_new_cluster_size: int = 0
    largest_reference_corridor_cluster_size: int = 0
    new_cells_in_reference_corridor: int = 0
    new_cells_in_current_capsule: int = 0
    new_cells_in_goal_window: int = 0
    is_corridor_violated: bool = False
    path_collision_monitor_enabled: bool = False
    path_collision_is_collision: bool = False
    path_collision_colliding_cells_count: int = 0
    path_collision_checked_sample_count: int = 0
    reference_blocked_source: str = "none"
    reference_future_slice_fallback: bool = False

    def as_dict(self) -> Dict[str, object]:
        return {
            "event_class": self.event_class,
            "replanning_triggered": bool(self.replanning_triggered),
            "reason": self.reason,
            "current_s_m": float(self.current_s_m),
            "current_pose_blocked_count": int(self.current_pose_blocked_count),
            "reference_blocked_count": int(self.reference_blocked_count),
            "goal_blocked_count": int(self.goal_blocked_count),
            "mask_changed_cell_count": int(self.mask_changed_cell_count),
            "lookahead_start_s_m": float(self.lookahead_start_s_m),
            "lookahead_end_s_m": float(self.lookahead_end_s_m),
            "goal_window_start_s_m": float(self.goal_window_start_s_m),
            "target_changed_event": bool(self.target_changed_event),
            "goal_feasible_neighborhood_count": int(self.goal_feasible_neighborhood_count),
            "goal_connectivity_ok": bool(self.goal_connectivity_ok),
            "reference_diff_enabled": bool(self.reference_diff_enabled),
            "new_obstacle_cells": int(self.new_obstacle_cells),
            "vanished_obstacle_cells": int(self.vanished_obstacle_cells),
            "fov_observed_cells": int(self.fov_observed_cells),
            "fov_coverage_ratio": float(self.fov_coverage_ratio),
            "largest_new_cluster_size": int(self.largest_new_cluster_size),
            "largest_reference_corridor_cluster_size": int(self.largest_reference_corridor_cluster_size),
            "new_cells_in_reference_corridor": int(self.new_cells_in_reference_corridor),
            "new_cells_in_current_capsule": int(self.new_cells_in_current_capsule),
            "new_cells_in_goal_window": int(self.new_cells_in_goal_window),
            "is_corridor_violated": bool(self.is_corridor_violated),
            "path_collision_monitor_enabled": bool(self.path_collision_monitor_enabled),
            "path_collision_is_collision": bool(self.path_collision_is_collision),
            "path_collision_colliding_cells_count": int(self.path_collision_colliding_cells_count),
            "path_collision_checked_sample_count": int(self.path_collision_checked_sample_count),
            "reference_blocked_source": self.reference_blocked_source,
            "reference_future_slice_fallback": bool(self.reference_future_slice_fallback),
        }


class ReferenceBlockageClassifier:
    """Classify whether a new snapshot invalidates the current reference.

    This is not a full BT supervisor.  It is a deterministic dry-run classifier
    used to validate replanning boundaries before live camera integration.
    """

    CURRENT_POSE_UNSAFE = RuntimeEventClass.CURRENT_POSE_UNSAFE
    TARGET_CHANGED = RuntimeEventClass.TARGET_CHANGED
    GOAL_CORRIDOR_BLOCKED = RuntimeEventClass.GOAL_CORRIDOR_BLOCKED
    REFERENCE_BLOCKED = RuntimeEventClass.REFERENCE_BLOCKED
    MASK_CHANGED_NONCRITICAL = RuntimeEventClass.MASK_CHANGED_NONCRITICAL
    NO_RELEVANT_CHANGE = RuntimeEventClass.NO_RELEVANT_CHANGE

    def __init__(
        self,
        *,
        lookahead_m: float = 0.14,
        goal_window_m: float = 0.08,
        corridor_radius_m: float = 0.015,
        current_pose_radius_m: float = 0.015,
        path_collision_monitor: object | None = None,
    ) -> None:
        self.lookahead_m = float(lookahead_m)
        self.goal_window_m = float(goal_window_m)
        self.corridor_radius_m = float(corridor_radius_m)
        self.current_pose_radius_m = float(current_pose_radius_m)
        self.path_collision_monitor = path_collision_monitor

    def classify(
        self,
        snapshot: ActiveMapSnapshot,
        *,
        s_table: np.ndarray,
        xz_table: np.ndarray,
        current_s_m: float,
        previous_blocked_mask: Optional[np.ndarray] = None,
        target_changed_event: bool = False,
        capsule_proxy_xz: Optional[np.ndarray] = None,
        goal_xz: Optional[Tuple[float, float]] = None,
        reference_snapshot_diff: Optional[DiffReport] = None,
        accepted_reference: object | None = None,
    ) -> BlockageReport:
        s_arr = np.asarray(s_table, dtype=np.float64).reshape(-1)
        xz_arr = np.asarray(xz_table, dtype=np.float64).reshape(-1, 2)
        if s_arr.size != xz_arr.shape[0] or s_arr.size == 0:
            return self._report(
                self.CURRENT_POSE_UNSAFE,
                True,
                "invalid_reference_table",
                0.0,
                1,
                0,
                0,
                self._mask_changed(snapshot.blocked_mask, previous_blocked_mask),
                0.0,
                0.0,
                0.0,
                target_changed_event,
                diff_report=reference_snapshot_diff,
            )

        length = float(s_arr[-1])
        current_s = float(np.clip(current_s_m, 0.0, max(length, 0.0)))
        lookahead_end = min(length, current_s + max(self.lookahead_m, 0.0))
        goal_start = max(0.0, length - max(self.goal_window_m, 0.0))

        current_pts = self._sample_range(s_arr, xz_arr, current_s, current_s, include_nearest=True)
        ref_pts = self._sample_range(s_arr, xz_arr, current_s, lookahead_end, include_nearest=True)
        goal_pts = self._sample_range(s_arr, xz_arr, goal_start, length, include_nearest=True)
        if capsule_proxy_xz is not None:
            capsule_pts = np.asarray(capsule_proxy_xz, dtype=np.float64).reshape(-1, 2)
            current_check_pts = np.vstack([current_pts, capsule_pts]) if current_pts.size else capsule_pts
        else:
            current_check_pts = current_pts

        current_blocked_count = self._blocked_count(snapshot, current_check_pts, self.current_pose_radius_m)
        raw_reference_blocked_count = self._blocked_count(snapshot, ref_pts, self.corridor_radius_m)
        reference_blocked_count = 0
        reference_blocked_source = "none"
        reference_future_slice_fallback = False
        path_collision_report = None
        path_collision_succeeded = False
        if self.path_collision_monitor is not None and accepted_reference is not None:
            future_reference, reference_future_slice_fallback = self._future_reference_for_collision(
                accepted_reference,
                s_arr=s_arr,
                xz_arr=xz_arr,
                current_s=current_s,
            )
            try:
                path_collision_report = self.path_collision_monitor.check(future_reference, snapshot.blocked_mask)
                path_collision_succeeded = True
            except NotImplementedError:
                path_collision_report = None
            if path_collision_report is not None and bool(getattr(path_collision_report, "is_collision", False)):
                reference_blocked_count = int(getattr(path_collision_report, "colliding_cells_count", 1))
                reference_blocked_source = "path_collision_monitor"
        goal_blocked_count = self._blocked_count(snapshot, goal_pts, self.corridor_radius_m)
        goal_status = self._goal_status(
            snapshot,
            current_pts=current_pts,
            goal_xz=np.asarray(goal_xz if goal_xz is not None else xz_arr[-1], dtype=np.float64),
        )
        if not goal_status["goal_connectivity_ok"] or goal_status["goal_feasible_neighborhood_count"] <= 0:
            goal_blocked_count = max(goal_blocked_count, 1)
        if reference_snapshot_diff is not None:
            mask_changed = int(
                reference_snapshot_diff.new_obstacle_cells
                + reference_snapshot_diff.vanished_obstacle_cells
            )
            if path_collision_report is None or not bool(getattr(path_collision_report, "is_collision", False)):
                reference_blocked_count = (
                    int(reference_snapshot_diff.new_cells_in_reference_corridor)
                    if bool(reference_snapshot_diff.is_corridor_violated)
                    else 0
                )
                reference_blocked_source = "reference_snapshot_diff" if reference_blocked_count > 0 else "none"
        else:
            mask_changed = self._mask_changed(snapshot.blocked_mask, previous_blocked_mask)
            if reference_blocked_source == "none" and not path_collision_succeeded and raw_reference_blocked_count > 0:
                reference_blocked_count = raw_reference_blocked_count
                reference_blocked_source = "corridor_radius_fallback"

        if current_blocked_count > 0:
            return self._report(
                self.CURRENT_POSE_UNSAFE,
                True,
                "current_pose_intersects_blocked_mask",
                current_s,
                current_blocked_count,
                reference_blocked_count,
                goal_blocked_count,
                mask_changed,
                current_s,
                lookahead_end,
                goal_start,
                target_changed_event,
                goal_status,
                diff_report=reference_snapshot_diff,
                path_collision_report=path_collision_report,
                reference_blocked_source=reference_blocked_source,
                reference_future_slice_fallback=reference_future_slice_fallback,
            )
        if target_changed_event:
            return self._report(
                self.TARGET_CHANGED,
                True,
                "explicit_synthetic_target_changed_event",
                current_s,
                current_blocked_count,
                reference_blocked_count,
                goal_blocked_count,
                mask_changed,
                current_s,
                lookahead_end,
                goal_start,
                target_changed_event,
                goal_status,
                diff_report=reference_snapshot_diff,
                path_collision_report=path_collision_report,
                reference_blocked_source=reference_blocked_source,
                reference_future_slice_fallback=reference_future_slice_fallback,
            )
        if goal_blocked_count > 0:
            return self._report(
                self.GOAL_CORRIDOR_BLOCKED,
                True,
                "goal_corridor_intersects_blocked_mask",
                current_s,
                current_blocked_count,
                reference_blocked_count,
                goal_blocked_count,
                mask_changed,
                current_s,
                lookahead_end,
                goal_start,
                target_changed_event,
                goal_status,
                diff_report=reference_snapshot_diff,
                path_collision_report=path_collision_report,
                reference_blocked_source=reference_blocked_source,
                reference_future_slice_fallback=reference_future_slice_fallback,
            )
        if reference_blocked_count > 0:
            reason = (
                "path_collision_monitor_current_blocked_mask_collision"
                if reference_blocked_source == "path_collision_monitor"
                else "reference_snapshot_diff_new_obstacle_in_corridor"
                if reference_blocked_source == "reference_snapshot_diff"
                else "lookahead_reference_intersects_blocked_mask"
            )
            return self._report(
                self.REFERENCE_BLOCKED,
                True,
                reason,
                current_s,
                current_blocked_count,
                reference_blocked_count,
                goal_blocked_count,
                mask_changed,
                current_s,
                lookahead_end,
                goal_start,
                target_changed_event,
                goal_status,
                diff_report=reference_snapshot_diff,
                path_collision_report=path_collision_report,
                reference_blocked_source=reference_blocked_source,
                reference_future_slice_fallback=reference_future_slice_fallback,
            )
        if mask_changed > 0:
            return self._report(
                self.MASK_CHANGED_NONCRITICAL,
                False,
                "blocked_mask_changed_but_not_current_reference_or_goal",
                current_s,
                current_blocked_count,
                reference_blocked_count,
                goal_blocked_count,
                mask_changed,
                current_s,
                lookahead_end,
                goal_start,
                target_changed_event,
                goal_status,
                diff_report=reference_snapshot_diff,
                path_collision_report=path_collision_report,
                reference_blocked_source=reference_blocked_source,
                reference_future_slice_fallback=reference_future_slice_fallback,
            )
        return self._report(
            self.NO_RELEVANT_CHANGE,
            False,
            "blocked_mask_unchanged_and_reference_clear",
            current_s,
            current_blocked_count,
            reference_blocked_count,
            goal_blocked_count,
            mask_changed,
            current_s,
            lookahead_end,
            goal_start,
            target_changed_event,
            goal_status,
            diff_report=reference_snapshot_diff,
            path_collision_report=path_collision_report,
            reference_blocked_source=reference_blocked_source,
            reference_future_slice_fallback=reference_future_slice_fallback,
        )

    @staticmethod
    def _mask_changed(mask: np.ndarray, previous: Optional[np.ndarray]) -> int:
        if previous is None:
            return 0
        prev = np.asarray(previous, dtype=bool)
        cur = np.asarray(mask, dtype=bool)
        if prev.shape != cur.shape:
            return int(cur.size)
        return int(np.count_nonzero(np.logical_xor(prev, cur)))

    def _future_reference_for_collision(
        self,
        accepted_reference: object,
        *,
        s_arr: np.ndarray,
        xz_arr: np.ndarray,
        current_s: float,
    ) -> tuple[object, bool]:
        """Return the causal-future portion of an accepted reference.

        The classifier owns progress semantics.  PathCollisionMonitor remains a
        geometry checker and receives only the future commitment whenever the
        accepted reference can be aligned with the arclength table.
        """

        s = np.asarray(s_arr, dtype=np.float64).reshape(-1)
        xz_table = np.asarray(xz_arr, dtype=np.float64).reshape(-1, 2)
        if s.size == 0 or xz_table.shape[0] != s.size:
            return accepted_reference, True

        future_mask = s >= float(current_s) - 1.0e-12
        if not np.any(future_mask):
            future_mask = np.zeros_like(s, dtype=bool)
            future_mask[-1] = True

        if isinstance(accepted_reference, Mapping):
            source: dict[str, Any] = dict(accepted_reference)
            return self._slice_reference_mapping(source, s, xz_table, future_mask, accepted_reference)

        source = {}
        for name in ("xz_path", "xz", "points_xz", "reference_xz", "q_traj", "q_samples", "q_active_samples"):
            if hasattr(accepted_reference, name):
                source[name] = getattr(accepted_reference, name)
        if not source:
            return accepted_reference, True
        return self._slice_reference_mapping(source, s, xz_table, future_mask, accepted_reference)

    @staticmethod
    def _slice_reference_mapping(
        source: dict[str, Any],
        s: np.ndarray,
        xz_table: np.ndarray,
        future_mask: np.ndarray,
        fallback_reference: object,
    ) -> tuple[object, bool]:
        expected_len = int(s.size)
        names = ("xz_path", "xz", "points_xz", "reference_xz", "q_traj", "q_samples", "q_active_samples")
        for name in names:
            if name not in source:
                continue
            arr = np.asarray(source[name])
            if arr.size == 0:
                continue
            try:
                n_rows = int(arr.reshape(-1, 2).shape[0]) if name in {"xz_path", "xz", "points_xz", "reference_xz"} else int(arr.reshape(-1, 3).shape[0])
            except ValueError:
                return fallback_reference, True
            if n_rows != expected_len:
                return fallback_reference, True

        sliced = dict(source)
        xz_key = next((name for name in ("xz_path", "xz", "points_xz", "reference_xz") if name in source), None)
        if xz_key is not None:
            sliced[xz_key] = np.asarray(source[xz_key], dtype=np.float64).reshape(-1, 2)[future_mask]
        else:
            sliced["xz_path"] = xz_table[future_mask]
        for name in ("q_traj", "q_samples", "q_active_samples"):
            if name in source:
                sliced[name] = np.asarray(source[name], dtype=np.float64).reshape(-1, 3)[future_mask]
        return sliced, False

    def _blocked_count(self, snapshot: ActiveMapSnapshot, xz: np.ndarray, radius_m: float) -> int:
        if xz.size == 0:
            return 0
        pts = self._expand_points(xz, radius_m)
        return int(self._blocked_count_inside_map(snapshot, pts))

    @staticmethod
    def _blocked_count_inside_map(snapshot: ActiveMapSnapshot, xz: np.ndarray) -> int:
        """Count blocked in-map samples.

        The classifier asks whether the *new snapshot* invalidates the current
        reference.  A small radius around a start/goal point can extend outside
        the finite stored grid; that boundary artifact must not be interpreted
        as a dynamic obstacle.  The planner still owns start/goal projection and
        active-map feasibility.  Here we only count samples that project to an
        in-map blocked cell.
        """
        pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        handle = snapshot.handle
        mask = np.asarray(snapshot.blocked_mask, dtype=bool)
        n_x, n_z = mask.shape
        blocked_cells = set()
        for x, z in pts:
            ix = int(round((float(x) - handle.x0) / handle.resolution_m))
            iz = int(round((float(z) - handle.z0) / handle.resolution_m))
            if 0 <= ix < n_x and 0 <= iz < n_z and bool(mask[ix, iz]):
                blocked_cells.add((ix, iz))
        return len(blocked_cells)

    def _goal_status(
        self,
        snapshot: ActiveMapSnapshot,
        *,
        current_pts: np.ndarray,
        goal_xz: np.ndarray,
    ) -> Dict[str, object]:
        final = np.asarray(snapshot.final_active_mask, dtype=bool)
        handle = snapshot.handle
        goal_cell = self._cell_from_xz(handle, np.asarray(goal_xz, dtype=np.float64).reshape(2))
        if goal_cell is None:
            return {"goal_feasible_neighborhood_count": 0, "goal_connectivity_ok": False}
        radius = max(2.0 * float(handle.resolution_m), float(self.corridor_radius_m))
        goal_cells = self._neighborhood_cells(handle, goal_cell, radius)
        feasible_goal_cells = [(ix, iz) for ix, iz in goal_cells if bool(final[ix, iz])]
        if not feasible_goal_cells:
            return {"goal_feasible_neighborhood_count": 0, "goal_connectivity_ok": False}
        if current_pts.size == 0:
            return {
                "goal_feasible_neighborhood_count": len(feasible_goal_cells),
                "goal_connectivity_ok": False,
            }
        current_cell = self._cell_from_xz(handle, np.asarray(current_pts[0], dtype=np.float64).reshape(2))
        if current_cell is None or not bool(final[current_cell]):
            return {
                "goal_feasible_neighborhood_count": len(feasible_goal_cells),
                "goal_connectivity_ok": False,
            }
        connected = self._connected_to_any_goal(final, current_cell, set(feasible_goal_cells))
        return {
            "goal_feasible_neighborhood_count": len(feasible_goal_cells),
            "goal_connectivity_ok": bool(connected),
        }

    @staticmethod
    def _cell_from_xz(handle, xz: np.ndarray) -> Optional[Tuple[int, int]]:
        ix = int(round((float(xz[0]) - handle.x0) / handle.resolution_m))
        iz = int(round((float(xz[1]) - handle.z0) / handle.resolution_m))
        n_x, n_z = handle.shape
        if ix < 0 or ix >= n_x or iz < 0 or iz >= n_z:
            return None
        return ix, iz

    @staticmethod
    def _neighborhood_cells(handle, center: Tuple[int, int], radius_m: float) -> list[Tuple[int, int]]:
        n_x, n_z = handle.shape
        radius_cells = int(np.ceil(max(float(radius_m), 0.0) / max(float(handle.resolution_m), 1.0e-12)))
        cx, cz = center
        cells = []
        for ix in range(max(0, cx - radius_cells), min(n_x, cx + radius_cells + 1)):
            for iz in range(max(0, cz - radius_cells), min(n_z, cz + radius_cells + 1)):
                dx = (ix - cx) * float(handle.resolution_m)
                dz = (iz - cz) * float(handle.resolution_m)
                if dx * dx + dz * dz <= radius_m * radius_m + 1.0e-12:
                    cells.append((ix, iz))
        return cells

    @staticmethod
    def _connected_to_any_goal(
        final_mask: np.ndarray,
        start: Tuple[int, int],
        goal_cells: set[Tuple[int, int]],
    ) -> bool:
        if start in goal_cells:
            return True
        n_x, n_z = final_mask.shape
        q = deque([start])
        visited = {start}
        neighbors = (
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        )
        while q:
            ix, iz = q.popleft()
            for dx, dz in neighbors:
                nx, nz = ix + dx, iz + dz
                cell = (nx, nz)
                if nx < 0 or nx >= n_x or nz < 0 or nz >= n_z:
                    continue
                if cell in visited or not bool(final_mask[nx, nz]):
                    continue
                if cell in goal_cells:
                    return True
                visited.add(cell)
                q.append(cell)
        return False

    @staticmethod
    def _sample_range(
        s_arr: np.ndarray,
        xz_arr: np.ndarray,
        s0: float,
        s1: float,
        *,
        include_nearest: bool,
    ) -> np.ndarray:
        if xz_arr.shape[0] == 0:
            return np.zeros((0, 2), dtype=np.float64)
        lo = min(float(s0), float(s1))
        hi = max(float(s0), float(s1))
        mask = (s_arr >= lo - 1.0e-12) & (s_arr <= hi + 1.0e-12)
        pts = xz_arr[mask]
        if pts.size == 0 and include_nearest:
            idx = int(np.argmin(np.abs(s_arr - 0.5 * (lo + hi))))
            pts = xz_arr[idx:idx + 1]
        return np.asarray(pts, dtype=np.float64).reshape(-1, 2)

    @staticmethod
    def _expand_points(xz: np.ndarray, radius_m: float) -> np.ndarray:
        pts = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        r = max(float(radius_m), 0.0)
        if r <= 1.0e-12:
            return pts
        offsets = np.array(
            [
                [0.0, 0.0],
                [r, 0.0],
                [-r, 0.0],
                [0.0, r],
                [0.0, -r],
                [0.70710678 * r, 0.70710678 * r],
                [0.70710678 * r, -0.70710678 * r],
                [-0.70710678 * r, 0.70710678 * r],
                [-0.70710678 * r, -0.70710678 * r],
            ],
            dtype=np.float64,
        )
        return (pts[:, None, :] + offsets[None, :, :]).reshape(-1, 2)

    @staticmethod
    def _report(
        event_class: str,
        triggered: bool,
        reason: str,
        current_s: float,
        current_count: int,
        reference_count: int,
        goal_count: int,
        mask_changed: int,
        lookahead_start: float,
        lookahead_end: float,
        goal_start: float,
        target_changed: bool,
        goal_status: Optional[Dict[str, object]] = None,
        diff_report: Optional[DiffReport] = None,
        path_collision_report: object | None = None,
        reference_blocked_source: str = "none",
        reference_future_slice_fallback: bool = False,
    ) -> BlockageReport:
        status = goal_status or {}
        diff = diff_report.as_dict() if diff_report is not None else {}
        path_collision = (
            path_collision_report.as_dict()
            if path_collision_report is not None and hasattr(path_collision_report, "as_dict")
            else {}
        )
        return BlockageReport(
            event_class=str(event_class),
            replanning_triggered=bool(triggered),
            reason=str(reason),
            current_s_m=float(current_s),
            current_pose_blocked_count=int(current_count),
            reference_blocked_count=int(reference_count),
            goal_blocked_count=int(goal_count),
            mask_changed_cell_count=int(mask_changed),
            lookahead_start_s_m=float(lookahead_start),
            lookahead_end_s_m=float(lookahead_end),
            goal_window_start_s_m=float(goal_start),
            target_changed_event=bool(target_changed),
            goal_feasible_neighborhood_count=int(status.get("goal_feasible_neighborhood_count", 0)),
            goal_connectivity_ok=bool(status.get("goal_connectivity_ok", True)),
            reference_diff_enabled=bool(diff.get("reference_diff_enabled", False)),
            new_obstacle_cells=int(diff.get("new_obstacle_cells", 0)),
            vanished_obstacle_cells=int(diff.get("vanished_obstacle_cells", 0)),
            fov_observed_cells=int(diff.get("fov_observed_cells", 0)),
            fov_coverage_ratio=float(diff.get("fov_coverage_ratio", 0.0)),
            largest_new_cluster_size=int(diff.get("largest_new_cluster_size", 0)),
            largest_reference_corridor_cluster_size=int(diff.get("largest_reference_corridor_cluster_size", 0)),
            new_cells_in_reference_corridor=int(diff.get("new_cells_in_reference_corridor", 0)),
            new_cells_in_current_capsule=int(diff.get("new_cells_in_current_capsule", 0)),
            new_cells_in_goal_window=int(diff.get("new_cells_in_goal_window", 0)),
            is_corridor_violated=bool(diff.get("is_corridor_violated", False)),
            path_collision_monitor_enabled=bool(path_collision_report is not None),
            path_collision_is_collision=bool(path_collision.get("path_collision_is_collision", False)),
            path_collision_colliding_cells_count=int(path_collision.get("path_collision_colliding_cells_count", 0)),
            path_collision_checked_sample_count=int(path_collision.get("path_collision_checked_sample_count", 0)),
            reference_blocked_source=str(reference_blocked_source),
            reference_future_slice_fallback=bool(reference_future_slice_fallback),
        )
