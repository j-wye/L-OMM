#!/usr/bin/env python3
"""Distance-only A* planner policy over a reduced active-manifold map."""
from __future__ import annotations

import heapq
import json
import math
import os
import time
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from capsule_collision import CapsuleCollision
from constants import DEFAULT_TAG, X_EE_GOAL, Cell
from kinematics import Kinematics
from map_handle import MapHandle
from plan_request import PlanRequest
from plan_result import PlanResult


class DistanceAStarPlanner:
    """Stateless distance-only active-map A* planner policy."""

    NEIGHBORS: Tuple[Tuple[int, int, float], ...] = (
        (-1,  0, 1.0), (1,  0, 1.0), (0, -1, 1.0), (0,  1, 1.0),
        (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),  (1, 1, math.sqrt(2.0)),
    )

    def __init__(self, map_path: Optional[str] = None) -> None:
        module_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.abspath(os.path.join(module_dir, "..", "..", ".."))
        self.default_map_path = map_path or os.path.join(
            project_root, "map", "map_test", "10mm", f"{DEFAULT_TAG}.npy"
        )
        self._kin = Kinematics()
        self._capsules = CapsuleCollision()
        self._handle_cache: Dict[str, MapHandle] = {}

    def load_map(self, map_path: Optional[str] = None) -> MapHandle:
        path = os.path.abspath(map_path or self.default_map_path)
        if path in self._handle_cache:
            return self._handle_cache[path]
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        tag, meta_name, q_name = self._sidecar_names(path)
        meta_path = os.path.join(os.path.dirname(path), meta_name)
        if not os.path.exists(meta_path):
            raise FileNotFoundError(meta_path)
        data = np.load(path)
        if data.ndim != 3 or data.shape[2] < 2:
            raise ValueError(f"expected active map shape (n_x, n_z, >=2), got {data.shape}")
        with open(meta_path, "r", encoding="utf-8") as f:
            meta: Dict[str, Any] = json.load(f)
        q_path = os.path.join(os.path.dirname(path), q_name)
        q_grid = np.load(q_path).astype(np.float64) if os.path.exists(q_path) else None
        handle = MapHandle(
            map_path=path,
            meta_path=meta_path,
            mu_grid=np.asarray(data[:, :, 0], dtype=np.float64),
            pitch_grid=np.asarray(data[:, :, 1], dtype=np.float64),
            q_grid=q_grid,
            meta=meta,
            resolution_m=float(meta["resolution_m"]),
            x0=float(meta["x_range"][0]),
            z0=float(meta["z_range"][0]),
            target_y=float(meta.get("target_y_fixed_m", meta.get("target_y", 0.0))),
            tag=tag,
        )
        self._handle_cache[path] = handle
        return handle

    @staticmethod
    def _sidecar_names(path: str) -> Tuple[str, str, str]:
        base = os.path.basename(path)
        if not base.endswith(".npy"):
            raise ValueError(f"map file must be a .npy file, got {base}")
        stem = base[:-len(".npy")]
        parts = stem.split(".")
        if (
            len(parts) != 2
            or len(parts[1]) != 2
            or not parts[0].isdigit()
            or not parts[1].isdigit()
        ):
            raise ValueError(f"map file must be named like 0.50.npy, got {base}")
        return stem, f"{stem}.json", f"q_{stem}.npy"

    def plan(self, request: PlanRequest) -> PlanResult:
        snapshot = request.map_snapshot
        handle = snapshot.handle
        n_x, n_z = handle.shape
        obs = snapshot.blocked_mask
        base_active = snapshot.final_active_mask
        feasible = base_active
        arm_volume_stats: Dict[str, float] = {}
        if np.any(obs):
            feasible, arm_volume_stats = self._capsules.prune_feasible_mask(handle, feasible, obs)
        clear_feasible = snapshot.base_feasible_mask

        start_req, goal_req = self._default_start_goal(handle, request.start_xz, request.goal_xz)
        start_raw = self.phys_to_cell(handle, *start_req)
        goal_raw = self.phys_to_cell(handle, *goal_req)
        projection_tolerance = max(float(request.goal_projection_factor) * handle.resolution_m, 1.0e-9)
        try:
            start_cell = self.nearest_feasible_cell(feasible, start_raw)
            goal_cell = self.nearest_feasible_cell(feasible, goal_raw)
            start_xz = self.cell_to_phys(handle, *start_cell)
            goal_xz = self.cell_to_phys(handle, *goal_cell)
            start_projection_dist = self._xz_dist(start_req, start_xz)
            goal_projection_dist = self._xz_dist(goal_req, goal_xz)
            start_projection_ok = start_projection_dist <= projection_tolerance
            goal_projection_ok = goal_projection_dist <= projection_tolerance
            projection_ok = start_projection_ok and goal_projection_ok
        except RuntimeError:
            start_cell = start_raw
            goal_cell = goal_raw
            start_xz = self.cell_to_phys(handle, *start_cell)
            goal_xz = self.cell_to_phys(handle, *goal_cell)
            start_projection_dist = float("inf")
            goal_projection_dist = float("inf")
            start_projection_ok = False
            goal_projection_ok = False
            projection_ok = False

        valid_grasp = bool(projection_ok)
        invalid_reason = ""
        invalid_reason_code = ""
        if not start_projection_ok:
            invalid_reason_code = self._projection_failure_code("start", start_raw, base_active, feasible)
            invalid_reason = (
                f"start_projected_by_{start_projection_dist * 1000.0:.2f}mm_"
                f"exceeds_tol_{projection_tolerance * 1000.0:.2f}mm"
            )
        if not goal_projection_ok:
            invalid_reason_code = self._projection_failure_code("goal", goal_raw, base_active, feasible)
            invalid_reason = (
                f"goal_projected_by_{goal_projection_dist * 1000.0:.2f}mm_"
                f"exceeds_tol_{projection_tolerance * 1000.0:.2f}mm"
            )

        if projection_ok:
            cells, cost, expanded, time_ms = self._astar(feasible, start_cell, goal_cell, handle.resolution_m)
            if not cells:
                valid_grasp = False
                invalid_reason_code = "no_feasible_path_between_start_and_goal"
                invalid_reason = "no_feasible_path_between_start_and_goal"
        else:
            cells, cost, expanded, time_ms = [], float("inf"), 0, 0.0
        xz, pitch = self.cells_to_xz_pitch(handle, cells)
        length = self.path_length(xz)
        collision_free = bool(cells) and not self.path_blocked(cells, obs)

        feasible_count = int(np.count_nonzero(feasible))
        clear_count = int(np.count_nonzero(clear_feasible))
        total = int(n_x * n_z)
        feasibility_stats = {
            "total_cells": float(total),
            "mu_feasible_cells": float(clear_count),
            "active_feasible_cells": float(feasible_count),
            "mu_feasible_ratio": float(clear_count / max(total, 1)),
            "active_feasible_ratio": float(feasible_count / max(total, 1)),
            "obstacle_mask_ratio": float(np.count_nonzero(obs) / max(total, 1)),
            "start_projection_dist_m": float(start_projection_dist),
            "goal_projection_dist_m": float(goal_projection_dist),
            "projection_tolerance_m": float(projection_tolerance),
            "start_projection_ok": float(1.0 if start_projection_ok else 0.0),
            "goal_projection_ok": float(1.0 if goal_projection_ok else 0.0),
            "projection_ok": float(1.0 if projection_ok else 0.0),
        }
        feasibility_stats.update(snapshot.stats)
        feasibility_stats.update(arm_volume_stats)
        return PlanResult(
            found=bool(cells),
            cells=list(cells),
            xz=xz,
            pitch=pitch,
            length_m=length,
            g_cost=float(cost),
            nodes_expanded=int(expanded),
            time_ms=float(time_ms),
            start_cell=start_cell,
            goal_cell=goal_cell,
            start_requested_xz=start_req,
            goal_requested_xz=goal_req,
            start_xz=start_xz,
            goal_xz=goal_xz,
            blocked_rects=list(snapshot.display_rects),
            blocked_mask=obs,
            feasible_mask=feasible,
            collision_free=collision_free,
            feasibility_stats=feasibility_stats,
            map_update_stats=dict(snapshot.stats),
            map_snapshot=snapshot,
            path_mu_stats=self.path_mu_stats(handle, cells),
            map_handle=handle,
            start_projection_m=float(start_projection_dist),
            goal_projection_m=float(goal_projection_dist),
            projection_tolerance_m=float(projection_tolerance),
            valid_grasp=bool(valid_grasp),
            invalid_reason_code=invalid_reason_code,
            invalid_reason=invalid_reason,
        )

    @staticmethod
    def path_blocked(cells: Sequence[Cell], mask: np.ndarray) -> bool:
        for ix, iz in cells:
            if mask[int(ix), int(iz)]:
                return True
        return False

    @staticmethod
    def path_length(xz: np.ndarray) -> float:
        arr = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
        if arr.shape[0] < 2:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))

    def phys_to_cell(self, handle: MapHandle, x: float, z: float) -> Cell:
        n_x, n_z = handle.shape
        ix = int(round((float(x) - handle.x0) / handle.resolution_m))
        iz = int(round((float(z) - handle.z0) / handle.resolution_m))
        return int(np.clip(ix, 0, n_x - 1)), int(np.clip(iz, 0, n_z - 1))

    def cell_to_phys(self, handle: MapHandle, ix: int, iz: int) -> Tuple[float, float]:
        return (
            float(handle.x0 + int(ix) * handle.resolution_m),
            float(handle.z0 + int(iz) * handle.resolution_m),
        )

    def cells_to_xz_pitch(self,
                          handle: MapHandle,
                          cells: Sequence[Cell]) -> Tuple[np.ndarray, np.ndarray]:
        if not cells:
            return np.zeros((0, 2), dtype=np.float64), np.zeros(0, dtype=np.float64)
        arr = np.asarray(cells, dtype=np.int64)
        xz = np.column_stack((
            handle.x0 + arr[:, 0].astype(np.float64) * handle.resolution_m,
            handle.z0 + arr[:, 1].astype(np.float64) * handle.resolution_m,
        ))
        pitch = handle.pitch_grid[arr[:, 0], arr[:, 1]].astype(np.float64)
        return xz, pitch

    def path_mu_stats(self, handle: MapHandle, cells: Sequence[Cell]) -> Dict[str, float]:
        if not cells:
            return {"min": float("nan"), "p5": float("nan"), "mean": float("nan")}
        values = np.array([handle.mu_grid[int(ix), int(iz)] for ix, iz in cells], dtype=np.float64)
        return {
            "min": float(np.min(values)),
            "p5": float(np.percentile(values, 5)),
            "mean": float(np.mean(values)),
        }

    @staticmethod
    def _xz_dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return float(math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1])))

    @staticmethod
    def _projection_failure_code(prefix: str,
                                 raw_cell: Cell,
                                 final_mask: np.ndarray,
                                 pruned_mask: np.ndarray) -> str:
        ix, iz = int(raw_cell[0]), int(raw_cell[1])
        if not np.any(pruned_mask):
            return f"{prefix}_no_active_feasible_cell"
        if bool(final_mask[ix, iz]) and not bool(pruned_mask[ix, iz]):
            return f"{prefix}_arm_volume_capsule_pruning"
        if not bool(final_mask[ix, iz]):
            return f"{prefix}_final_mask_blocked"
        return f"{prefix}_projection_exceeded"

    def nearest_feasible_cell(self, feasible: np.ndarray, target: Cell) -> Cell:
        if feasible[target]:
            return int(target[0]), int(target[1])
        valid = np.argwhere(feasible)
        if valid.size == 0:
            raise RuntimeError("no feasible cells in active map")
        d2 = np.sum((valid.astype(np.float64) - np.asarray(target, dtype=np.float64)) ** 2, axis=1)
        best = valid[int(np.argmin(d2))]
        return int(best[0]), int(best[1])

    def _default_start_goal(self,
                            handle: MapHandle,
                            start_xz: Optional[Tuple[float, float]],
                            goal_xz: Optional[Tuple[float, float]]) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if start_xz is None:
            start = self._kin.default_start_xz()
        else:
            start = (float(start_xz[0]), float(start_xz[1]))
        if goal_xz is None:
            goal_x = float(handle.meta.get("x_opt_ee_goal_m", handle.meta.get("optimal_x", X_EE_GOAL)))
            goal_z = float(handle.meta.get("target_z", handle.meta.get("target_height", 0.50)))
            goal = (goal_x, goal_z)
        else:
            goal = (float(goal_xz[0]), float(goal_xz[1]))
        return start, goal

    def _astar(self,
               feasible: np.ndarray,
               start: Cell,
               goal: Cell,
               res: float) -> Tuple[List[Cell], float, int, float]:
        if not feasible[start] or not feasible[goal]:
            return [], float("inf"), 0, 0.0
        n_x, n_z = feasible.shape
        t0 = time.perf_counter()
        g = np.full((n_x, n_z), np.inf, dtype=np.float64)
        closed = np.zeros((n_x, n_z), dtype=bool)
        came: Dict[Cell, Cell] = {}
        g[start] = 0.0
        heap: List[Tuple[float, int, Cell]] = []
        counter = 0
        heapq.heappush(heap, (self._heuristic(start, goal, res), counter, start))
        expanded = 0
        while heap:
            _, _, cur = heapq.heappop(heap)
            if closed[cur]:
                continue
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                path.reverse()
                return path, float(g[goal]), int(expanded), float((time.perf_counter() - t0) * 1000.0)
            closed[cur] = True
            expanded += 1
            cx, cz = cur
            for dx, dz, scale in self.NEIGHBORS:
                nxt = (cx + dx, cz + dz)
                if nxt[0] < 0 or nxt[0] >= n_x or nxt[1] < 0 or nxt[1] >= n_z:
                    continue
                if closed[nxt] or not feasible[nxt]:
                    continue
                tentative = g[cur] + scale * res
                if tentative < g[nxt]:
                    came[nxt] = cur
                    g[nxt] = tentative
                    counter += 1
                    heapq.heappush(heap, (tentative + self._heuristic(nxt, goal, res), counter, nxt))
        return [], float("inf"), int(expanded), float((time.perf_counter() - t0) * 1000.0)

    @staticmethod
    def _heuristic(a: Cell, b: Cell, res: float) -> float:
        return float(math.hypot(a[0] - b[0], a[1] - b[1]) * res)
