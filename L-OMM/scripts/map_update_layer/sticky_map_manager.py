#!/usr/bin/env python3
"""FOV-aware sticky occupied/blocked mask bookkeeping."""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .active_map_snapshot import ActiveMapSnapshot


@dataclass(frozen=True)
class StickyMapState:
    occupied_mask: np.ndarray
    blocked_mask: np.ndarray
    free_streak_counter: np.ndarray
    unknown_persistence_counter: np.ndarray
    occluded_persistence_counter: np.ndarray
    last_update_frame_id: int


@dataclass(frozen=True)
class StickyMapParams:
    inflation_m: float = 0.01
    n_free_frames_to_unblock: int = 5
    conservative_unknown_persistence: int = 10
    resolution_m: float = 0.01


class StickyMapManager:
    """Maintain sticky observed occupancy on the reduced task plane."""

    def __init__(self, handle: Any | None = None, params: StickyMapParams | None = None) -> None:
        self.handle = handle
        self.params = params if params is not None else StickyMapParams()

    def initialize(self, initial_snapshot: Any) -> StickyMapState:
        occupied = self._mask(initial_snapshot, "occupied_mask")
        if occupied is None:
            blocked = self._mask(initial_snapshot, "blocked_mask")
            if blocked is None:
                raise ValueError("initial snapshot requires occupied_mask or blocked_mask")
            occupied = np.zeros_like(blocked, dtype=bool)
        occupied = np.asarray(occupied, dtype=bool)
        unknown = self._mask(initial_snapshot, "unknown_mask")
        occluded = self._mask(initial_snapshot, "occluded_mask")
        unknown_counter = np.zeros(occupied.shape, dtype=np.int32)
        occluded_counter = np.zeros(occupied.shape, dtype=np.int32)
        if unknown is not None:
            unknown_arr = np.asarray(unknown, dtype=bool)
            if unknown_arr.shape != occupied.shape:
                raise ValueError("unknown_mask shape must match occupied_mask")
            unknown_counter[unknown_arr] = max(int(self.params.conservative_unknown_persistence), 0)
        if occluded is not None:
            occluded_arr = np.asarray(occluded, dtype=bool)
            if occluded_arr.shape != occupied.shape:
                raise ValueError("occluded_mask shape must match occupied_mask")
            occluded_counter[occluded_arr] = max(int(self.params.conservative_unknown_persistence), 0)
            occluded_for_blocked = occluded_arr | (occluded_counter > 0)
        else:
            occluded_for_blocked = None
        unknown_for_blocked = None if unknown is None else (unknown_counter > 0)
        blocked = self._compose_blocked(occupied, unknown_for_blocked, occluded_for_blocked)
        return StickyMapState(
            occupied_mask=occupied.copy(),
            blocked_mask=blocked,
            free_streak_counter=np.zeros(occupied.shape, dtype=np.int32),
            unknown_persistence_counter=unknown_counter,
            occluded_persistence_counter=occluded_counter,
            last_update_frame_id=0,
        )

    def update(
        self,
        prev_state: StickyMapState,
        current_observation: Any,
        current_fov_mask: np.ndarray | None,
    ) -> StickyMapState:
        prev_occ = np.asarray(prev_state.occupied_mask, dtype=bool)
        if current_fov_mask is None:
            current_fov_mask = self._mask(current_observation, "fov_mask")
            if current_fov_mask is None:
                current_fov_mask = self._mask(current_observation, "cell_projection_fov")
        if current_fov_mask is None:
            raise ValueError("current_fov_mask or observation fov_mask is required")
        fov = np.asarray(current_fov_mask, dtype=bool)
        if fov.shape != prev_occ.shape:
            raise ValueError("current_fov_mask shape must match sticky state")
        observed_occ = self._mask(current_observation, "occupied_mask")
        observed_unknown = self._mask(current_observation, "unknown_mask")
        observed_occluded = self._mask(current_observation, "occluded_mask")
        observed_free = self._mask(current_observation, "free_mask")
        if observed_occ is None:
            observed_occ = np.zeros_like(prev_occ, dtype=bool)
        if observed_unknown is None:
            observed_unknown_arr = np.zeros_like(prev_occ, dtype=bool)
        else:
            observed_unknown_arr = np.asarray(observed_unknown, dtype=bool)
            if observed_unknown_arr.shape != prev_occ.shape:
                raise ValueError("unknown_mask shape must match sticky state")
        if observed_occluded is None:
            observed_occluded_arr = np.zeros_like(prev_occ, dtype=bool)
        else:
            observed_occluded_arr = np.asarray(observed_occluded, dtype=bool)
            if observed_occluded_arr.shape != prev_occ.shape:
                raise ValueError("occluded_mask shape must match sticky state")
        if observed_free is None:
            observed_free = (
                fov
                & (~np.asarray(observed_occ, dtype=bool))
                & (~observed_unknown_arr)
                & (~observed_occluded_arr)
            )

        observed_occ = np.asarray(observed_occ, dtype=bool)
        observed_free = np.asarray(observed_free, dtype=bool)
        if observed_occ.shape != prev_occ.shape or observed_free.shape != prev_occ.shape:
            raise ValueError("observation masks must match sticky state")
        prev_unknown_counter = np.asarray(prev_state.unknown_persistence_counter, dtype=np.int32)
        if prev_unknown_counter.shape != prev_occ.shape:
            raise ValueError("unknown_persistence_counter shape must match sticky state")
        prev_occluded_counter = np.asarray(prev_state.occluded_persistence_counter, dtype=np.int32)
        if prev_occluded_counter.shape != prev_occ.shape:
            raise ValueError("occluded_persistence_counter shape must match sticky state")

        next_occ = prev_occ.copy()
        next_streak = np.asarray(prev_state.free_streak_counter, dtype=np.int32).copy()
        next_unknown_counter = np.maximum(prev_unknown_counter - 1, 0)
        next_occluded_counter = np.maximum(prev_occluded_counter - 1, 0)

        occ_inside = fov & observed_occ
        free_inside = fov & observed_free & (~observed_occ) & (~observed_unknown_arr) & (~observed_occluded_arr)
        unknown_inside = fov & observed_unknown_arr
        occluded_inside = fov & observed_occluded_arr
        next_streak[free_inside] += 1
        next_streak[~free_inside] = 0
        next_occ[occ_inside] = True
        next_streak[occ_inside] = 0
        next_streak[unknown_inside] = 0
        next_streak[occluded_inside] = 0

        unblock = free_inside & (next_streak >= max(int(self.params.n_free_frames_to_unblock), 1))
        next_occ[unblock] = False

        unknown_ttl = max(int(self.params.conservative_unknown_persistence), 0)
        if unknown_ttl > 0:
            next_unknown_counter[unknown_inside] = unknown_ttl
            next_occluded_counter[occluded_inside] = unknown_ttl
        else:
            next_unknown_counter[unknown_inside] = 0
            next_occluded_counter[occluded_inside] = 0
        next_unknown_counter[unblock] = 0
        next_occluded_counter[unblock] = 0
        unknown_mask = unknown_inside | (next_unknown_counter > 0)
        occluded_mask = occluded_inside | (next_occluded_counter > 0)
        blocked = self._compose_blocked(next_occ, unknown_mask, occluded_mask)
        next_state = StickyMapState(
            occupied_mask=next_occ,
            blocked_mask=blocked,
            free_streak_counter=next_streak,
            unknown_persistence_counter=next_unknown_counter,
            occluded_persistence_counter=next_occluded_counter,
            last_update_frame_id=int(prev_state.last_update_frame_id) + 1,
        )
        _jetson_sensitivity_log(
            "StickyMapManager.update",
            prev_state=prev_state,
            next_state=next_state,
            fov=fov,
            observed_occ=observed_occ,
            observed_unknown=observed_unknown_arr,
            observed_occluded=observed_occluded_arr,
            observed_free=observed_free,
            unblock=unblock,
        )
        return next_state

    def _compose_blocked(
        self,
        occupied: np.ndarray,
        unknown: np.ndarray | None,
        occluded: np.ndarray | None = None,
    ) -> np.ndarray:
        blocked = _inflate_mask(
            np.asarray(occupied, dtype=bool),
            inflation_m=float(self.params.inflation_m),
            resolution_m=self._resolution_m(),
        )
        if unknown is not None:
            unk = np.asarray(unknown, dtype=bool)
            if unk.shape != blocked.shape:
                raise ValueError("unknown_mask shape must match occupied_mask")
            blocked = blocked | unk
        if occluded is not None:
            occd = np.asarray(occluded, dtype=bool)
            if occd.shape != blocked.shape:
                raise ValueError("occluded_mask shape must match occupied_mask")
            blocked = blocked | occd
        return blocked

    def apply_to_snapshot(
        self,
        snapshot: ActiveMapSnapshot,
        state: StickyMapState,
        *,
        source_suffix: str = "sticky_runtime",
    ) -> ActiveMapSnapshot:
        """Return a runtime snapshot whose planning masks use sticky state."""

        blocked = np.asarray(state.blocked_mask, dtype=bool)
        occupied = np.asarray(state.occupied_mask, dtype=bool)
        if blocked.shape != tuple(snapshot.handle.shape) or occupied.shape != tuple(snapshot.handle.shape):
            raise ValueError("sticky state shape must match snapshot map shape")
        base = np.asarray(snapshot.base_feasible_mask, dtype=bool)
        final = base & (~blocked)
        stats = dict(snapshot.stats)
        stats.update(
            {
                "sticky_runtime_applied": 1.0,
                "sticky_last_update_frame_id": float(state.last_update_frame_id),
                "sticky_occupied_cells": float(np.count_nonzero(occupied)),
                "sticky_unknown_cells": float(
                    np.count_nonzero(np.asarray(state.unknown_persistence_counter, dtype=np.int32) > 0)
                ),
                "sticky_occluded_cells": float(
                    np.count_nonzero(np.asarray(state.occluded_persistence_counter, dtype=np.int32) > 0)
                ),
                "sticky_blocked_cells": float(np.count_nonzero(blocked)),
            }
        )
        layers = dict(snapshot.layer_masks)
        layers["sticky_occupied"] = occupied.copy()
        layers["sticky_unknown"] = np.asarray(state.unknown_persistence_counter, dtype=np.int32) > 0
        layers["sticky_occluded"] = np.asarray(state.occluded_persistence_counter, dtype=np.int32) > 0
        layers["sticky_blocked"] = blocked.copy()
        # G1 (validator coherence): the runtime snapshot's planning authority is the
        # sticky blocked_mask, so the cross-checked "blocked" layer must equal it.
        # SnapshotValidator.validate compares layer_masks["blocked"] against
        # blocked_mask; without this the standard validator would falsely fail on
        # every sticky-applied snapshot. This re-labels the diagnostic layer only and
        # does NOT change any sticky STATE computation above (occupied/unknown/
        # occluded persistence counters and the composed blocked mask are untouched).
        layers["blocked"] = blocked.copy()
        return ActiveMapSnapshot(
            handle=snapshot.handle,
            base_feasible_mask=base.copy(),
            final_active_mask=final,
            blocked_mask=blocked.copy(),
            occupied_mask=occupied.copy(),
            display_rects=list(snapshot.display_rects),
            stats=stats,
            layer_masks=layers,
            source=f"{snapshot.source}+{source_suffix}",
            mu_min=float(snapshot.mu_min),
        )

    def _resolution_m(self) -> float:
        if self.handle is not None and hasattr(self.handle, "resolution_m"):
            return float(self.handle.resolution_m)
        return float(self.params.resolution_m)

    @staticmethod
    def _mask(obj: Any, name: str) -> np.ndarray | None:
        if isinstance(obj, Mapping):
            value = obj.get(name)
        else:
            value = getattr(obj, name, None)
            if hasattr(obj, "layer_masks"):
                layers = getattr(obj, "layer_masks", {}) or {}
                base_name = name[:-5] if name.endswith("_mask") else name
                aliases = (
                    name,
                    base_name,
                    f"cell_projection_{base_name}",
                    f"sticky_{base_name}",
                )
                for alias in aliases:
                    if alias in layers:
                        layer_value = np.asarray(layers[alias], dtype=bool)
                        if value is None:
                            value = layer_value
                        else:
                            value = np.asarray(value, dtype=bool) | layer_value
                        break
        if value is None:
            return None
        return np.asarray(value, dtype=bool)


def _inflate_mask(mask: np.ndarray, *, inflation_m: float, resolution_m: float) -> np.ndarray:
    base = np.asarray(mask, dtype=bool)
    if not np.any(base) or inflation_m <= 0.0:
        return base.copy()
    res = max(float(resolution_m), 1.0e-9)
    radius_cells = int(np.ceil(float(inflation_m) / res))
    out = base.copy()
    xs, zs = np.nonzero(base)
    n_x, n_z = base.shape
    for dx in range(-radius_cells, radius_cells + 1):
        for dz in range(-radius_cells, radius_cells + 1):
            if np.hypot(dx * res, dz * res) > float(inflation_m) + 0.5 * res:
                continue
            nx = xs + dx
            nz = zs + dz
            valid = (nx >= 0) & (nx < n_x) & (nz >= 0) & (nz < n_z)
            out[nx[valid], nz[valid]] = True
    return out


# === DIAGNOSTIC LOGGING (REMOVABLE) - JETSON_SENSITIVITY_SWEEP ===
def _jetson_sensitivity_log(
    event: str,
    *,
    prev_state: StickyMapState,
    next_state: StickyMapState,
    fov: np.ndarray,
    observed_occ: np.ndarray,
    observed_unknown: np.ndarray,
    observed_occluded: np.ndarray,
    observed_free: np.ndarray,
    unblock: np.ndarray,
) -> None:
    path = os.environ.get("JETSON_SENSITIVITY_SWEEP_LOG")
    if not path:
        return
    prev_occ = np.asarray(prev_state.occupied_mask, dtype=bool)
    next_occ = np.asarray(next_state.occupied_mask, dtype=bool)
    payload = {
        "timestamp_s": time.time(),
        "event": str(event),
        "frame_id": int(next_state.last_update_frame_id),
        "mask_shape": list(next_occ.shape),
        "fov_cell_count": int(np.count_nonzero(np.asarray(fov, dtype=bool))),
        "observed_occ_count": int(np.count_nonzero(np.asarray(observed_occ, dtype=bool))),
        "observed_unknown_count": int(np.count_nonzero(np.asarray(observed_unknown, dtype=bool))),
        "observed_occluded_count": int(np.count_nonzero(np.asarray(observed_occluded, dtype=bool))),
        "observed_free_count": int(np.count_nonzero(np.asarray(observed_free, dtype=bool))),
        "unblock_count": int(np.count_nonzero(np.asarray(unblock, dtype=bool))),
        "occupied_count_prev": int(np.count_nonzero(prev_occ)),
        "occupied_count_next": int(np.count_nonzero(next_occ)),
        "blocked_count_next": int(np.count_nonzero(np.asarray(next_state.blocked_mask, dtype=bool))),
        "unknown_persistence_count_next": int(
            np.count_nonzero(np.asarray(next_state.unknown_persistence_counter, dtype=np.int32) > 0)
        ),
        "occluded_persistence_count_next": int(
            np.count_nonzero(np.asarray(next_state.occluded_persistence_counter, dtype=np.int32) > 0)
        ),
        "occupied_transition_count": int(np.count_nonzero(prev_occ != next_occ)),
    }
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")
# === END DIAGNOSTIC LOGGING ===
