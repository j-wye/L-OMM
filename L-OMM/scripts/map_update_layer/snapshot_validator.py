#!/usr/bin/env python3
"""Structural validation for active-map snapshots."""
from __future__ import annotations

from typing import Dict

import numpy as np

from .active_map_snapshot import ActiveMapSnapshot


class SnapshotValidator:
    """Check invariants that should hold before passing a snapshot to control."""

    def validate(self, snapshot: ActiveMapSnapshot) -> Dict[str, float]:
        base = np.asarray(snapshot.base_feasible_mask, dtype=bool)
        final = np.asarray(snapshot.final_active_mask, dtype=bool)
        blocked = np.asarray(snapshot.blocked_mask, dtype=bool)
        expected_final = base & (~blocked)
        final_subset_base = bool(np.all(final <= base))
        final_disjoint_blocked = bool(not np.any(final & blocked))
        final_equals_base_minus_blocked = bool(np.array_equal(final, expected_final))
        blocked_matches_layers = True
        if snapshot.layer_masks:
            layer_blocked = np.asarray(snapshot.layer_masks.get("blocked", blocked), dtype=bool)
            blocked_matches_layers = bool(np.array_equal(layer_blocked, blocked))
        ok = bool(
            final_subset_base
            and final_disjoint_blocked
            and final_equals_base_minus_blocked
            and blocked_matches_layers
        )
        return {
            "snapshot_valid": 1.0 if ok else 0.0,
            "final_subset_base": 1.0 if final_subset_base else 0.0,
            "final_disjoint_blocked": 1.0 if final_disjoint_blocked else 0.0,
            "final_equals_base_minus_blocked": 1.0 if final_equals_base_minus_blocked else 0.0,
            "blocked_matches_layers": 1.0 if blocked_matches_layers else 0.0,
            "base_feasible_cells": float(np.count_nonzero(base)),
            "final_feasible_cells": float(np.count_nonzero(final)),
            "blocked_cells": float(np.count_nonzero(blocked)),
        }

    def assert_valid(self, snapshot: ActiveMapSnapshot) -> None:
        metrics = self.validate(snapshot)
        if metrics["snapshot_valid"] < 1.0:
            raise ValueError(f"invalid ActiveMapSnapshot invariants: {metrics}")
