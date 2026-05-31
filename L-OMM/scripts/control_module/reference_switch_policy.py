#!/usr/bin/env python3
"""Reference validity and candidate acceptance policy for event-triggered replanning."""
from __future__ import annotations

from typing import Dict, Tuple

import numpy as np

from map_update_layer.map_update_layer import MapUpdateLayer
from episode_result import EpisodeResult
from plan_result import PlanResult


class ReferenceSwitchPolicy:
    """Keep blockage/acceptance logic above the DLS controller."""

    def remaining_reference_blocked(self,
                                    plan: PlanResult,
                                    reference,
                                    blocked_mask: np.ndarray,
                                    start_s: float = 0.0) -> Tuple[bool, int]:
        s, xz, _ = reference.table_arrays()
        if s.size != xz.shape[0]:
            return False, 0
        remain = xz[np.asarray(s, dtype=np.float64) >= float(start_s)]
        if remain.size == 0:
            return False, 0
        return MapUpdateLayer.samples_blocked(plan.map_handle, remain, blocked_mask)

    def candidate_accepted(self,
                           plan: PlanResult,
                           reference,
                           episode: EpisodeResult,
                           capsule_metrics: Dict[str, float]) -> Dict[str, object]:
        ref_metrics = dict(getattr(reference, "metrics", {}))
        proxy_free = float(capsule_metrics.get("capsule_proxy_collision_free", 0.0))
        accepted = bool(
            plan.found
            and plan.collision_free
            and plan.valid_grasp
            and bool(ref_metrics.get("reference_samples_feasible", True))
            and episode.success_basic
            and proxy_free >= 1.0
        )
        return {
            "candidate_accepted": accepted,
            "reject_reason": "" if accepted else self._reject_reason(plan, ref_metrics, episode, capsule_metrics),
        }

    @staticmethod
    def _reject_reason(plan: PlanResult,
                       ref_metrics: Dict[str, object],
                       episode: EpisodeResult,
                       capsule_metrics: Dict[str, float]) -> str:
        if not plan.valid_grasp:
            token = plan.invalid_reason_code or plan.invalid_reason
            return f"grasp_plan_invalid:{token}" if token else "grasp_plan_invalid"
        if not plan.found:
            return "path_not_found"
        if not plan.collision_free:
            return "path_blocked"
        if not bool(ref_metrics.get("reference_samples_feasible", True)):
            return "reference_infeasible"
        if not episode.success_basic:
            return f"dls_{episode.fail_reason.name.lower()}"
        proxy_free = float(capsule_metrics.get("capsule_proxy_collision_free", 0.0))
        if proxy_free < 1.0:
            return "capsule_proxy_collision"
        return "unknown"
