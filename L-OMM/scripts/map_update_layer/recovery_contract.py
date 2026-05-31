#!/usr/bin/env python3
"""Mobile-base recovery handoff contract without implementing base motion."""
from __future__ import annotations

import hashlib
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np

from .active_map_snapshot import ActiveMapSnapshot


@dataclass(frozen=True)
class RecoveryRequest:
    """Minimal request passed upward when the current snapshot is not usable."""

    event_class: str
    reject_reason: str
    invalid_reason_code: str
    active_map_tag: str
    target_height_m: float
    reference_progress_s: float
    snapshot_sequence_id: int
    blocked_mask_hash: str
    current_start_xz: Optional[Tuple[float, float]] = None
    notes: str = "mobile_base_reposition_required_before_retry"

    def as_dict(self) -> Dict[str, object]:
        return {
            "event_class": self.event_class,
            "reject_reason": self.reject_reason,
            "invalid_reason_code": self.invalid_reason_code,
            "active_map_tag": self.active_map_tag,
            "target_height_m": float(self.target_height_m),
            "reference_progress_s": float(self.reference_progress_s),
            "snapshot_sequence_id": int(self.snapshot_sequence_id),
            "blocked_mask_hash": self.blocked_mask_hash,
            "current_start_xz": list(self.current_start_xz) if self.current_start_xz is not None else None,
            "notes": self.notes,
        }


@dataclass(frozen=True)
class SyntheticRecoveryResponse:
    """Dry-run response used before a real mobile-base controller exists."""

    base_reposition_executed: bool
    new_snapshot_available: bool
    retry_planning_allowed: bool
    notes: str = ""

    def as_dict(self) -> Dict[str, object]:
        return {
            "base_reposition_executed": bool(self.base_reposition_executed),
            "new_snapshot_available": bool(self.new_snapshot_available),
            "retry_planning_allowed": bool(self.retry_planning_allowed),
            "notes": self.notes,
        }


class RecoveryHandoffContract:
    """Build recovery handoff records without commanding a mobile base."""

    def build_request(
        self,
        *,
        snapshot: ActiveMapSnapshot,
        event_class: str,
        reject_reason: str = "",
        invalid_reason_code: str = "",
        reference_progress_s: float = 0.0,
        current_start_xz: Optional[Tuple[float, float]] = None,
    ) -> RecoveryRequest:
        meta = snapshot.handle.meta
        return RecoveryRequest(
            event_class=str(event_class),
            reject_reason=str(reject_reason),
            invalid_reason_code=str(invalid_reason_code),
            active_map_tag=str(snapshot.handle.tag),
            target_height_m=float(meta.get("target_height", meta.get("target_z", np.nan))),
            reference_progress_s=float(reference_progress_s),
            snapshot_sequence_id=int(snapshot.stats.get("map_update_sequence_id", 0.0)),
            blocked_mask_hash=self.blocked_mask_hash(snapshot.blocked_mask),
            current_start_xz=current_start_xz,
        )

    @staticmethod
    def blocked_mask_hash(mask: np.ndarray) -> str:
        arr = np.ascontiguousarray(np.asarray(mask, dtype=np.uint8))
        digest = hashlib.sha256()
        digest.update(str(tuple(arr.shape)).encode("ascii"))
        digest.update(arr.tobytes())
        return f"mask_{arr.shape[0]}x{arr.shape[1]}_{digest.hexdigest()[:16]}"

    @staticmethod
    def synthetic_response(*, new_snapshot_available: bool) -> SyntheticRecoveryResponse:
        return SyntheticRecoveryResponse(
            base_reposition_executed=bool(new_snapshot_available),
            new_snapshot_available=bool(new_snapshot_available),
            retry_planning_allowed=bool(new_snapshot_available),
            notes="synthetic_response_only_no_mobile_base_command",
        )
