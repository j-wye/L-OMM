from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence

import numpy as np

from .joint_mapping import ReducedJointMapper


class ProgressEstimatorError(ValueError):
    """Raised when current state cannot be aligned with the accepted reference."""


@dataclass(frozen=True)
class ProgressEstimate:
    progress_s: float
    nearest_index: int
    q_active: np.ndarray
    distance: float
    stamp_s: float | None = None
    age_s: float | None = None
    stale: bool = False

    def as_dict(self) -> Dict[str, object]:
        return {
            "progress_s": float(self.progress_s),
            "nearest_index": int(self.nearest_index),
            "q_active": [float(v) for v in self.q_active.tolist()],
            "distance": float(self.distance),
            "stamp_s": None if self.stamp_s is None else float(self.stamp_s),
            "age_s": None if self.age_s is None else float(self.age_s),
            "stale": bool(self.stale),
        }


class ProgressEstimator:
    """Estimate arclength progress from the current reduced active state."""

    def __init__(self, mapper: ReducedJointMapper, max_state_age_s: float = 0.25) -> None:
        self.mapper = mapper
        self.max_state_age_s = float(max_state_age_s)

    def estimate_from_joint_state(
        self,
        *,
        joint_names: Sequence[str],
        positions: Sequence[float],
        stamp_s: float,
        now_s: float,
        s_table: Sequence[float] | np.ndarray,
        q_active_trajectory: Sequence[Sequence[float]] | np.ndarray,
    ) -> ProgressEstimate:
        age = float(now_s) - float(stamp_s)
        if not np.isfinite(age) or age < -1.0e-12:
            raise ProgressEstimatorError("joint-state timestamp is invalid")
        if age > self.max_state_age_s + 1.0e-12:
            raise ProgressEstimatorError("joint-state sample is stale")
        q_active = self.mapper.active_from_joint_state(joint_names, positions)
        est = self.estimate_from_active_q(
            q_active,
            s_table=s_table,
            q_active_trajectory=q_active_trajectory,
        )
        return ProgressEstimate(
            progress_s=est.progress_s,
            nearest_index=est.nearest_index,
            q_active=est.q_active,
            distance=est.distance,
            stamp_s=float(stamp_s),
            age_s=float(age),
            stale=False,
        )

    def estimate_from_active_q(
        self,
        q_active: Sequence[float] | np.ndarray,
        *,
        s_table: Sequence[float] | np.ndarray,
        q_active_trajectory: Sequence[Sequence[float]] | np.ndarray,
    ) -> ProgressEstimate:
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        if q.shape[0] != 3 or not np.all(np.isfinite(q)):
            raise ProgressEstimatorError("q_active must be finite length-3")
        s = np.asarray(s_table, dtype=np.float64).reshape(-1)
        traj = np.asarray(q_active_trajectory, dtype=np.float64)
        if traj.ndim != 2 or traj.shape[1] != 3 or traj.shape[0] == 0:
            raise ProgressEstimatorError("q_active_trajectory must have shape Nx3")
        if s.shape[0] != traj.shape[0] or not np.all(np.isfinite(s)) or not np.all(np.isfinite(traj)):
            raise ProgressEstimatorError("s_table and q_active_trajectory must be finite and aligned")
        distances = np.linalg.norm(traj - q[None, :], axis=1)
        idx = int(np.argmin(distances))
        return ProgressEstimate(
            progress_s=float(s[idx]),
            nearest_index=idx,
            q_active=q.copy(),
            distance=float(distances[idx]),
        )

