from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .joint_mapping import ReducedJointMapper


class TrajectoryValidationError(ValueError):
    """Raised when an active trajectory is unsafe or malformed."""


@dataclass(frozen=True)
class TrajectoryLimits:
    max_velocity_rad_s: float = 0.5
    max_acceleration_rad_s2: float = 0.4
    min_dt_s: float = 0.01
    max_duration_s: float = 120.0


@dataclass(frozen=True)
class TrajectoryPoint:
    positions: np.ndarray
    time_from_start_s: float
    velocities: Optional[np.ndarray] = None

    def as_dict(self) -> Dict[str, object]:
        return {
            "positions": [float(v) for v in self.positions.tolist()],
            "time_from_start_s": float(self.time_from_start_s),
            "velocities": None if self.velocities is None else [float(v) for v in self.velocities.tolist()],
        }


@dataclass(frozen=True)
class JointTrajectoryCommand:
    joint_names: Tuple[str, ...]
    points: Tuple[TrajectoryPoint, ...]
    action_name: str
    metadata: Dict[str, object] = field(default_factory=dict)
    summary: Dict[str, object] = field(default_factory=dict)

    @property
    def duration_s(self) -> float:
        if not self.points:
            return 0.0
        return float(self.points[-1].time_from_start_s)

    def as_dict(self) -> Dict[str, object]:
        return {
            "joint_names": list(self.joint_names),
            "action_name": self.action_name,
            "points": [p.as_dict() for p in self.points],
            "metadata": dict(self.metadata),
            "summary": dict(self.summary),
        }


class ReducedTrajectoryBuilder:
    """Convert accepted reduced q_act trajectories into full joint commands."""

    def __init__(self, mapper: ReducedJointMapper, limits: TrajectoryLimits | None = None) -> None:
        self.mapper = mapper
        self.limits = limits if limits is not None else TrajectoryLimits()

    def from_active_trajectory(
        self,
        q_active_trajectory: Sequence[Sequence[float]] | np.ndarray,
        *,
        dt_s: float | None = None,
        timestamps_s: Sequence[float] | np.ndarray | None = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> JointTrajectoryCommand:
        q = np.asarray(q_active_trajectory, dtype=np.float64)
        if q.ndim != 2 or q.shape[1] != 3:
            raise TrajectoryValidationError("q_active_trajectory must have shape Nx3")
        if q.shape[0] == 0:
            raise TrajectoryValidationError("q_active_trajectory is empty")
        if not np.all(np.isfinite(q)):
            raise TrajectoryValidationError("q_active_trajectory contains non-finite values")

        times = self._timestamps(q.shape[0], dt_s=dt_s, timestamps_s=timestamps_s)
        full = np.stack([self.mapper.full_from_active(row).positions for row in q], axis=0)
        joint_names = self.mapper.contract.joint_names
        self.mapper.validate_goal_joint_names(joint_names)
        self._validate_timing(times)
        summary = self._validate_motion(full, times)

        points = []
        velocities = self._velocities(full, times)
        for idx in range(full.shape[0]):
            vel = velocities[idx].copy() if velocities is not None else None
            points.append(TrajectoryPoint(full[idx].copy(), float(times[idx]), vel))
        summary.update(
            {
                "point_count": int(full.shape[0]),
                "duration_s": float(times[-1]),
                "active_point_count": int(q.shape[0]),
                "command_endpoint": self.mapper.contract.action_name,
            }
        )
        return JointTrajectoryCommand(
            joint_names=joint_names,
            points=tuple(points),
            action_name=self.mapper.contract.action_name,
            metadata=dict(metadata or {}),
            summary=summary,
        )

    def _timestamps(
        self,
        n: int,
        *,
        dt_s: float | None,
        timestamps_s: Sequence[float] | np.ndarray | None,
    ) -> np.ndarray:
        if timestamps_s is not None:
            t = np.asarray(timestamps_s, dtype=np.float64).reshape(-1)
            if t.shape[0] != n:
                raise TrajectoryValidationError("timestamps_s length must match q trajectory")
            return t
        if dt_s is None:
            raise TrajectoryValidationError("dt_s or timestamps_s is required")
        dt = float(dt_s)
        if not np.isfinite(dt) or dt <= 0.0:
            raise TrajectoryValidationError("dt_s must be positive and finite")
        return np.arange(n, dtype=np.float64) * dt

    def _validate_timing(self, times: np.ndarray) -> None:
        if not np.all(np.isfinite(times)):
            raise TrajectoryValidationError("trajectory timestamps contain non-finite values")
        if times[0] < -1.0e-12:
            raise TrajectoryValidationError("trajectory timestamps must start at a non-negative time")
        if times.shape[0] > 1:
            dt = np.diff(times)
            if np.any(dt < float(self.limits.min_dt_s) - 1.0e-12):
                raise TrajectoryValidationError("trajectory timestamps must be strictly increasing with min_dt")
        if float(times[-1]) > float(self.limits.max_duration_s) + 1.0e-12:
            raise TrajectoryValidationError("trajectory duration exceeds limit")

    def _validate_motion(self, full: np.ndarray, times: np.ndarray) -> Dict[str, object]:
        max_velocity = 0.0
        max_acceleration = 0.0
        if full.shape[0] > 1:
            dt = np.diff(times)
            dq = np.diff(full, axis=0)
            vel = np.abs(dq / dt[:, None])
            max_velocity = float(np.max(vel)) if vel.size else 0.0
            if max_velocity > float(self.limits.max_velocity_rad_s) + 1.0e-12:
                raise TrajectoryValidationError("trajectory exceeds velocity limit")
            if vel.shape[0] > 1:
                acc_dt = 0.5 * (dt[:-1] + dt[1:])
                acc = np.abs(np.diff(dq / dt[:, None], axis=0) / acc_dt[:, None])
                max_acceleration = float(np.max(acc)) if acc.size else 0.0
                if max_acceleration > float(self.limits.max_acceleration_rad_s2) + 1.0e-12:
                    raise TrajectoryValidationError("trajectory exceeds acceleration limit")
        return {
            "max_velocity_rad_s": float(max_velocity),
            "max_acceleration_rad_s2": float(max_acceleration),
            "velocity_limit_rad_s": float(self.limits.max_velocity_rad_s),
            "acceleration_limit_rad_s2": float(self.limits.max_acceleration_rad_s2),
        }

    @staticmethod
    def _velocities(full: np.ndarray, times: np.ndarray) -> np.ndarray | None:
        if full.shape[0] <= 1:
            return np.zeros_like(full)
        velocities = np.zeros_like(full)
        dt = np.diff(times)
        segment_vel = np.diff(full, axis=0) / dt[:, None]
        velocities[0] = segment_vel[0]
        velocities[-1] = segment_vel[-1]
        if full.shape[0] > 2:
            velocities[1:-1] = 0.5 * (segment_vel[:-1] + segment_vel[1:])
        return velocities
