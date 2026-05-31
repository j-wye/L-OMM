from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from .joint_mapping import ReducedJointMapper
from .trajectory_builder import JointTrajectoryCommand, TrajectoryLimits, TrajectoryPoint


class ExecutionHistoryError(ValueError):
    """Raised when recorded execution history cannot produce a safe retreat."""


@dataclass(frozen=True)
class ExecutionHistorySample:
    stamp_s: float
    joint_names: Tuple[str, ...]
    positions: np.ndarray
    q_active: np.ndarray
    progress_s: float | None = None
    blocked_mask_hash: str | None = None
    metadata: Dict[str, object] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, object]:
        return {
            "stamp_s": float(self.stamp_s),
            "joint_names": list(self.joint_names),
            "positions": [float(v) for v in self.positions.tolist()],
            "q_active": [float(v) for v in self.q_active.tolist()],
            "progress_s": None if self.progress_s is None else float(self.progress_s),
            "blocked_mask_hash": self.blocked_mask_hash,
            "metadata": dict(self.metadata),
        }


class ExecutionHistoryRecorder:
    """Record executed joint states and build reverse backtrack commands.

    The recorder stores the controller-order 7-joint state, not only the reduced
    active joints, so the finger joint follows the same command contract as the
    arm controller during a reverse retreat.
    """

    def __init__(
        self,
        mapper: ReducedJointMapper,
        *,
        limits: TrajectoryLimits | None = None,
        max_samples: int = 2000,
    ) -> None:
        self.mapper = mapper
        self.limits = limits if limits is not None else TrajectoryLimits()
        self.max_samples = max(int(max_samples), 2)
        self._samples: list[ExecutionHistorySample] = []

    @property
    def samples(self) -> Tuple[ExecutionHistorySample, ...]:
        return tuple(self._samples)

    def clear(self) -> None:
        self._samples.clear()

    def record_joint_state(
        self,
        *,
        joint_names: Sequence[str],
        positions: Sequence[float] | np.ndarray,
        stamp_s: float,
        progress_s: float | None = None,
        blocked_mask_hash: str | None = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> ExecutionHistorySample:
        names = tuple(str(v) for v in joint_names)
        if len(names) != len(set(names)):
            raise ExecutionHistoryError("joint-state sample contains duplicate joint names")
        pos = np.asarray(positions, dtype=np.float64).reshape(-1)
        if pos.shape[0] != len(names) or not np.all(np.isfinite(pos)):
            raise ExecutionHistoryError("joint-state positions must be finite and match joint names")
        if not np.isfinite(float(stamp_s)):
            raise ExecutionHistoryError("joint-state timestamp must be finite")
        table = {name: float(pos[idx]) for idx, name in enumerate(names)}
        contract_names = self.mapper.contract.joint_names
        missing = [name for name in contract_names if name not in table]
        if missing:
            raise ExecutionHistoryError(f"joint-state sample missing controller joints: {missing}")
        ordered = np.asarray([table[name] for name in contract_names], dtype=np.float64)
        self.mapper.validate_goal_joint_names(contract_names)
        q_active = self.mapper.active_from_joint_state(contract_names, ordered)
        sample = ExecutionHistorySample(
            stamp_s=float(stamp_s),
            joint_names=contract_names,
            positions=ordered,
            q_active=q_active,
            progress_s=None if progress_s is None else float(progress_s),
            blocked_mask_hash=None if blocked_mask_hash is None else str(blocked_mask_hash),
            metadata=dict(metadata or {}),
        )
        self._append(sample)
        return sample

    def record_active_state(
        self,
        q_active: Sequence[float] | np.ndarray,
        *,
        stamp_s: float,
        finger_position: float | None = None,
        progress_s: float | None = None,
        blocked_mask_hash: str | None = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> ExecutionHistorySample:
        full = self.mapper.full_from_active(q_active)
        positions = full.positions.copy()
        if finger_position is not None:
            finger_idx = full.joint_names.index("right_finger_bottom_joint")
            positions[finger_idx] = float(finger_position)
        return self.record_joint_state(
            joint_names=full.joint_names,
            positions=positions,
            stamp_s=float(stamp_s),
            progress_s=progress_s,
            blocked_mask_hash=blocked_mask_hash,
            metadata=metadata,
        )

    def build_reverse_command(
        self,
        *,
        include_current: bool = True,
        metadata: Optional[Dict[str, object]] = None,
    ) -> JointTrajectoryCommand:
        if len(self._samples) < 2:
            raise ExecutionHistoryError("at least two history samples are required for backtrack")
        samples = list(reversed(self._samples))
        positions = np.stack([s.positions for s in samples], axis=0)
        positions = self._deduplicate_consecutive_positions(positions)
        if not include_current and positions.shape[0] > 1:
            positions = positions[1:]
        if positions.shape[0] < 2:
            raise ExecutionHistoryError("reverse backtrack requires at least two distinct positions")
        times, summary = self._retime(positions)
        points = tuple(
            TrajectoryPoint(positions[idx].copy(), float(times[idx]))
            for idx in range(positions.shape[0])
        )
        merged_metadata = {
            "source": "execution_history_reverse_backtrack",
            "history_sample_count": int(len(self._samples)),
            **dict(metadata or {}),
        }
        summary.update(
            {
                "point_count": int(positions.shape[0]),
                "duration_s": float(times[-1]),
                "history_sample_count": int(len(self._samples)),
                "command_endpoint": self.mapper.contract.action_name,
                "backtrack": True,
            }
        )
        return JointTrajectoryCommand(
            joint_names=self.mapper.contract.joint_names,
            points=points,
            action_name=self.mapper.contract.action_name,
            metadata=merged_metadata,
            summary=summary,
        )

    def as_dict(self) -> Dict[str, object]:
        return {
            "sample_count": int(len(self._samples)),
            "max_samples": int(self.max_samples),
            "latest_sample": None if not self._samples else self._samples[-1].as_dict(),
        }

    def _append(self, sample: ExecutionHistorySample) -> None:
        self._samples.append(sample)
        if len(self._samples) > self.max_samples:
            del self._samples[: len(self._samples) - self.max_samples]

    @staticmethod
    def _deduplicate_consecutive_positions(positions: np.ndarray) -> np.ndarray:
        keep = [0]
        for idx in range(1, positions.shape[0]):
            if not np.allclose(positions[idx], positions[keep[-1]], atol=1.0e-12, rtol=0.0):
                keep.append(idx)
        return positions[keep]

    def _retime(self, positions: np.ndarray) -> tuple[np.ndarray, Dict[str, object]]:
        if not np.all(np.isfinite(positions)):
            raise ExecutionHistoryError("history positions must be finite")
        max_velocity = max(float(self.limits.max_velocity_rad_s), 1.0e-9)
        min_dt = max(float(self.limits.min_dt_s), 1.0e-9)
        dq = np.abs(np.diff(positions, axis=0))
        segment_dt = np.maximum(np.max(dq, axis=1) / max_velocity, min_dt)
        times = np.concatenate(([0.0], np.cumsum(segment_dt)))

        for _ in range(12):
            summary = self._motion_summary(positions, times)
            if (
                summary["max_velocity_rad_s"] <= float(self.limits.max_velocity_rad_s) + 1.0e-12
                and summary["max_acceleration_rad_s2"] <= float(self.limits.max_acceleration_rad_s2) + 1.0e-12
            ):
                if float(times[-1]) > float(self.limits.max_duration_s) + 1.0e-12:
                    raise ExecutionHistoryError("reverse backtrack duration exceeds limit")
                summary.update(
                    {
                        "velocity_limit_rad_s": float(self.limits.max_velocity_rad_s),
                        "acceleration_limit_rad_s2": float(self.limits.max_acceleration_rad_s2),
                    }
                )
                return times, summary
            acc = max(float(summary["max_acceleration_rad_s2"]), 1.0e-12)
            scale = max(1.05, float(np.sqrt(acc / max(float(self.limits.max_acceleration_rad_s2), 1.0e-12))) * 1.05)
            times = times * scale
        raise ExecutionHistoryError("could not retime reverse history within safety limits")

    @staticmethod
    def _motion_summary(positions: np.ndarray, times: np.ndarray) -> Dict[str, object]:
        max_velocity = 0.0
        max_acceleration = 0.0
        if positions.shape[0] > 1:
            dt = np.diff(times)
            if np.any(dt <= 0.0):
                raise ExecutionHistoryError("reverse timestamps must be strictly increasing")
            vel = np.diff(positions, axis=0) / dt[:, None]
            max_velocity = float(np.max(np.abs(vel))) if vel.size else 0.0
            if vel.shape[0] > 1:
                acc_dt = 0.5 * (dt[:-1] + dt[1:])
                acc = np.diff(vel, axis=0) / acc_dt[:, None]
                max_acceleration = float(np.max(np.abs(acc))) if acc.size else 0.0
        return {
            "max_velocity_rad_s": float(max_velocity),
            "max_acceleration_rad_s2": float(max_acceleration),
        }
