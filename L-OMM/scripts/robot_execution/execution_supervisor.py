from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence

import numpy as np

from map_update_layer.lightweight_supervisor import LightweightSupervisorDryRun
from map_update_layer.recovery_contract import RecoveryHandoffContract
from map_update_layer.runtime_contract import RuntimeEventClass


@dataclass(frozen=True)
class ExecutionCandidate:
    q_active_trajectory: np.ndarray
    s_table: np.ndarray
    xz_table: np.ndarray
    dt_s: float | None = None
    timestamps_s: np.ndarray | None = None
    candidate_metrics: Dict[str, object] = field(default_factory=dict)
    metadata: Dict[str, object] = field(default_factory=dict)

    def __init__(
        self,
        q_active_trajectory: Sequence[Sequence[float]] | np.ndarray,
        s_table: Sequence[float] | np.ndarray,
        xz_table: Sequence[Sequence[float]] | np.ndarray,
        *,
        dt_s: float | None = None,
        timestamps_s: Sequence[float] | np.ndarray | None = None,
        candidate_metrics: Optional[Dict[str, object]] = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> None:
        object.__setattr__(self, "q_active_trajectory", np.asarray(q_active_trajectory, dtype=np.float64).reshape(-1, 3))
        object.__setattr__(self, "s_table", np.asarray(s_table, dtype=np.float64).reshape(-1))
        object.__setattr__(self, "xz_table", np.asarray(xz_table, dtype=np.float64).reshape(-1, 2))
        object.__setattr__(self, "dt_s", None if dt_s is None else float(dt_s))
        object.__setattr__(self, "timestamps_s", None if timestamps_s is None else np.asarray(timestamps_s, dtype=np.float64).reshape(-1))
        object.__setattr__(self, "candidate_metrics", dict(candidate_metrics or {}))
        object.__setattr__(self, "metadata", dict(metadata or {}))
        self._validate()

    @property
    def accepted(self) -> bool:
        return bool(self.candidate_metrics.get("candidate_accepted", False))

    def _validate(self) -> None:
        if self.q_active_trajectory.shape[0] == 0:
            raise ValueError("ExecutionCandidate requires non-empty q_active_trajectory")
        if self.s_table.shape[0] != self.q_active_trajectory.shape[0]:
            raise ValueError("ExecutionCandidate s_table must align with q_active_trajectory")
        if self.xz_table.shape[0] != self.q_active_trajectory.shape[0]:
            raise ValueError("ExecutionCandidate xz_table must align with q_active_trajectory")
        if not np.all(np.isfinite(self.q_active_trajectory)):
            raise ValueError("ExecutionCandidate q_active_trajectory must be finite")


@dataclass(frozen=True)
class ExecutionSupervisorResult:
    action: str
    replanning_triggered: bool = False
    recovery_required: bool = False
    candidate_accepted: bool = False
    cancel_sent: bool = False
    goal_sent: bool = False
    backtrack_goal_sent: bool = False
    reference_capture_count: int = 0
    recovery_request: Any | None = None
    same_snapshot_retry_prevented: bool = False
    command_status: Dict[str, object] = field(default_factory=dict)
    runtime_decision: Any | None = None
    reason: str = ""

    def as_dict(self) -> Dict[str, object]:
        return {
            "action": self.action,
            "replanning_triggered": bool(self.replanning_triggered),
            "recovery_required": bool(self.recovery_required),
            "candidate_accepted": bool(self.candidate_accepted),
            "cancel_sent": bool(self.cancel_sent),
            "goal_sent": bool(self.goal_sent),
            "backtrack_goal_sent": bool(self.backtrack_goal_sent),
            "reference_capture_count": int(self.reference_capture_count),
            "recovery_request": self._recovery_dict(),
            "same_snapshot_retry_prevented": bool(self.same_snapshot_retry_prevented),
            "command_status": dict(self.command_status),
            "runtime_decision": self._decision_dict(),
            "reason": self.reason,
        }

    def _recovery_dict(self) -> object:
        if self.recovery_request is None:
            return None
        if hasattr(self.recovery_request, "as_dict"):
            return self.recovery_request.as_dict()
        return self.recovery_request

    def _decision_dict(self) -> object:
        if self.runtime_decision is None:
            return None
        if hasattr(self.runtime_decision, "as_dict"):
            return self.runtime_decision.as_dict()
        return self.runtime_decision


class MovingArmExecutionSupervisor:
    """Tie accepted references, fake/armed action boundary, and event handling."""

    def __init__(
        self,
        *,
        trajectory_builder,
        trajectory_client,
        reference_recorder: Any | None = None,
        recovery_contract: RecoveryHandoffContract | None = None,
        T_base_cam: Any | None = None,
        execution_history: Any | None = None,
        backtrack_on_recovery: bool = False,
    ) -> None:
        self.trajectory_builder = trajectory_builder
        self.trajectory_client = trajectory_client
        self.reference_recorder = reference_recorder
        self.recovery_contract = recovery_contract if recovery_contract is not None else RecoveryHandoffContract()
        self.execution_history = execution_history
        self.backtrack_on_recovery = bool(backtrack_on_recovery)
        self.decision_layer = LightweightSupervisorDryRun()
        self.T_base_cam = T_base_cam
        self.current_candidate: ExecutionCandidate | None = None
        self.current_command = None
        self.reference_capture_count = 0
        self._last_recovery_blocked_hash: str | None = None
        self._last_backtrack_error: str | None = None

    def record_joint_state(
        self,
        *,
        joint_names: Sequence[str],
        positions: Sequence[float] | np.ndarray,
        stamp_s: float,
        progress_s: float | None = None,
        blocked_mask_hash: str | None = None,
        metadata: Optional[Dict[str, object]] = None,
    ) -> Any:
        if self.execution_history is None:
            raise ValueError("execution_history recorder is not configured")
        return self.execution_history.record_joint_state(
            joint_names=joint_names,
            positions=positions,
            stamp_s=stamp_s,
            progress_s=progress_s,
            blocked_mask_hash=blocked_mask_hash,
            metadata=metadata,
        )

    def start(self, candidate: ExecutionCandidate, snapshot: Any) -> ExecutionSupervisorResult:
        if not candidate.accepted:
            raise ValueError("cannot start execution from a rejected candidate")
        command = self._command_from_candidate(candidate)
        status = self.trajectory_client.send(command)
        self.current_candidate = candidate
        self.current_command = command
        self._capture_reference(candidate, snapshot)
        self._last_recovery_blocked_hash = None
        return ExecutionSupervisorResult(
            action="started",
            candidate_accepted=True,
            goal_sent=True,
            reference_capture_count=self.reference_capture_count,
            command_status=status.as_dict() if hasattr(status, "as_dict") else {"state": str(status)},
            reason="initial_candidate_accepted",
        )

    def retry_would_be_prevented(self, snapshot: Any) -> bool:
        """Read-only precheck: would the blocked-mask-hash guard short-circuit a replan?

        Returns True iff ``_last_recovery_blocked_hash`` is set and equals this
        snapshot's blocked-mask hash, i.e. exactly the condition ``handle_event``
        uses (line: ``if self._last_recovery_blocked_hash == blocked_hash``) to
        return ``same_snapshot_retry_prevented`` before touching any candidate.

        It mutates NO supervisor state, so the orchestrator may call it BEFORE the
        expensive ControlModule replan to skip a redundant run on a repeated
        identical blocking snapshot. Because it mirrors the gate exactly, when this
        returns True a subsequent ``handle_event(..., candidate=None)`` still
        short-circuits to ``same_snapshot_retry_prevented`` via the same hash gate.
        """
        if self._last_recovery_blocked_hash is None:
            return False
        blocked_hash = self.recovery_contract.blocked_mask_hash(snapshot.blocked_mask)
        return self._last_recovery_blocked_hash == blocked_hash

    def handle_event(
        self,
        report: Any,
        *,
        snapshot: Any,
        candidate: ExecutionCandidate | None = None,
        current_start_xz: tuple[float, float] | None = None,
    ) -> ExecutionSupervisorResult:
        event = str(getattr(report, "event_class", ""))
        reason = str(getattr(report, "reason", ""))
        progress = float(getattr(report, "current_s_m", 0.0))
        triggered = bool(getattr(report, "replanning_triggered", False))

        if event == RuntimeEventClass.CURRENT_POSE_UNSAFE:
            status = self.trajectory_client.cancel()
            self._invalidate_reference()
            backtrack_status = self._maybe_start_backtrack(
                {
                    "event_class": event,
                    "reason": reason,
                    "reference_progress_s": progress,
                }
            )
            decision = self.decision_layer.decide(
                event_class=event,
                replanning_triggered=False,
                reason=reason,
            )
            recovery = self.recovery_contract.build_request(
                snapshot=snapshot,
                event_class=event,
                reject_reason=RuntimeEventClass.CURRENT_POSE_UNSAFE,
                invalid_reason_code=reason or "current_pose_intersects_blocked_mask",
                reference_progress_s=progress,
                current_start_xz=current_start_xz,
            )
            return ExecutionSupervisorResult(
                action="safety_recovery_boundary",
                replanning_triggered=False,
                recovery_required=True,
                cancel_sent=True,
                goal_sent=bool(backtrack_status),
                backtrack_goal_sent=bool(backtrack_status),
                reference_capture_count=self.reference_capture_count,
                recovery_request=recovery,
                command_status=self._merge_status(status, backtrack_status),
                runtime_decision=decision,
                reason=reason,
            )

        if not triggered:
            decision = self.decision_layer.decide(
                event_class=event,
                replanning_triggered=False,
                reason=reason,
            )
            return ExecutionSupervisorResult(
                action="continue_current_reference",
                replanning_triggered=False,
                reference_capture_count=self.reference_capture_count,
                runtime_decision=decision,
                reason=reason,
            )

        blocked_hash = self.recovery_contract.blocked_mask_hash(snapshot.blocked_mask)
        if self._last_recovery_blocked_hash == blocked_hash:
            return ExecutionSupervisorResult(
                action="same_snapshot_retry_prevented",
                replanning_triggered=True,
                recovery_required=True,
                reference_capture_count=self.reference_capture_count,
                same_snapshot_retry_prevented=True,
                reason="same_blocked_mask_hash_retry_prevented",
            )

        if candidate is None:
            status = self.trajectory_client.cancel()
            decision = self.decision_layer.decide(
                event_class=event,
                replanning_triggered=True,
                reason=reason,
                candidate_metrics=None,
            )
            return ExecutionSupervisorResult(
                action="replan_required",
                replanning_triggered=True,
                cancel_sent=True,
                reference_capture_count=self.reference_capture_count,
                command_status=status.as_dict() if hasattr(status, "as_dict") else {},
                runtime_decision=decision,
                reason=reason,
            )

        status = self.trajectory_client.cancel()
        if candidate.accepted:
            decision = self.decision_layer.decide(
                event_class=event,
                replanning_triggered=True,
                reason=reason,
                candidate_metrics=candidate.candidate_metrics,
            )
            command = self._command_from_candidate(candidate)
            send_status = self.trajectory_client.send(command)
            self.current_candidate = candidate
            self.current_command = command
            self._capture_reference(candidate, snapshot)
            self._last_recovery_blocked_hash = None
            return ExecutionSupervisorResult(
                action="switched_to_accepted_reference",
                replanning_triggered=True,
                candidate_accepted=True,
                cancel_sent=True,
                goal_sent=True,
                reference_capture_count=self.reference_capture_count,
                command_status=send_status.as_dict() if hasattr(send_status, "as_dict") else {},
                runtime_decision=decision,
                reason=reason,
            )

        metrics = dict(candidate.candidate_metrics)
        backtrack_status = self._maybe_start_backtrack(
            {
                "event_class": event,
                "reason": reason,
                "reference_progress_s": progress,
                "candidate_rejected": True,
            }
        )
        decision = self.decision_layer.decide(
            event_class=event,
            replanning_triggered=True,
            reason=reason,
            candidate_metrics=metrics,
        )
        recovery = self.recovery_contract.build_request(
            snapshot=snapshot,
            event_class=event,
            reject_reason=str(metrics.get("rejection_reason", "UNGRASPABLE")),
            invalid_reason_code=str(metrics.get("invalid_reason_code", "")),
            reference_progress_s=progress,
            current_start_xz=current_start_xz,
        )
        self._last_recovery_blocked_hash = blocked_hash
        return ExecutionSupervisorResult(
            action="recovery_request",
            replanning_triggered=True,
            recovery_required=True,
            candidate_accepted=False,
            cancel_sent=bool(status.state == "cancelled"),
            goal_sent=bool(backtrack_status),
            backtrack_goal_sent=bool(backtrack_status),
            reference_capture_count=self.reference_capture_count,
            recovery_request=recovery,
            command_status=self._merge_status(status, backtrack_status),
            runtime_decision=decision,
            reason=reason,
        )

    def _command_from_candidate(self, candidate: ExecutionCandidate):
        return self.trajectory_builder.from_active_trajectory(
            candidate.q_active_trajectory,
            dt_s=candidate.dt_s,
            timestamps_s=candidate.timestamps_s,
            metadata=candidate.metadata,
        )

    def _capture_reference(self, candidate: ExecutionCandidate, snapshot: Any) -> None:
        self.reference_capture_count += 1
        if self.reference_recorder is not None:
            self.reference_recorder.capture(
                snapshot,
                q_act=candidate.q_active_trajectory,
                T_base_cam=self.T_base_cam,
                reference_corridor=candidate.xz_table,
                metadata={
                    "s_table": candidate.s_table.tolist(),
                    **dict(candidate.metadata),
                },
            )

    def _invalidate_reference(self) -> None:
        if self.reference_recorder is not None and hasattr(self.reference_recorder, "invalidate"):
            self.reference_recorder.invalidate()

    def _maybe_start_backtrack(self, metadata: Dict[str, object]) -> Any | None:
        self._last_backtrack_error = None
        if not self.backtrack_on_recovery or self.execution_history is None:
            return None
        try:
            command = self.execution_history.build_reverse_command(metadata=metadata)
            return self.trajectory_client.send(command)
        except Exception as exc:
            self._last_backtrack_error = f"{type(exc).__name__}: {exc}"
            return None

    def _merge_status(self, primary: Any, backtrack: Any | None) -> Dict[str, object]:
        out: Dict[str, object] = {}
        if hasattr(primary, "as_dict"):
            out.update(primary.as_dict())
        elif primary is not None:
            out["primary"] = str(primary)
        if backtrack is not None:
            out["backtrack"] = backtrack.as_dict() if hasattr(backtrack, "as_dict") else {"state": str(backtrack)}
        if self._last_backtrack_error:
            out["backtrack_error"] = self._last_backtrack_error
        return out
