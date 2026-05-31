"""Check-only / default-closed physical dry-run runner, monitor, and logger.

This is the last software layer before the human-triggered physical dry-run. It
orchestrates:

    preflight (offline) -> name-based start-posture check -> GATED single send
    -> live feedback monitoring (abort classes A-D) -> cancel-on-violation
    -> log-schema evidence package + verdict.

It DEFAULTS to check-only / no-autosend. A send opens ONLY when the runner's own
``PhysicalDryRunAuthorization`` carries a matching armed token AND an explicit
single-send approval AND an allow-action flag (enforced here, independently of the
safety gate's fake-mode rule). The runner depends only on the
``ControllerRuntimeInterface`` seam; a scripted fake implements it for verification
and a real Jetson ROS adapter (out of scope) would implement the same interface.

Two-layer verdict (do not conflate): ``runner_readiness_verdict`` is the pre-run
readiness, ``contract_verdict`` is the safety-contract §9 result (only meaningful
after a send). ``physical`` is always False here; only a real hardware run may set
it True. The production armed token never appears as a literal in this file.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Protocol, Sequence, Tuple

import numpy as np


# -- verdict vocabularies ------------------------------------------------------

# runner readiness (pre-run / no physical claim)
CHECK_ONLY_READY_NO_SEND = "CHECK_ONLY_READY_NO_SEND"
NOT_READY_PREFLIGHT_FAIL = "NOT_READY_PREFLIGHT_FAIL"
NOT_READY_POSTURE_FAIL = "NOT_READY_POSTURE_FAIL"
AUTHORIZED_SENT = "AUTHORIZED_SENT"
SECOND_SEND_REFUSED = "SECOND_SEND_REFUSED"

# safety contract §9 verdicts (+ a "not applicable until a send happens")
NOT_APPLICABLE_UNTIL_SEND = "NOT_APPLICABLE_UNTIL_SEND"
PASS_CONTROLLER_CONTRACT = "PASS_CONTROLLER_CONTRACT"
FAIL_PRECHECK = "FAIL_PRECHECK"
FAIL_CONTROLLER_REJECTED = "FAIL_CONTROLLER_REJECTED"
FAIL_TRACKING_ERROR = "FAIL_TRACKING_ERROR"
FAIL_FEEDBACK_TIMEOUT = "FAIL_FEEDBACK_TIMEOUT"
FAIL_OPERATOR_ABORT = "FAIL_OPERATOR_ABORT"
FAIL_SAFETY_GATE = "FAIL_SAFETY_GATE"
FAIL_LOG_INCOMPLETE = "FAIL_LOG_INCOMPLETE"
INCONCLUSIVE = "INCONCLUSIVE"

FEEDBACK_SOURCE_NONE = "none"
FEEDBACK_SOURCE_FAKE = "fake_controller_simulated"

_FINGER_JOINT = "right_finger_bottom_joint"
_ACTIVE_JOINTS = ("arm_joint_2", "arm_joint_3", "arm_joint_5")


# -- controller runtime seam (fake now, real ROS adapter later) ----------------

@dataclass(frozen=True)
class ControllerJointState:
    joint_names: Tuple[str, ...]
    positions: np.ndarray
    stamp_s: float
    finite: bool = True


@dataclass(frozen=True)
class ControllerFeedbackFrame:
    stamp_s: float
    joint_names: Tuple[str, ...]
    actual_positions: np.ndarray
    result_state: str = "executing"  # executing | succeeded | aborted | rejected
    operator_abort: bool = False
    unexpected_endpoint: str = ""


class ControllerRuntimeInterface(Protocol):
    """Seam between the runner and a controller (fake or real ROS adapter)."""

    def current_joint_state(self) -> ControllerJointState: ...

    def send_goal(self, command: Any) -> bool: ...

    def feedback_frames(self) -> Iterable[ControllerFeedbackFrame]: ...

    def cancel(self) -> bool: ...

    def real_action_client_created(self) -> bool: ...


# -- plan / authorization / config ---------------------------------------------

@dataclass(frozen=True)
class PhysicalDryRunPlan:
    candidate: Any
    command: Any
    preflight_record: Any
    controller_contract: Any
    planned_start_joint_names: Tuple[str, ...]
    planned_start_positions: np.ndarray
    candidate_id: str = ""
    control_artifact_dir: str = ""


@dataclass(frozen=True)
class PhysicalDryRunAuthorization:
    """Default fully closed. The agent constructs only this default."""

    armed_token: str = ""
    user_approved_single_send: bool = False
    allow_action_client: bool = False

    def is_authorized(self, safety_gate: Any) -> bool:
        required = str(getattr(getattr(safety_gate, "config", safety_gate), "required_armed_token", ""))
        token_match = bool(self.armed_token) and str(self.armed_token) == required
        return bool(token_match and self.user_approved_single_send and self.allow_action_client)


@dataclass(frozen=True)
class PhysicalDryRunConfig:
    tracking_error_threshold_rad: float = 0.05
    feedback_timeout_s: float = 0.5
    start_posture_tol_rad: float = 0.05
    finger_open_value: float = 0.0
    finger_hold_tol: float = 1.0e-6


# -- name-based remap helper (Codex 3.5) ---------------------------------------

def remap_by_name(
    names: Sequence[str], positions: Sequence[float] | np.ndarray, target_names: Sequence[str]
) -> Optional[np.ndarray]:
    table = {str(n): float(p) for n, p in zip(names, np.asarray(positions, dtype=np.float64).reshape(-1))}
    out = np.empty(len(target_names), dtype=np.float64)
    for i, name in enumerate(target_names):
        if str(name) not in table:
            return None
        out[i] = table[str(name)]
    return out


# -- abort-class monitor (abort_and_recovery_policy.md A-D) ---------------------

@dataclass(frozen=True)
class MonitorViolation:
    abort_class: str          # "A" | "B" | "C" | "D"
    contract_verdict: str
    detail: str
    threshold: float = 0.0
    observed: float = 0.0


class RuntimeMonitor:
    def __init__(self, config: PhysicalDryRunConfig, *, contract: Any) -> None:
        self.config = config
        self.contract = contract
        self.joint_names = tuple(str(n) for n in contract.joint_names)

    def check_frame(
        self,
        frame: ControllerFeedbackFrame,
        commanded_positions: np.ndarray,
        *,
        last_stamp_s: float,
    ) -> Optional[MonitorViolation]:
        cfg = self.config
        # Class A: operator stop.
        if bool(frame.operator_abort):
            return MonitorViolation("A", FAIL_OPERATOR_ABORT, "operator aborted")
        # Class D: boundary violation (unexpected endpoint, base/cmd_vel, finger close).
        endpoint = str(frame.unexpected_endpoint).strip()
        if endpoint:
            return MonitorViolation("D", FAIL_SAFETY_GATE, f"unexpected endpoint active: {endpoint}")
        actual = remap_by_name(frame.joint_names, frame.actual_positions, self.joint_names)
        if actual is None:
            return MonitorViolation("D", FAIL_SAFETY_GATE, "feedback joint names do not match the contract")
        if _FINGER_JOINT in self.joint_names:
            fi = self.joint_names.index(_FINGER_JOINT)
            if abs(float(actual[fi]) - cfg.finger_open_value) > cfg.finger_hold_tol:
                return MonitorViolation(
                    "D", FAIL_SAFETY_GATE, "finger left the open hold value",
                    cfg.finger_hold_tol, abs(float(actual[fi]) - cfg.finger_open_value),
                )
        # Class B: controller terminal failure / feedback timeout.
        state = str(frame.result_state).lower()
        if state in {"aborted", "rejected"}:
            return MonitorViolation("B", FAIL_CONTROLLER_REJECTED, f"controller result {state}")
        gap = float(frame.stamp_s) - float(last_stamp_s)
        if gap > cfg.feedback_timeout_s + 1.0e-12:
            return MonitorViolation("B", FAIL_FEEDBACK_TIMEOUT, "feedback timeout", cfg.feedback_timeout_s, gap)
        # Class C: tracking safety (non-finite, tracking error).
        if not np.all(np.isfinite(actual)):
            return MonitorViolation("C", FAIL_TRACKING_ERROR, "non-finite joint state")
        active_idx = [self.joint_names.index(j) for j in _ACTIVE_JOINTS if j in self.joint_names]
        if active_idx:
            err = np.abs(actual[active_idx] - np.asarray(commanded_positions, dtype=np.float64)[active_idx])
            max_err = float(np.max(err)) if err.size else 0.0
            if max_err > cfg.tracking_error_threshold_rad + 1.0e-12:
                return MonitorViolation(
                    "C", FAIL_TRACKING_ERROR, "tracking error exceeded",
                    cfg.tracking_error_threshold_rad, max_err,
                )
        return None


# -- result --------------------------------------------------------------------

@dataclass
class PhysicalDryRunResult:
    runner_readiness_verdict: str
    contract_verdict: str
    feedback_source: str
    physical: bool
    goal_sent: bool
    goal_accepted: bool
    feedback_received: bool
    cancel_sent: bool
    controller_succeeded: bool
    abort_class: str
    max_tracking_error_rad: float
    real_action_client_created: bool
    log_dir: str
    evidence: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "runner_readiness_verdict": self.runner_readiness_verdict,
            "contract_verdict": self.contract_verdict,
            "feedback_source": self.feedback_source,
            "physical": bool(self.physical),
            "goal_sent": bool(self.goal_sent),
            "goal_accepted": bool(self.goal_accepted),
            "feedback_received": bool(self.feedback_received),
            "cancel_sent": bool(self.cancel_sent),
            "controller_succeeded": bool(self.controller_succeeded),
            "abort_class": self.abort_class,
            "max_tracking_error_rad": float(self.max_tracking_error_rad),
            "real_action_client_created": bool(self.real_action_client_created),
            "log_dir": self.log_dir,
            "evidence": self.evidence,
        }


# -- logger (log-schema sections 1-8 + five CSVs) ------------------------------

class DryRunLogger:
    FEEDBACK_HEADER = ["timestamp_s", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "finger"]
    TRACKING_HEADER = ["timestamp_s", "max_abs_error_rad", "joint_name", "error_rad", "threshold_rad"]
    COMMANDED_HEADER = ["sample_index", "time_from_start_s", "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "finger"]
    CONTROLLER_EVENTS_HEADER = ["timestamp_s", "event_type", "status_code", "message"]
    SAFETY_EVENTS_HEADER = ["timestamp_s", "event_type", "triggered", "threshold", "observed_value", "action_taken"]

    def __init__(self, out_dir: str | Path) -> None:
        self.dir = Path(out_dir)
        self.dir.mkdir(parents=True, exist_ok=True)

    def _write_csv(self, name: str, header: List[str], rows: Sequence[Sequence[Any]]) -> None:
        with (self.dir / name).open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(header)
            for r in rows:
                w.writerow(list(r))

    def write_commanded(self, command: Any) -> None:
        rows = []
        for i, p in enumerate(command.points):
            rows.append([i, float(p.time_from_start_s), *[float(v) for v in p.positions.tolist()]])
        self._write_csv("commanded_joint_trajectory.csv", self.COMMANDED_HEADER, rows)

    def write_feedback(self, rows: Sequence[Sequence[Any]]) -> None:
        self._write_csv("feedback_joint_trajectory.csv", self.FEEDBACK_HEADER, rows)

    def write_tracking(self, rows: Sequence[Sequence[Any]]) -> None:
        self._write_csv("tracking_error.csv", self.TRACKING_HEADER, rows)

    def write_controller_events(self, rows: Sequence[Sequence[Any]]) -> None:
        self._write_csv("controller_events.csv", self.CONTROLLER_EVENTS_HEADER, rows)

    def write_safety_events(self, rows: Sequence[Sequence[Any]]) -> None:
        self._write_csv("safety_events.csv", self.SAFETY_EVENTS_HEADER, rows)

    def write_evidence(self, evidence: Dict[str, Any]) -> None:
        (self.dir / "evidence.json").write_text(json.dumps(evidence, indent=2, ensure_ascii=False), encoding="utf-8")

    @property
    def csv_names(self) -> List[str]:
        return [
            "commanded_joint_trajectory.csv",
            "feedback_joint_trajectory.csv",
            "tracking_error.csv",
            "controller_events.csv",
            "safety_events.csv",
        ]

    def all_csvs_present(self) -> bool:
        return all((self.dir / n).exists() for n in self.csv_names)


# -- runner --------------------------------------------------------------------

class PhysicalDryRunRunner:
    """Check-only by default; one gated send; monitor; log; verdict."""

    def __init__(self, config: Optional[PhysicalDryRunConfig] = None) -> None:
        self.config = config if config is not None else PhysicalDryRunConfig()
        self._sent_once = False

    def run(
        self,
        plan: PhysicalDryRunPlan,
        *,
        authorization: PhysicalDryRunAuthorization,
        controller: ControllerRuntimeInterface,
        safety_gate: Any,
        out_dir: str | Path,
    ) -> PhysicalDryRunResult:
        cfg = self.config
        logger = DryRunLogger(out_dir)
        logger.write_commanded(plan.command)
        controller_events: List[List[Any]] = []
        safety_events: List[List[Any]] = []

        def finalize(
            readiness: str, contract: str, *, feedback_source: str = FEEDBACK_SOURCE_NONE,
            goal_sent: bool = False, goal_accepted: bool = False, feedback_received: bool = False,
            cancel_sent: bool = False, controller_succeeded: bool = False, abort_class: str = "",
            max_err: float = 0.0, feedback_rows: Optional[List[List[Any]]] = None,
            tracking_rows: Optional[List[List[Any]]] = None,
        ) -> PhysicalDryRunResult:
            logger.write_feedback(feedback_rows or [])
            logger.write_tracking(tracking_rows or [])
            logger.write_controller_events(controller_events)
            logger.write_safety_events(safety_events)
            real_client = bool(controller.real_action_client_created())
            logs_complete = logger.all_csvs_present()
            if not logs_complete and contract == PASS_CONTROLLER_CONTRACT:
                contract = FAIL_LOG_INCOMPLETE
            evidence = {
                "run_metadata": {
                    "objective": "object_free_controller_contract",
                    "runner_readiness_verdict": readiness,
                    "contract_verdict": contract,
                    "physical": False,
                },
                "artifact_identity": {
                    "candidate_id": plan.candidate_id,
                    "control_artifact_dir": plan.control_artifact_dir,
                    "map_or_contract": list(plan.controller_contract.joint_names),
                },
                "safety_boundary": {
                    "base_motion_allowed": False,
                    "cmd_vel_allowed": False,
                    "gripper_close_allowed": False,
                    "single_send_only": True,
                    "no_real_action_client": not real_client,
                },
                "controller_contract": {
                    "joint_names": list(plan.command.joint_names),
                    "partial_goal_allowed": bool(plan.controller_contract.allow_partial_joints_goal),
                    "trajectory_sample_count": len(plan.command.points),
                },
                "trajectory_limits": (
                    plan.preflight_record.as_dict() if hasattr(plan.preflight_record, "as_dict") else {}
                ),
                "result_summary": {
                    "goal_sent": bool(goal_sent),
                    "goal_accepted": bool(goal_accepted),
                    "feedback_received": bool(feedback_received),
                    "cancel_sent": bool(cancel_sent),
                    "controller_succeeded": bool(controller_succeeded),
                    "max_tracking_error_rad": float(max_err),
                    "tracking_error_threshold_rad": cfg.tracking_error_threshold_rad,
                    "feedback_source": feedback_source,
                    "abort_class": abort_class,
                },
                "verdict": {
                    "runner_readiness_verdict": readiness,
                    "contract_verdict": contract,
                    "physical": False,
                },
                "logs_complete": bool(logs_complete),
            }
            logger.write_evidence(evidence)
            return PhysicalDryRunResult(
                runner_readiness_verdict=readiness,
                contract_verdict=contract,
                feedback_source=feedback_source,
                physical=False,
                goal_sent=goal_sent,
                goal_accepted=goal_accepted,
                feedback_received=feedback_received,
                cancel_sent=cancel_sent,
                controller_succeeded=controller_succeeded,
                abort_class=abort_class,
                max_tracking_error_rad=float(max_err),
                real_action_client_created=real_client,
                log_dir=str(logger.dir),
                evidence=evidence,
            )

        # 1) preflight must be PASS-eligible.
        if not bool(getattr(plan.preflight_record, "passed", False)):
            safety_events.append([0.0, "preflight", True, 0.0, 0.0, "stop_no_send"])
            return finalize(NOT_READY_PREFLIGHT_FAIL, FAIL_PRECHECK)

        # 2) name-based start-posture check.
        cur = controller.current_joint_state()
        mapped = remap_by_name(cur.joint_names, cur.positions, plan.planned_start_joint_names)
        planned = np.asarray(plan.planned_start_positions, dtype=np.float64).reshape(-1)
        posture_ok = bool(
            cur.finite and mapped is not None and np.all(np.isfinite(mapped))
            and mapped.shape[0] == planned.shape[0]
            and float(np.max(np.abs(mapped - planned))) <= cfg.start_posture_tol_rad + 1.0e-12
        )
        if not posture_ok:
            safety_events.append([float(cur.stamp_s), "start_posture_mismatch", True, cfg.start_posture_tol_rad, 0.0, "stop_no_send"])
            return finalize(NOT_READY_POSTURE_FAIL, FAIL_PRECHECK)

        # 3) authorization gate (runner-owned, independent of the safety gate's fake rule).
        if not authorization.is_authorized(safety_gate):
            controller_events.append([0.0, "authorization", "closed", "check_only_no_send"])
            return finalize(CHECK_ONLY_READY_NO_SEND, NOT_APPLICABLE_UNTIL_SEND, feedback_source=FEEDBACK_SOURCE_NONE)

        # 4) one-send rule.
        if self._sent_once:
            controller_events.append([0.0, "one_send_rule", "refused", "second_send_blocked"])
            return finalize(SECOND_SEND_REFUSED, INCONCLUSIVE)
        self._sent_once = True
        accepted = bool(controller.send_goal(plan.command))
        controller_events.append([0.0, "goal_sent", "accepted" if accepted else "rejected", "single_send"])
        if not accepted:
            cancelled = bool(controller.cancel())
            return finalize(
                AUTHORIZED_SENT, FAIL_CONTROLLER_REJECTED, feedback_source=FEEDBACK_SOURCE_FAKE,
                goal_sent=True, goal_accepted=False, cancel_sent=cancelled, abort_class="B",
            )

        # 5) monitor loop over simulated feedback.
        monitor = RuntimeMonitor(cfg, contract=plan.controller_contract)
        points = list(plan.command.points)
        feedback_rows: List[List[Any]] = []
        tracking_rows: List[List[Any]] = []
        last_stamp = 0.0
        max_err = 0.0
        abort: Optional[MonitorViolation] = None
        controller_succeeded = False
        feedback_received = False
        for i, frame in enumerate(controller.feedback_frames()):
            feedback_received = True
            commanded = points[min(i, len(points) - 1)].positions
            actual_mapped = remap_by_name(frame.joint_names, frame.actual_positions, plan.command.joint_names)
            # feedback CSV row (timestamp + 7 joints), simulated provenance.
            if actual_mapped is not None and np.all(np.isfinite(actual_mapped)):
                feedback_rows.append([float(frame.stamp_s), *[float(v) for v in actual_mapped.tolist()]])
                active_idx = [plan.command.joint_names.index(j) for j in _ACTIVE_JOINTS if j in plan.command.joint_names]
                if active_idx:
                    err = float(np.max(np.abs(actual_mapped[active_idx] - np.asarray(commanded)[active_idx])))
                    max_err = max(max_err, err)
                    tracking_rows.append([float(frame.stamp_s), err, "active", err, cfg.tracking_error_threshold_rad])
            else:
                feedback_rows.append([float(frame.stamp_s), *["nan"] * 7])
            v = monitor.check_frame(frame, np.asarray(commanded, dtype=np.float64), last_stamp_s=last_stamp)
            if v is not None:
                abort = v
                cancelled = bool(controller.cancel())
                safety_events.append([float(frame.stamp_s), f"abort_class_{v.abort_class}", True, float(v.threshold), float(v.observed), "cancel_sent"])
                controller_events.append([float(frame.stamp_s), "cancel", "requested", v.detail])
                return finalize(
                    AUTHORIZED_SENT, v.contract_verdict, feedback_source=FEEDBACK_SOURCE_FAKE,
                    goal_sent=True, goal_accepted=True, feedback_received=True, cancel_sent=cancelled,
                    abort_class=v.abort_class, max_err=max_err,
                    feedback_rows=feedback_rows, tracking_rows=tracking_rows,
                )
            last_stamp = float(frame.stamp_s)
            if str(frame.result_state).lower() == "succeeded":
                controller_succeeded = True
                controller_events.append([float(frame.stamp_s), "result", "succeeded", "controller_success"])
                break

        # 6) verdict.
        if controller_succeeded and feedback_received:
            contract = PASS_CONTROLLER_CONTRACT
        else:
            contract = INCONCLUSIVE
        return finalize(
            AUTHORIZED_SENT, contract, feedback_source=FEEDBACK_SOURCE_FAKE,
            goal_sent=True, goal_accepted=True, feedback_received=feedback_received,
            controller_succeeded=controller_succeeded, max_err=max_err,
            feedback_rows=feedback_rows, tracking_rows=tracking_rows,
        )
