"""Offline pre-physical trajectory preflight (no-command).

This module machine-checks the existing ``safety/`` contract's §3-§4 trajectory
requirements and the agent-side of §5 (arming) against the real no-command
``ExecutionCandidate`` / built ``JointTrajectoryCommand``.  It NEVER moves the
robot, arms the system, creates a real action client, sends a goal, or exports
the armed token.  It only answers, offline, the contract's trajectory-preflight
question and proves the safety gate blocks any real send without the token.

Authority split (safety/physical_dry_run_contract.md §5): the agent MAY prepare
the run checklist and analyze artifacts; the agent MUST NOT arm or trigger motion.
The actual workspace/robot preconditions, real arming, live monitoring, and human
signoffs remain MANUAL and are out of scope here.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .execution_supervisor import ExecutionCandidate
from .joint_mapping import ControllerContract
from .safety_gate import ExecutionMode, ExecutionSafetyConfig, ExecutionSafetyGate, SafetyViolation

try:  # constants live under control_module; support both import roots.
    from control_module.constants import (
        JL_HI_ACTIVE,
        JL_LO_ACTIVE,
        Q1_FIXED,
        Q4_FIXED,
        Q6_FIXED,
    )
except Exception:  # pragma: no cover - direct execution fallback
    from constants import JL_HI_ACTIVE, JL_LO_ACTIVE, Q1_FIXED, Q4_FIXED, Q6_FIXED  # type: ignore


# Verdict eligibility strings (trajectory-preflight subset of the contract §9 verdicts).
VERDICT_PASS_TRAJECTORY_PREFLIGHT = "PASS_CONTROLLER_CONTRACT_TRAJECTORY_PREFLIGHT"
VERDICT_FAIL_PRECHECK = "FAIL_PRECHECK"

# Fixed full-chain joints (held by the reduced active manifold) and the open finger.
_FIXED_JOINTS = (
    ("arm_joint_1", float(Q1_FIXED)),
    ("arm_joint_4", float(Q4_FIXED)),
    ("arm_joint_6", float(Q6_FIXED)),
)
_FINGER_JOINT = "right_finger_bottom_joint"
_ACTIVE_JOINTS = ("arm_joint_2", "arm_joint_3", "arm_joint_5")


@dataclass(frozen=True)
class PrePhysicalPreflightConfig:
    """Conservative thresholds for the first object-free controller-contract test."""

    min_execution_dt_s: float = 0.1
    dt_uniform_tol_s: float = 1.0e-6
    max_velocity_rad_s: float = 0.5
    max_acceleration_rad_s2: float = 0.4
    max_tracking_step_rad: float = 0.1
    max_ee_displacement_m: float = 0.1
    finger_open_value: float = 0.0
    finger_hold_tol: float = 1.0e-9
    fixed_joint_tol: float = 1.0e-9
    active_limit_slack: float = 1.0e-9


@dataclass
class TrajectoryPreflightRecord:
    """Structured, log-schema (§4/§5) compatible preflight outcome."""

    joint_names: Tuple[str, ...]
    trajectory_sample_count: int
    execution_dt_s: float
    first_time_from_start_s: float
    last_time_from_start_s: float
    uses_control_internal_dt_as_execution_dt: bool
    velocity_cap_rad_s: float
    acceleration_cap_rad_s2: float
    max_commanded_velocity_rad_s: float
    max_commanded_acceleration_rad_s2: float
    max_predicted_tracking_step_rad: float
    estimated_ee_displacement_m: float
    net_ee_displacement_m: float
    finger_policy: str
    finger_start: float
    finger_end: float
    partial_goal_allowed: bool
    checks: Dict[str, bool] = field(default_factory=dict)
    failures: List[str] = field(default_factory=list)

    @property
    def passed(self) -> bool:
        return len(self.failures) == 0 and all(self.checks.values())

    @property
    def verdict_eligibility(self) -> str:
        return VERDICT_PASS_TRAJECTORY_PREFLIGHT if self.passed else VERDICT_FAIL_PRECHECK

    def as_dict(self) -> Dict[str, Any]:
        return {
            "joint_names": list(self.joint_names),
            "trajectory_sample_count": int(self.trajectory_sample_count),
            "execution_dt_s": float(self.execution_dt_s),
            "first_time_from_start_s": float(self.first_time_from_start_s),
            "last_time_from_start_s": float(self.last_time_from_start_s),
            "uses_control_internal_dt_as_execution_dt": bool(self.uses_control_internal_dt_as_execution_dt),
            "velocity_cap_rad_s": float(self.velocity_cap_rad_s),
            "acceleration_cap_rad_s2": float(self.acceleration_cap_rad_s2),
            "max_commanded_velocity_rad_s": float(self.max_commanded_velocity_rad_s),
            "max_commanded_acceleration_rad_s2": float(self.max_commanded_acceleration_rad_s2),
            "max_predicted_tracking_step_rad": float(self.max_predicted_tracking_step_rad),
            "estimated_ee_displacement_m": float(self.estimated_ee_displacement_m),
            "net_ee_displacement_m": float(self.net_ee_displacement_m),
            "finger_policy": str(self.finger_policy),
            "finger_start": float(self.finger_start),
            "finger_end": float(self.finger_end),
            "partial_goal_allowed": bool(self.partial_goal_allowed),
            "checks": {str(k): bool(v) for k, v in self.checks.items()},
            "failures": list(self.failures),
            "passed": bool(self.passed),
            "verdict_eligibility": self.verdict_eligibility,
        }


@dataclass
class ConformingSegmentResult:
    candidate: Optional[ExecutionCandidate]
    n_points: int
    path_length_m: float
    reason: str


def _ee_cumulative_path_length(xz_table: np.ndarray) -> np.ndarray:
    xz = np.asarray(xz_table, dtype=np.float64).reshape(-1, 2)
    if xz.shape[0] <= 1:
        return np.zeros(max(xz.shape[0], 1), dtype=np.float64)
    seg = np.linalg.norm(np.diff(xz, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


class PrePhysicalTrajectoryPreflight:
    """Offline machine-checker for the safety contract trajectory + agent-arming preflight."""

    def __init__(self, config: Optional[PrePhysicalPreflightConfig] = None) -> None:
        self.config = config if config is not None else PrePhysicalPreflightConfig()

    # -- trajectory preflight --------------------------------------------------

    def evaluate(
        self,
        command: Any,
        candidate: ExecutionCandidate,
        *,
        contract: ControllerContract,
    ) -> TrajectoryPreflightRecord:
        cfg = self.config
        joint_names = tuple(str(n) for n in command.joint_names)
        points = list(command.points)
        positions = np.asarray([np.asarray(p.positions, dtype=np.float64) for p in points], dtype=np.float64)
        times = np.asarray([float(p.time_from_start_s) for p in points], dtype=np.float64)
        n = positions.shape[0]

        checks: Dict[str, bool] = {}
        failures: List[str] = []

        def fail(name: str, ok: bool, reason: str) -> None:
            checks[name] = bool(ok)
            if not ok:
                failures.append(reason)

        # joint contract (names, order, count, partial policy) -------------------
        contract_names = tuple(str(n) for n in contract.joint_names)
        fail("joint_names_match_contract", joint_names == contract_names,
             f"joint_names {joint_names} != contract {contract_names}")
        fail("partial_goal_disallowed", not bool(contract.allow_partial_joints_goal),
             "controller contract allows partial/reordered joint goals")
        index = {name: i for i, name in enumerate(joint_names)}
        has_all = all(name in index for name in (_FINGER_JOINT, *[j for j, _ in _FIXED_JOINTS], *_ACTIVE_JOINTS))
        fail("all_required_joints_present", has_all, "command is missing required joint columns")

        # cadence / retiming -----------------------------------------------------
        if n >= 2:
            dt = np.diff(times)
            execution_dt_s = float(np.median(dt))
            uniform = bool(np.max(np.abs(dt - execution_dt_s)) <= cfg.dt_uniform_tol_s)
            positive = bool(np.all(dt > 0.0))
            at_least = bool(execution_dt_s >= cfg.min_execution_dt_s - 1.0e-12)
        else:
            dt = np.zeros(0)
            execution_dt_s = float(getattr(candidate, "dt_s", 0.0) or 0.0)
            uniform = False
            positive = False
            at_least = False
        uses_internal_dt = not (uniform and positive and at_least)
        fail("cadence_uniform", uniform, "trajectory cadence is not uniform")
        fail("cadence_strictly_increasing", positive, "trajectory time_from_start is not strictly increasing")
        fail("execution_dt_at_least_min", at_least,
             f"execution_dt_s {execution_dt_s:.6f} < contract minimum {cfg.min_execution_dt_s}")
        fail("not_control_internal_dt", not uses_internal_dt,
             "execution dt looks like the control internal step, not a retimed hardware cadence")

        # object-free finger (hold-only) ----------------------------------------
        if _FINGER_JOINT in index and n >= 1:
            finger = positions[:, index[_FINGER_JOINT]]
            finger_start = float(finger[0])
            finger_end = float(finger[-1])
            held = bool(np.max(np.abs(finger - cfg.finger_open_value)) <= cfg.finger_hold_tol)
        else:
            finger_start = float("nan")
            finger_end = float("nan")
            held = False
        fail("finger_hold_open_object_free", held,
             "finger joint is not held constant at the open value (object-free violated)")

        # fixed posture (arm_joint_1/4/6) ---------------------------------------
        fixed_ok = True
        for jname, jval in _FIXED_JOINTS:
            if jname not in index or n < 1:
                fixed_ok = False
                continue
            col = positions[:, index[jname]]
            if not bool(np.max(np.abs(col - jval)) <= cfg.fixed_joint_tol):
                fixed_ok = False
        fail("fixed_posture_constant", fixed_ok, "fixed joints (1/4/6) are not constant at their fixed values")

        # active joint limits ----------------------------------------------------
        active_ok = True
        if all(j in index for j in _ACTIVE_JOINTS) and n >= 1:
            lo = np.asarray(JL_LO_ACTIVE, dtype=np.float64).reshape(3)
            hi = np.asarray(JL_HI_ACTIVE, dtype=np.float64).reshape(3)
            active = positions[:, [index[j] for j in _ACTIVE_JOINTS]]
            active_ok = bool(
                np.all(active >= lo[None, :] - cfg.active_limit_slack)
                and np.all(active <= hi[None, :] + cfg.active_limit_slack)
            )
        else:
            active_ok = False
        fail("active_joints_within_limits", active_ok, "active joints violate the active joint limits")

        # velocity / acceleration / tracking step --------------------------------
        max_vel = 0.0
        max_acc = 0.0
        max_step = 0.0
        if n >= 2:
            dq = np.diff(positions, axis=0)
            max_step = float(np.max(np.abs(dq))) if dq.size else 0.0
            with np.errstate(divide="ignore", invalid="ignore"):
                vel = dq / dt[:, None]
            max_vel = float(np.max(np.abs(vel))) if vel.size else 0.0
            if n >= 3:
                acc_dt = 0.5 * (dt[:-1] + dt[1:])
                acc = np.diff(vel, axis=0) / acc_dt[:, None]
                max_acc = float(np.max(np.abs(acc))) if acc.size else 0.0
        fail("velocity_within_cap", max_vel <= cfg.max_velocity_rad_s + 1.0e-12,
             f"max commanded velocity {max_vel:.4f} > cap {cfg.max_velocity_rad_s}")
        fail("acceleration_within_cap", max_acc <= cfg.max_acceleration_rad_s2 + 1.0e-12,
             f"max commanded acceleration {max_acc:.4f} > cap {cfg.max_acceleration_rad_s2}")
        fail("tracking_step_within_cap", max_step <= cfg.max_tracking_step_rad + 1.0e-12,
             f"max joint step {max_step:.4f} > cap {cfg.max_tracking_step_rad}")

        # EE displacement (xz path-length proxy; conservative >= net) -------------
        cum = _ee_cumulative_path_length(candidate.xz_table)
        est_ee = float(cum[-1]) if cum.size else 0.0
        xz = np.asarray(candidate.xz_table, dtype=np.float64).reshape(-1, 2)
        net_ee = float(np.linalg.norm(xz[-1] - xz[0])) if xz.shape[0] >= 2 else 0.0
        fail("ee_displacement_within_cap", est_ee <= cfg.max_ee_displacement_m + 1.0e-12,
             f"estimated EE displacement {est_ee:.4f} m > cap {cfg.max_ee_displacement_m} m")

        return TrajectoryPreflightRecord(
            joint_names=joint_names,
            trajectory_sample_count=int(n),
            execution_dt_s=float(execution_dt_s),
            first_time_from_start_s=float(times[0]) if n else 0.0,
            last_time_from_start_s=float(times[-1]) if n else 0.0,
            uses_control_internal_dt_as_execution_dt=bool(uses_internal_dt),
            velocity_cap_rad_s=float(cfg.max_velocity_rad_s),
            acceleration_cap_rad_s2=float(cfg.max_acceleration_rad_s2),
            max_commanded_velocity_rad_s=float(max_vel),
            max_commanded_acceleration_rad_s2=float(max_acc),
            max_predicted_tracking_step_rad=float(max_step),
            estimated_ee_displacement_m=float(est_ee),
            net_ee_displacement_m=float(net_ee),
            finger_policy="hold",
            finger_start=float(finger_start),
            finger_end=float(finger_end),
            partial_goal_allowed=bool(contract.allow_partial_joints_goal),
            checks=checks,
            failures=failures,
        )

    # -- contract-conforming short leading segment ----------------------------

    def conforming_leading_segment(
        self,
        candidate: ExecutionCandidate,
        *,
        max_ee_displacement_m: Optional[float] = None,
    ) -> ConformingSegmentResult:
        cap = float(self.config.max_ee_displacement_m if max_ee_displacement_m is None else max_ee_displacement_m)
        q = np.asarray(candidate.q_active_trajectory, dtype=np.float64).reshape(-1, 3)
        s = np.asarray(candidate.s_table, dtype=np.float64).reshape(-1)
        xz = np.asarray(candidate.xz_table, dtype=np.float64).reshape(-1, 2)
        cum = _ee_cumulative_path_length(xz)
        n = q.shape[0]
        if n < 2:
            return ConformingSegmentResult(None, n, 0.0, "candidate has fewer than two points")
        # largest leading prefix (>=2 points) whose cumulative EE path length <= cap.
        k = 1
        while k + 1 < n and float(cum[k + 1]) <= cap + 1.0e-12:
            k += 1
        if float(cum[k]) > cap + 1.0e-12:
            return ConformingSegmentResult(
                None, 2, float(cum[1]),
                f"even the first segment ({cum[1]:.4f} m) exceeds the EE cap {cap} m",
            )
        m = k + 1
        seg = ExecutionCandidate(
            q_active_trajectory=q[:m].copy(),
            s_table=s[:m].copy(),
            xz_table=xz[:m].copy(),
            dt_s=candidate.dt_s,
            candidate_metrics={"candidate_accepted": True, "source": "conforming_leading_segment"},
            metadata={
                **dict(candidate.metadata),
                "conforming_leading_segment": True,
                "segment_point_count": int(m),
                "segment_ee_path_length_m": float(cum[k]),
                "ee_cap_m": cap,
            },
        )
        return ConformingSegmentResult(seg, int(m), float(cum[k]), "ok")

    # -- offline arming-gate proof (no real send) -----------------------------

    @staticmethod
    def assert_no_command_offline(safety_gate: ExecutionSafetyGate) -> Dict[str, Any]:
        config = safety_gate.config
        action = str(config.allowed_action_name)
        real_send_blocked = False
        block_reason = ""
        try:
            safety_gate.assert_can_send_goal(action, fake=False)
        except SafetyViolation as exc:
            real_send_blocked = True
            block_reason = str(exc)
        token_hardcoded_as_armed = bool(
            str(config.mode) == ExecutionMode.ARMED
            and bool(config.armed)
            and str(config.armed_token) == str(config.required_armed_token)
        )
        return {
            "real_send_blocked_without_token": bool(real_send_blocked),
            "block_reason": block_reason,
            "mode_is_not_armed": bool(str(config.mode) != ExecutionMode.ARMED),
            "armed_flag_false": bool(not config.armed),
            "armed_token_not_hardcoded_as_armed": bool(not token_hardcoded_as_armed),
            "allow_real_action_client_false": bool(not config.allow_real_action_client),
            "agent_exported_token": False,
        }

    # -- log-schema emission ---------------------------------------------------

    @staticmethod
    def emit_log_schema_sections(
        record: TrajectoryPreflightRecord,
        *,
        contract: ControllerContract,
        candidate_id: str = "",
        control_artifact_dir: str = "",
    ) -> Dict[str, Any]:
        controller_contract = {
            "command_interface": list(contract.command_interfaces)[0] if contract.command_interfaces else "",
            "joint_names": list(record.joint_names),
            "partial_goal_allowed": bool(contract.allow_partial_joints_goal),
            "controller_update_rate_hz": float(contract.update_rate_hz),
            "state_publish_rate_hz": float(contract.state_publish_rate_hz),
            "interpolator": "",
            "trajectory_sample_count": int(record.trajectory_sample_count),
            "execution_dt_s": float(record.execution_dt_s),
            "first_time_from_start_s": float(record.first_time_from_start_s),
            "last_time_from_start_s": float(record.last_time_from_start_s),
            "uses_control_internal_dt_as_execution_dt": bool(record.uses_control_internal_dt_as_execution_dt),
        }
        trajectory_limits = {
            "velocity_cap_rad_s": float(record.velocity_cap_rad_s),
            "acceleration_cap_rad_s2": float(record.acceleration_cap_rad_s2),
            "max_commanded_velocity_rad_s": float(record.max_commanded_velocity_rad_s),
            "max_commanded_acceleration_rad_s2": float(record.max_commanded_acceleration_rad_s2),
            "max_predicted_tracking_step_rad": float(record.max_predicted_tracking_step_rad),
            "estimated_ee_displacement_m": float(record.estimated_ee_displacement_m),
            "finger_policy": str(record.finger_policy),
            "finger_start": float(record.finger_start),
            "finger_end": float(record.finger_end),
        }
        return {
            "candidate_id": str(candidate_id),
            "control_artifact_dir": str(control_artifact_dir),
            "controller_contract": controller_contract,
            "trajectory_limits": trajectory_limits,
            "verdict_eligibility": record.verdict_eligibility,
            "passed": bool(record.passed),
            "failures": list(record.failures),
        }


def default_no_token_gate() -> ExecutionSafetyGate:
    """A dry-run gate with no armed token (the offline default; cannot send real goals)."""

    return ExecutionSafetyGate(ExecutionSafetyConfig(mode=ExecutionMode.DRY_RUN))
