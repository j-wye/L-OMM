from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Sequence, Tuple

import numpy as np

try:
    from control_module.constants import (
        JL_HI_ACTIVE,
        JL_LO_ACTIVE,
        Q1_FIXED,
        Q4_FIXED,
        Q6_FIXED,
    )
except Exception:
    from constants import JL_HI_ACTIVE, JL_LO_ACTIVE, Q1_FIXED, Q4_FIXED, Q6_FIXED


class JointMappingError(ValueError):
    """Raised when reduced active joints cannot be mapped safely."""


DEFAULT_CONTROLLER_JOINTS: Tuple[str, ...] = (
    "arm_joint_1",
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5",
    "arm_joint_6",
    "right_finger_bottom_joint",
)


@dataclass(frozen=True)
class ControllerContract:
    joint_names: Tuple[str, ...] = DEFAULT_CONTROLLER_JOINTS
    command_interfaces: Tuple[str, ...] = ("position",)
    state_interfaces: Tuple[str, ...] = ("position", "velocity")
    allow_partial_joints_goal: bool = False
    state_publish_rate_hz: float = 100.0
    update_rate_hz: float = 100.0
    action_name: str = "/arm_controller/follow_joint_trajectory"

    def __init__(
        self,
        joint_names: Sequence[str] = DEFAULT_CONTROLLER_JOINTS,
        command_interfaces: Sequence[str] = ("position",),
        state_interfaces: Sequence[str] = ("position", "velocity"),
        allow_partial_joints_goal: bool = False,
        state_publish_rate_hz: float = 100.0,
        update_rate_hz: float = 100.0,
        action_name: str = "/arm_controller/follow_joint_trajectory",
    ) -> None:
        object.__setattr__(self, "joint_names", tuple(str(v) for v in joint_names))
        object.__setattr__(self, "command_interfaces", tuple(str(v) for v in command_interfaces))
        object.__setattr__(self, "state_interfaces", tuple(str(v) for v in state_interfaces))
        object.__setattr__(self, "allow_partial_joints_goal", bool(allow_partial_joints_goal))
        object.__setattr__(self, "state_publish_rate_hz", float(state_publish_rate_hz))
        object.__setattr__(self, "update_rate_hz", float(update_rate_hz))
        object.__setattr__(self, "action_name", str(action_name))
        self._validate()

    def _validate(self) -> None:
        if len(self.joint_names) != len(set(self.joint_names)):
            raise JointMappingError("controller joint names must be unique")
        if "position" not in self.command_interfaces:
            raise JointMappingError("execution harness requires a position command interface")
        missing = [name for name in DEFAULT_CONTROLLER_JOINTS if name not in self.joint_names]
        if missing:
            raise JointMappingError(f"controller contract missing required joints: {missing}")

    def as_dict(self) -> Dict[str, object]:
        return {
            "joint_names": list(self.joint_names),
            "command_interfaces": list(self.command_interfaces),
            "state_interfaces": list(self.state_interfaces),
            "allow_partial_joints_goal": bool(self.allow_partial_joints_goal),
            "state_publish_rate_hz": float(self.state_publish_rate_hz),
            "update_rate_hz": float(self.update_rate_hz),
            "action_name": self.action_name,
        }


@dataclass(frozen=True)
class FixedPosture:
    q1: float = float(Q1_FIXED)
    q4: float = float(Q4_FIXED)
    q6: float = float(Q6_FIXED)
    finger: float = 0.0

    def as_dict(self) -> Dict[str, float]:
        return {
            "arm_joint_1": float(self.q1),
            "arm_joint_4": float(self.q4),
            "arm_joint_6": float(self.q6),
            "right_finger_bottom_joint": float(self.finger),
        }


@dataclass(frozen=True)
class FullJointVector:
    joint_names: Tuple[str, ...]
    positions: np.ndarray

    def as_dict(self) -> Dict[str, object]:
        return {
            "joint_names": list(self.joint_names),
            "positions": [float(v) for v in self.positions.tolist()],
        }


class ReducedJointMapper:
    """Lift q_act = [q2, q3, q5] into the Jetson controller joint order."""

    def __init__(self, contract: ControllerContract | None = None, fixed_posture: FixedPosture | None = None) -> None:
        self.contract = contract if contract is not None else ControllerContract()
        self.fixed_posture = fixed_posture if fixed_posture is not None else FixedPosture()

    def full_from_active(self, q_active: Sequence[float] | np.ndarray) -> FullJointVector:
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        if q.shape[0] != 3 or not np.all(np.isfinite(q)):
            raise JointMappingError("q_active must be finite length-3 [q2, q3, q5]")
        lo = np.asarray(JL_LO_ACTIVE, dtype=np.float64).reshape(3)
        hi = np.asarray(JL_HI_ACTIVE, dtype=np.float64).reshape(3)
        if np.any(q < lo - 1.0e-12) or np.any(q > hi + 1.0e-12):
            raise JointMappingError("q_active violates active joint limits")
        values = {
            "arm_joint_1": float(self.fixed_posture.q1),
            "arm_joint_2": float(q[0]),
            "arm_joint_3": float(q[1]),
            "arm_joint_4": float(self.fixed_posture.q4),
            "arm_joint_5": float(q[2]),
            "arm_joint_6": float(self.fixed_posture.q6),
            "right_finger_bottom_joint": float(self.fixed_posture.finger),
        }
        positions = np.asarray([values[name] for name in self.contract.joint_names], dtype=np.float64)
        return FullJointVector(joint_names=self.contract.joint_names, positions=positions)

    def active_from_joint_state(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float] | np.ndarray,
    ) -> np.ndarray:
        names = tuple(str(v) for v in joint_names)
        if len(names) != len(set(names)):
            raise JointMappingError("joint state contains duplicate joint names")
        pos = np.asarray(positions, dtype=np.float64).reshape(-1)
        if pos.shape[0] != len(names) or not np.all(np.isfinite(pos)):
            raise JointMappingError("joint state positions must be finite and match joint names")
        table = {name: float(pos[idx]) for idx, name in enumerate(names)}
        required = ("arm_joint_2", "arm_joint_3", "arm_joint_5")
        missing = [name for name in required if name not in table]
        if missing:
            raise JointMappingError(f"joint state missing active joints: {missing}")
        return np.asarray([table["arm_joint_2"], table["arm_joint_3"], table["arm_joint_5"]], dtype=np.float64)

    def validate_goal_joint_names(self, goal_joint_names: Sequence[str]) -> None:
        names = tuple(str(v) for v in goal_joint_names)
        if len(names) != len(set(names)):
            raise JointMappingError("goal joint names must be unique")
        unknown = [name for name in names if name not in self.contract.joint_names]
        if unknown:
            raise JointMappingError(f"goal contains unknown joints: {unknown}")
        if not self.contract.allow_partial_joints_goal and names != self.contract.joint_names:
            raise JointMappingError("controller disallows partial or reordered joint goals")

    def as_dict(self) -> Dict[str, object]:
        return {
            "controller_contract": self.contract.as_dict(),
            "fixed_posture": self.fixed_posture.as_dict(),
            "active_order": ["q2", "q3", "q5"],
        }


def contract_from_mapping(data: Mapping[str, object]) -> ControllerContract:
    return ControllerContract(
        joint_names=tuple(data.get("joint_names", DEFAULT_CONTROLLER_JOINTS)),
        command_interfaces=tuple(data.get("command_interfaces", ("position",))),
        state_interfaces=tuple(data.get("state_interfaces", ("position", "velocity"))),
        allow_partial_joints_goal=bool(data.get("allow_partial_joints_goal", False)),
        state_publish_rate_hz=float(data.get("state_publish_rate_hz", 100.0)),
        update_rate_hz=float(data.get("update_rate_hz", 100.0)),
        action_name=str(data.get("action_name", "/arm_controller/follow_joint_trajectory")),
    )

