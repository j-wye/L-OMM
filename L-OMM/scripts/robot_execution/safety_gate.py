from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, Tuple


class ExecutionMode:
    DRY_RUN = "dry_run"
    FAKE_ACTION = "fake_action"
    ARMED = "armed"


class SafetyViolation(RuntimeError):
    """Raised when code attempts to cross the no-command boundary."""


@dataclass(frozen=True)
class ExecutionSafetyConfig:
    mode: str = ExecutionMode.DRY_RUN
    armed: bool = False
    armed_token: str = ""
    required_armed_token: str = "I_UNDERSTAND_MOVING_ARM_RISK"
    allow_real_action_client: bool = False
    allow_fake_action: bool = True
    allowed_action_name: str = "/arm_controller/follow_joint_trajectory"
    forbidden_endpoint_fragments: Tuple[str, ...] = (
        "/cmd_vel",
        "cmd_vel",
        "gripper",
        "servo",
        "base_controller",
        "tracer",
    )
    max_velocity_rad_s: float = 0.5
    max_acceleration_rad_s2: float = 0.4
    min_duration_s: float = 0.0
    max_duration_s: float = 120.0
    command_timeout_s: float = 30.0


@dataclass
class ExecutionSafetyGate:
    """Single authority for command-endpoint access.

    The gate is deliberately checked before real ROS2 action-client creation
    and again before goal sending.  Dry-run mode never permits either action.
    """

    config: ExecutionSafetyConfig = field(default_factory=ExecutionSafetyConfig)

    def assert_can_create_action_client(self, action_name: str, *, fake: bool = False) -> None:
        self._assert_endpoint_allowed(action_name)
        if fake:
            self._assert_fake_allowed()
            return
        self._assert_real_allowed(action_name, operation="create real action client")

    def assert_can_send_goal(self, action_name: str, *, fake: bool = False) -> None:
        self._assert_endpoint_allowed(action_name)
        if fake:
            self._assert_fake_allowed()
            return
        self._assert_real_allowed(action_name, operation="send real trajectory goal")

    def assert_trajectory_safe(self, command: Any) -> None:
        duration = float(getattr(command, "duration_s", 0.0))
        if duration < float(self.config.min_duration_s) - 1.0e-12:
            raise SafetyViolation(f"trajectory duration {duration:.6f}s below configured minimum")
        if duration > float(self.config.max_duration_s) + 1.0e-12:
            raise SafetyViolation(f"trajectory duration {duration:.6f}s exceeds configured maximum")

        summary = getattr(command, "summary", {}) or {}
        max_velocity = float(summary.get("max_velocity_rad_s", 0.0))
        max_acceleration = float(summary.get("max_acceleration_rad_s2", 0.0))
        if max_velocity > float(self.config.max_velocity_rad_s) + 1.0e-12:
            raise SafetyViolation("trajectory exceeds safety velocity cap")
        if max_acceleration > float(self.config.max_acceleration_rad_s2) + 1.0e-12:
            raise SafetyViolation("trajectory exceeds safety acceleration cap")

    def as_dict(self) -> Dict[str, object]:
        return {
            "mode": str(self.config.mode),
            "armed": bool(self.config.armed),
            "allow_real_action_client": bool(self.config.allow_real_action_client),
            "allow_fake_action": bool(self.config.allow_fake_action),
            "allowed_action_name": str(self.config.allowed_action_name),
            "max_velocity_rad_s": float(self.config.max_velocity_rad_s),
            "max_acceleration_rad_s2": float(self.config.max_acceleration_rad_s2),
            "min_duration_s": float(self.config.min_duration_s),
            "max_duration_s": float(self.config.max_duration_s),
            "command_timeout_s": float(self.config.command_timeout_s),
        }

    def _assert_endpoint_allowed(self, action_name: str) -> None:
        text = str(action_name)
        forbidden = _matches_any(text, self.config.forbidden_endpoint_fragments)
        if forbidden:
            raise SafetyViolation(f"forbidden command endpoint requested: {text}")

    def _assert_fake_allowed(self) -> None:
        if str(self.config.mode) != ExecutionMode.FAKE_ACTION:
            raise SafetyViolation("fake action endpoint allowed only in fake_action mode")
        if not bool(self.config.allow_fake_action):
            raise SafetyViolation("fake action endpoint disabled by safety config")

    def _assert_real_allowed(self, action_name: str, *, operation: str) -> None:
        if str(action_name) != str(self.config.allowed_action_name):
            raise SafetyViolation(f"{operation} blocked: unexpected action endpoint {action_name}")
        if str(self.config.mode) != ExecutionMode.ARMED:
            raise SafetyViolation(f"{operation} blocked: mode is {self.config.mode}")
        if not bool(self.config.armed):
            raise SafetyViolation(f"{operation} blocked: armed flag is false")
        if str(self.config.armed_token) != str(self.config.required_armed_token):
            raise SafetyViolation(f"{operation} blocked: armed token mismatch")
        if not bool(self.config.allow_real_action_client):
            raise SafetyViolation(f"{operation} blocked: real action client disabled")


def _matches_any(text: str, fragments: Iterable[str]) -> bool:
    lower = text.lower()
    return any(str(fragment).lower() in lower for fragment in fragments)
