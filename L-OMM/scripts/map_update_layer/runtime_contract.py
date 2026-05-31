#!/usr/bin/env python3
"""Minimal runtime event and outcome contract for dry-run supervision.

This module intentionally stays camera-free and robot-command-free.  It only
names the event classes, plan outcomes, and high-level next actions shared by
the map-update dry-run harness and future supervisor layer.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict


class RuntimeEventClass:
    """Named snapshot events ordered by safety priority."""

    CURRENT_POSE_UNSAFE = "CURRENT_POSE_UNSAFE"
    GOAL_CORRIDOR_BLOCKED = "GOAL_CORRIDOR_BLOCKED"
    REFERENCE_BLOCKED = "REFERENCE_BLOCKED"
    TARGET_CHANGED = "TARGET_CHANGED"
    MASK_CHANGED_NONCRITICAL = "MASK_CHANGED_NONCRITICAL"
    NO_RELEVANT_CHANGE = "NO_RELEVANT_CHANGE"

    PRIORITY = (
        CURRENT_POSE_UNSAFE,
        TARGET_CHANGED,
        GOAL_CORRIDOR_BLOCKED,
        REFERENCE_BLOCKED,
        MASK_CHANGED_NONCRITICAL,
        NO_RELEVANT_CHANGE,
    )

    @classmethod
    def priority_rank(cls, event_class: str) -> int:
        """Return lower rank for higher-priority runtime events."""
        try:
            return cls.PRIORITY.index(str(event_class))
        except ValueError:
            return len(cls.PRIORITY)


class PlanOutcome:
    """External candidate outcome visible above the control module."""

    CANDIDATE_ACCEPTED = "CANDIDATE_ACCEPTED"
    UNGRASPABLE = "UNGRASPABLE"
    CURRENT_POSE_UNSAFE = "CURRENT_POSE_UNSAFE"
    CONTINUE_CURRENT_REFERENCE = "CONTINUE_CURRENT_REFERENCE"


class NextAction:
    """Action labels for the dry-run supervisor boundary."""

    CONTINUE_CURRENT_REFERENCE = "continue_current_reference"
    SWITCH_TO_ACCEPTED_REFERENCE = "switch_to_accepted_reference"
    REQUEST_MOBILE_BASE_REPOSITION = "request_mobile_base_reposition"
    SAFETY_RECOVERY_BOUNDARY = "safety_recovery_boundary"


@dataclass(frozen=True)
class RuntimeDecision:
    """One-step decision record produced by the lightweight dry-run layer."""

    event_class: str
    plan_outcome: str
    replanning_triggered: bool
    control_called: bool
    candidate_accepted: bool
    next_action: str
    recovery_required: bool
    reject_reason: str = ""
    invalid_reason_code: str = ""
    reason: str = ""

    def as_dict(self) -> Dict[str, object]:
        return {
            "event_class": self.event_class,
            "plan_outcome": self.plan_outcome,
            "replanning_triggered": bool(self.replanning_triggered),
            "control_called": bool(self.control_called),
            "candidate_accepted": bool(self.candidate_accepted),
            "next_action": self.next_action,
            "recovery_required": bool(self.recovery_required),
            "reject_reason": self.reject_reason,
            "invalid_reason_code": self.invalid_reason_code,
            "reason": self.reason,
        }
