#!/usr/bin/env python3
"""Lightweight dry-run supervisor decision logic.

This is deliberately not a full Behavior Tree.  It is a small deterministic
boundary checker that turns a classifier event plus an optional Control Module
candidate result into a single next-action record.
"""
from __future__ import annotations

from typing import Dict, Optional

from .runtime_contract import NextAction, PlanOutcome, RuntimeDecision, RuntimeEventClass


class LightweightSupervisorDryRun:
    """Convert event and candidate metrics into a minimal runtime decision."""

    def decide(
        self,
        *,
        event_class: str,
        replanning_triggered: bool,
        reason: str = "",
        candidate_metrics: Optional[Dict[str, object]] = None,
    ) -> RuntimeDecision:
        event = str(event_class)
        if event == RuntimeEventClass.CURRENT_POSE_UNSAFE:
            return RuntimeDecision(
                event_class=event,
                plan_outcome=PlanOutcome.CURRENT_POSE_UNSAFE,
                replanning_triggered=False,
                control_called=False,
                candidate_accepted=False,
                next_action=NextAction.SAFETY_RECOVERY_BOUNDARY,
                recovery_required=True,
                reject_reason=PlanOutcome.CURRENT_POSE_UNSAFE,
                invalid_reason_code="current_pose_intersects_blocked_mask",
                reason=reason,
            )

        if not bool(replanning_triggered):
            return RuntimeDecision(
                event_class=event,
                plan_outcome=PlanOutcome.CONTINUE_CURRENT_REFERENCE,
                replanning_triggered=False,
                control_called=False,
                candidate_accepted=False,
                next_action=NextAction.CONTINUE_CURRENT_REFERENCE,
                recovery_required=False,
                reason=reason,
            )

        if candidate_metrics is None:
            return RuntimeDecision(
                event_class=event,
                plan_outcome="REPLAN_REQUIRED",
                replanning_triggered=True,
                control_called=True,
                candidate_accepted=False,
                next_action="call_control_module",
                recovery_required=False,
                reason=reason,
            )

        accepted = bool(candidate_metrics.get("candidate_accepted", False))
        reject_reason = str(candidate_metrics.get("rejection_reason", ""))
        invalid_reason_code = str(candidate_metrics.get("invalid_reason_code", ""))
        if accepted:
            return RuntimeDecision(
                event_class=event,
                plan_outcome=PlanOutcome.CANDIDATE_ACCEPTED,
                replanning_triggered=True,
                control_called=True,
                candidate_accepted=True,
                next_action=NextAction.SWITCH_TO_ACCEPTED_REFERENCE,
                recovery_required=False,
                reject_reason="",
                invalid_reason_code=invalid_reason_code,
                reason=reason,
            )
        return RuntimeDecision(
            event_class=event,
            plan_outcome=PlanOutcome.UNGRASPABLE,
            replanning_triggered=True,
            control_called=True,
            candidate_accepted=False,
            next_action=NextAction.REQUEST_MOBILE_BASE_REPOSITION,
            recovery_required=True,
            reject_reason=reject_reason or PlanOutcome.UNGRASPABLE,
            invalid_reason_code=invalid_reason_code,
            reason=reason,
        )
