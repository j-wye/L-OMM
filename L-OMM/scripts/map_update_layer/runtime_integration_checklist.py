#!/usr/bin/env python3
"""Jetson runtime integration checklist for the pre-live-camera stage."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class ChecklistItem:
    name: str
    required_before_live: bool
    implemented_now: bool
    notes: str

    def as_dict(self) -> Dict[str, object]:
        return {
            "name": self.name,
            "required_before_live": bool(self.required_before_live),
            "implemented_now": bool(self.implemented_now),
            "notes": self.notes,
        }


class JetsonRuntimeIntegrationChecklist:
    """Structured checklist separating camera-rate, planning-rate, and servo-rate work."""

    def items(self) -> List[ChecklistItem]:
        return [
            ChecklistItem("camera_rate_snapshot_update_contract", True, True,
                          "MapUpdateRequest/ActiveMapSnapshot contract is available without live camera."),
            ChecklistItem("event_triggered_replanning_boundary", True, True,
                          "ReferenceBlockageClassifier and dry-run harness separate snapshot update from replanning."),
            ChecklistItem("dls_servo_rate_target", True, True,
                          "DLS code is written for 100 Hz class dt=0.01 simulation; hardware command is excluded."),
            ChecklistItem("map_update_rate_target", True, True,
                          "MapUpdateRequest carries map_update_hz; synthetic defaults are 10 Hz."),
            ChecklistItem("perception_to_map_adapter_core", True, True,
                          "PerceptionToMapAdapter converts synthetic/stored RGB-D detections to MapUpdateRequest."),
            ChecklistItem("reference_snapshot_diff", True, True,
                          "ReferenceSnapshotDiff compares accepted snapshots against live x-z depth evidence."),
            ChecklistItem("mobile_base_recovery_contract", True, True,
                          "Recovery request/response contracts exist; no base controller is implemented."),
            ChecklistItem("live_camera_connection", False, False,
                          "Explicitly excluded from the current stage."),
            ChecklistItem("ros2_topic_subscription", False, False,
                          "Explicitly excluded from the current stage."),
            ChecklistItem("real_robot_command_publish", False, False,
                          "Explicitly excluded from the current stage."),
            ChecklistItem("full_bt_supervisor", False, False,
                          "Only lightweight dry-run decision logic is implemented."),
        ]

    def as_dict(self) -> Dict[str, object]:
        items = self.items()
        return {
            "checklist_type": "pre_live_camera_jetson_runtime_integration",
            "servo_rate_hz_target_min": 100.0,
            "map_update_rate_hz_target_range": [10.0, 30.0],
            "planning_policy": "event_triggered_not_every_camera_frame",
            "live_camera_included": False,
            "ros2_topic_included": False,
            "real_robot_command_included": False,
            "items": [item.as_dict() for item in items],
        }
