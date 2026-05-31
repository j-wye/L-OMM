#!/usr/bin/env python3
"""Synthetic moving-obstacle sequence factory.

This module deliberately avoids live camera, ROS2 subscriptions, and real
perception adapters.  It provides deterministic geometry streams for testing
snapshot updates, blockage classification, and event-triggered replanning.
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Tuple

try:
    from control_module.constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE_INFLATION_M, Rect
except ImportError:
    from constants import DEFAULT_MAP_UPDATE_HZ, DEFAULT_OBSTACLE_INFLATION_M, Rect

from .dynamic_frame import SyntheticDynamicFrame, SyntheticDynamicScenario
from .map_update_request import MapUpdateRequest


class SyntheticDynamicSequences:
    """Build deterministic camera-free dynamic scenarios."""

    def __init__(
        self,
        *,
        inflation_m: float = DEFAULT_OBSTACLE_INFLATION_M,
        map_update_hz: float = DEFAULT_MAP_UPDATE_HZ,
    ) -> None:
        self.inflation_m = float(inflation_m)
        self.map_update_hz = float(map_update_hz)

    def all_scenarios(
        self,
        *,
        include_reference_diff_scenarios: bool = True,
        include_p2_validation_scenarios: bool = False,
    ) -> Dict[str, SyntheticDynamicScenario]:
        scenarios = {
            "clear_baseline": self.clear_baseline(),
            "noncritical_moving_box": self.noncritical_moving_box(),
            "reference_crossing_box": self.reference_crossing_box(),
            "goal_blocking_box": self.goal_blocking_box(),
            "inflation_sensitive_box": self.inflation_sensitive_box(),
            "current_pose_unsafe_box": self.current_pose_unsafe_box(),
            "base_reposition_recovery": self.base_reposition_recovery(),
        }
        if include_reference_diff_scenarios:
            scenarios.update(
                {
                    "reference_snapshot_new_obstacle_on_path": self.reference_snapshot_new_obstacle_on_path(),
                    "reference_snapshot_change_outside_fov": self.reference_snapshot_change_outside_fov(),
                    "reference_snapshot_vanished_obstacle_nontrigger": self.reference_snapshot_vanished_obstacle_nontrigger(),
                    "reference_snapshot_single_cell_jitter_ignored": self.reference_snapshot_single_cell_jitter_ignored(),
                }
            )
        if include_p2_validation_scenarios:
            scenarios.update(
                {
                    "capsule_link_collision_box": self.capsule_link_collision_box(),
                    "target_inflation_vs_grasp_window": self.target_inflation_vs_grasp_window(),
                }
            )
        return scenarios

    def clear_baseline(self) -> SyntheticDynamicScenario:
        frames = [
            self._frame(0, "clear_0", "synthetic_dynamic_clear_0", expected="NO_RELEVANT_CHANGE", s_frac=0.00,
                        ground_truth_replan_required=False, ground_truth_event_class="NO_RELEVANT_CHANGE"),
            self._frame(1, "clear_1", "synthetic_dynamic_clear_1", expected="NO_RELEVANT_CHANGE", s_frac=0.25,
                        ground_truth_replan_required=False, ground_truth_event_class="NO_RELEVANT_CHANGE"),
            self._frame(2, "clear_2", "synthetic_dynamic_clear_2", expected="NO_RELEVANT_CHANGE", s_frac=0.50,
                        ground_truth_replan_required=False, ground_truth_event_class="NO_RELEVANT_CHANGE"),
        ]
        return SyntheticDynamicScenario(
            name="clear_baseline",
            description="No semantic layer changes; no replanning should be triggered.",
            frames=tuple(frames),
        )

    def noncritical_moving_box(self) -> SyntheticDynamicScenario:
        rects = [
            (0.02, 0.34, 0.10, 0.26),
            (0.04, 0.35, 0.12, 0.27),
            (0.06, 0.36, 0.14, 0.28),
            (0.08, 0.36, 0.16, 0.28),
        ]
        frames = [
            self._frame(i, f"noncritical_{i}", f"synthetic_dynamic_noncritical_{i}",
                        occupied=(rect,), expected="MASK_CHANGED_NONCRITICAL", s_frac=0.20 + 0.08 * i,
                        ground_truth_replan_required=False, ground_truth_event_class="MASK_CHANGED_NONCRITICAL")
            for i, rect in enumerate(rects)
        ]
        return SyntheticDynamicScenario(
            name="noncritical_moving_box",
            description="A slow box moves away from the current reference corridor.",
            frames=tuple(frames),
        )

    def reference_crossing_box(self) -> SyntheticDynamicScenario:
        rects = [
            (0.10, 0.78, 0.18, 0.70),
            (0.15, 0.73, 0.23, 0.65),
            (0.20, 0.68, 0.28, 0.60),
            (0.25, 0.63, 0.33, 0.55),
        ]
        frames = [
            self._frame(i, f"reference_cross_{i}", f"synthetic_dynamic_reference_cross_{i}",
                        occupied=(rect,),
                        expected="REFERENCE_BLOCKED|CURRENT_POSE_UNSAFE",
                        s_frac=0.18 + 0.08 * i,
                        ground_truth_replan_required=True,
                        ground_truth_event_class="REFERENCE_BLOCKED|CURRENT_POSE_UNSAFE")
            for i, rect in enumerate(rects)
        ]
        return SyntheticDynamicScenario(
            name="reference_crossing_box",
            description="A slow box crosses the current lookahead reference corridor.",
            frames=tuple(frames),
        )

    def goal_blocking_box(self) -> SyntheticDynamicScenario:
        # This box is intentionally close to the goal-side arm-volume corridor.
        rects = [
            (0.24, 0.52, 0.32, 0.40),
            (0.28, 0.54, 0.36, 0.42),
            (0.32, 0.56, 0.40, 0.44),
        ]
        frames = [
            self._frame(i, f"goal_block_{i}", f"synthetic_dynamic_goal_block_{i}",
                        occupied=(rect,),
                        expected="GOAL_CORRIDOR_BLOCKED|MASK_CHANGED_NONCRITICAL" if i == 0 else "GOAL_CORRIDOR_BLOCKED",
                        s_frac=0.55 + 0.10 * i,
                        ground_truth_replan_required=True if i > 0 else None,
                        ground_truth_event_class="GOAL_CORRIDOR_BLOCKED" if i > 0 else "GOAL_CORRIDOR_BLOCKED|MASK_CHANGED_NONCRITICAL",
                        ground_truth_use_in_metrics=i > 0)
            for i, rect in enumerate(rects)
        ]
        return SyntheticDynamicScenario(
            name="goal_blocking_box",
            description="The box blocks the goal-side corridor and may lead to UNGRASPABLE.",
            frames=tuple(frames),
        )

    def inflation_sensitive_box(self) -> SyntheticDynamicScenario:
        rect = (0.17, 0.64, 0.24, 0.57)
        frames = [
            self._frame(0, "inflation_small", "synthetic_dynamic_inflation_small",
                        occupied=(rect,), inflation_m=0.0,
                        expected="MASK_CHANGED_NONCRITICAL|REFERENCE_BLOCKED", s_frac=0.28,
                        ground_truth_replan_required=None,
                        ground_truth_event_class="MASK_CHANGED_NONCRITICAL|REFERENCE_BLOCKED",
                        ground_truth_use_in_metrics=False),
            self._frame(1, "inflation_default", "synthetic_dynamic_inflation_default",
                        occupied=(rect,), inflation_m=self.inflation_m,
                        expected="REFERENCE_BLOCKED", s_frac=0.30,
                        ground_truth_replan_required=True, ground_truth_event_class="REFERENCE_BLOCKED"),
            self._frame(2, "inflation_large", "synthetic_dynamic_inflation_large",
                        occupied=(rect,), inflation_m=0.03,
                        expected="REFERENCE_BLOCKED", s_frac=0.32,
                        ground_truth_replan_required=True, ground_truth_event_class="REFERENCE_BLOCKED"),
        ]
        return SyntheticDynamicScenario(
            name="inflation_sensitive_box",
            description="The same semantic box changes classification as inflation increases.",
            frames=tuple(frames),
        )

    def current_pose_unsafe_box(self) -> SyntheticDynamicScenario:
        rects = [
            (0.11, 0.73, 0.19, 0.65),
            (0.15, 0.70, 0.23, 0.62),
        ]
        frames = [
            self._frame(0, "current_pose_near", "synthetic_dynamic_current_pose_near",
                        occupied=(rects[0],), expected="CURRENT_POSE_UNSAFE", s_frac=0.30,
                        ground_truth_replan_required=True, ground_truth_event_class="CURRENT_POSE_UNSAFE"),
            self._frame(1, "current_pose_unsafe", "synthetic_dynamic_current_pose_unsafe",
                        occupied=(rects[1],), expected="CURRENT_POSE_UNSAFE", s_frac=0.38,
                        ground_truth_replan_required=True, ground_truth_event_class="CURRENT_POSE_UNSAFE"),
        ]
        return SyntheticDynamicScenario(
            name="current_pose_unsafe_box",
            description="A synthetic obstacle is placed at the current reference pose.",
            frames=tuple(frames),
        )

    def base_reposition_recovery(self) -> SyntheticDynamicScenario:
        """UNGRASPABLE on one snapshot, then success only after a new snapshot.

        This does not simulate a mobile-base controller.  It only verifies the
        contract that retry is allowed after a new synthetic snapshot arrives.
        """
        frames = [
            self._frame(
                0,
                "goal_block_before_base_motion",
                "synthetic_dynamic_base_recovery_blocked",
                occupied=((0.32, 0.56, 0.40, 0.44),),
                expected="GOAL_CORRIDOR_BLOCKED",
                s_frac=0.65,
                notes="current snapshot is intentionally ungraspable",
                ground_truth_replan_required=True,
                ground_truth_event_class="GOAL_CORRIDOR_BLOCKED",
            ),
            self._frame(
                1,
                "new_snapshot_after_base_motion",
                "synthetic_dynamic_base_recovery_new_snapshot",
                expected="TARGET_CHANGED",
                s_frac=0.65,
                target_changed=True,
                notes="synthetic new observation after base reposition; no mobile-base command is generated",
                ground_truth_replan_required=True,
                ground_truth_event_class="TARGET_CHANGED",
            ),
        ]
        return SyntheticDynamicScenario(
            name="base_reposition_recovery",
            description="Same-snapshot retry is skipped; a new synthetic snapshot allows planning again.",
            frames=tuple(frames),
            metadata={
                "allow_continue_after_ungraspable": True,
                "requires_new_snapshot_for_retry": True,
                "implements_mobile_base_controller": False,
            },
        )

    def reference_snapshot_new_obstacle_on_path(self) -> SyntheticDynamicScenario:
        """A new in-FOV obstacle enters the accepted reference corridor."""
        frame = self._frame(
            0,
            "new_obstacle_on_path",
            "synthetic_reference_diff_new_obstacle_on_path",
            occupied=((0.17, 0.64, 0.24, 0.57),),
            expected="REFERENCE_BLOCKED",
            s_frac=0.30,
            metadata={
                "reference_diff_fov_override": "full",
                "reference_diff_expected": {
                    "replanning_triggered": True,
                    "is_corridor_violated": True,
                    "new_cells_in_reference_corridor_min": 3,
                    "largest_reference_corridor_cluster_size_min": 5,
                    "vanished_obstacle_cells_max": 0,
                },
                "ground_truth_replan_required": True,
                "ground_truth_event_class": "REFERENCE_BLOCKED",
                "ground_truth_use_in_metrics": True,
            },
            notes="new obstacle is intentionally placed on the accepted reference corridor",
        )
        return SyntheticDynamicScenario(
            name="reference_snapshot_new_obstacle_on_path",
            description="Reference diff must trigger only when a new obstacle enters the accepted path corridor.",
            frames=(frame,),
            metadata={
                "p2_4_reference_diff_scenario": True,
                "ground_truth_replan_required": True,
            },
        )

    def reference_snapshot_change_outside_fov(self) -> SyntheticDynamicScenario:
        """A path-like change is ignored when outside the current task FOV."""
        frame = self._frame(
            0,
            "change_outside_fov",
            "synthetic_reference_diff_change_outside_fov",
            occupied=((0.17, 0.64, 0.24, 0.57),),
            expected="NO_RELEVANT_CHANGE",
            s_frac=0.20,
            metadata={
                "reference_diff_fov_override": "rects",
                "reference_diff_fov_rects": ((0.02, 0.36, 0.10, 0.26),),
                "reference_diff_expected": {
                    "replanning_triggered": False,
                    "is_corridor_violated": False,
                    "new_obstacle_cells_max": 0,
                    "new_cells_in_reference_corridor_max": 0,
                    "fov_observed_cells_min": 1,
                },
                "ground_truth_replan_required": False,
                "ground_truth_event_class": "NO_RELEVANT_CHANGE",
                "ground_truth_use_in_metrics": True,
            },
            notes="blocked geometry is outside the synthetic task-FOV mask and must not be promoted to a trigger",
        )
        return SyntheticDynamicScenario(
            name="reference_snapshot_change_outside_fov",
            description="A changed blocked mask outside M_FOV_t is ignored by reference snapshot diff.",
            frames=(frame,),
            metadata={
                "p2_4_reference_diff_scenario": True,
                "ground_truth_replan_required": False,
            },
        )

    def reference_snapshot_vanished_obstacle_nontrigger(self) -> SyntheticDynamicScenario:
        """An obstacle present at accepted-plan time disappears later."""
        initial_request = self._request(
            occupied=((0.02, 0.34, 0.10, 0.26),),
            source="synthetic_reference_diff_vanished_reference",
            sequence_id=-1,
        )
        frame = self._frame(
            0,
            "vanished_obstacle",
            "synthetic_reference_diff_vanished_current",
            expected="MASK_CHANGED_NONCRITICAL",
            s_frac=0.25,
            metadata={
                "reference_diff_fov_override": "full",
                "reference_diff_expected": {
                    "replanning_triggered": False,
                    "is_corridor_violated": False,
                    "vanished_obstacle_cells_min": 1,
                    "new_obstacle_cells_max": 0,
                },
                "ground_truth_replan_required": False,
                "ground_truth_event_class": "MASK_CHANGED_NONCRITICAL",
                "ground_truth_use_in_metrics": True,
            },
            notes="vanished cells reduce risk and are recorded without becoming a replan trigger",
        )
        return SyntheticDynamicScenario(
            name="reference_snapshot_vanished_obstacle_nontrigger",
            description="A vanished obstacle is a non-triggering reference diff statistic.",
            frames=(frame,),
            metadata={
                "p2_4_reference_diff_scenario": True,
                "ground_truth_replan_required": False,
            },
            initial_request=initial_request,
        )

    def reference_snapshot_single_cell_jitter_ignored(self) -> SyntheticDynamicScenario:
        """A one-cell new obstacle is below the spatial cluster threshold."""
        frame = self._frame(
            0,
            "single_cell_jitter",
            "synthetic_reference_diff_single_cell_jitter",
            occupied=((0.20, 0.60, 0.20, 0.60),),
            inflation_m=0.0,
            expected="MASK_CHANGED_NONCRITICAL",
            s_frac=0.05,
            metadata={
                "reference_diff_fov_override": "full",
                "reference_diff_expected": {
                    "replanning_triggered": False,
                    "is_corridor_violated": False,
                    "new_obstacle_cells_max": 1,
                    "largest_reference_corridor_cluster_size_max": 1,
                },
                "ground_truth_replan_required": False,
                "ground_truth_event_class": "MASK_CHANGED_NONCRITICAL",
                "ground_truth_use_in_metrics": True,
            },
            notes="single-cell flicker is recorded as noncritical mask change, not a reference violation",
        )
        return SyntheticDynamicScenario(
            name="reference_snapshot_single_cell_jitter_ignored",
            description="A 1-cell spatial jitter is below tau_diff/tau_cluster and must not trigger replanning.",
            frames=(frame,),
            metadata={
                "p2_4_reference_diff_scenario": True,
                "ground_truth_replan_required": False,
            },
        )

    def target_inflation_vs_grasp_window(self) -> SyntheticDynamicScenario:
        """Target inflation near the EE-safe goal window.

        The frames are intentionally metric-oriented and are excluded from the
        default 7-scenario regression.  The separate P2.3 sweep script provides
        the quantitative boundary; this scenario gives P2.6 one sequence-level
        oracle row for the same contract.
        """
        frames = [
            self._frame(
                0,
                "target_clearance_safe",
                "synthetic_dynamic_target_inflation_safe",
                target=((0.47, 0.55, 0.55, 0.45),),
                inflation_m=0.005,
                expected="MASK_CHANGED_NONCRITICAL",
                s_frac=0.50,
                ground_truth_replan_required=False,
                ground_truth_event_class="MASK_CHANGED_NONCRITICAL",
                notes="target remains outside the EE-safe goal window after nominal inflation",
            ),
            self._frame(
                1,
                "target_clearance_boundary",
                "synthetic_dynamic_target_inflation_boundary",
                target=((0.43, 0.55, 0.55, 0.45),),
                inflation_m=0.020,
                expected="GOAL_CORRIDOR_BLOCKED|MASK_CHANGED_NONCRITICAL",
                s_frac=0.55,
                ground_truth_replan_required=None,
                ground_truth_event_class="GOAL_CORRIDOR_BLOCKED|MASK_CHANGED_NONCRITICAL",
                ground_truth_use_in_metrics=False,
                notes="boundary case is excluded from aggregate precision/recall",
            ),
            self._frame(
                2,
                "target_clearance_blocked",
                "synthetic_dynamic_target_inflation_blocked",
                target=((0.42, 0.55, 0.55, 0.45),),
                inflation_m=0.030,
                expected="GOAL_CORRIDOR_BLOCKED",
                s_frac=0.60,
                ground_truth_replan_required=True,
                ground_truth_event_class="GOAL_CORRIDOR_BLOCKED",
                notes="inflated target volume intrudes into the EE-safe goal-side corridor",
            ),
        ]
        return SyntheticDynamicScenario(
            name="target_inflation_vs_grasp_window",
            description="P2.3 target inflation near the EE-safe goal window.",
            frames=tuple(frames),
            metadata={
                "p2_3_target_inflation_scenario": True,
            },
        )

    def capsule_link_collision_box(self) -> SyntheticDynamicScenario:
        """EE reference is clear, but a supplied capsule-chain proxy hits the box."""
        frame = self._frame(
            0,
            "capsule_link_collision",
            "synthetic_dynamic_capsule_link_collision",
            occupied=((0.02, 0.34, 0.10, 0.26),),
            expected="CURRENT_POSE_UNSAFE",
            s_frac=0.55,
            metadata={
                "capsule_proxy_xz": ((0.06, 0.30), (0.07, 0.31), (0.08, 0.30)),
                "p1_4_capsule_proxy_scenario": True,
            },
            ground_truth_replan_required=True,
            ground_truth_event_class="CURRENT_POSE_UNSAFE",
            notes="blocked region intersects a synthetic capsule link proxy while the nominal EE reference remains elsewhere",
        )
        return SyntheticDynamicScenario(
            name="capsule_link_collision_box",
            description="P1.4/P2.6 capsule proxy current-pose unsafe sequence.",
            frames=(frame,),
            metadata={"p1_4_capsule_proxy_scenario": True},
        )

    def _frame(
        self,
        frame_index: int,
        label: str,
        source: str,
        *,
        occupied: Iterable[Rect] = (),
        target: Iterable[Rect] = (),
        unknown: Iterable[Rect] = (),
        occluded: Iterable[Rect] = (),
        inflation_m: float | None = None,
        expected: str = "",
        s_frac: float = 0.0,
        target_changed: bool = False,
        notes: str = "",
        metadata: Dict[str, object] | None = None,
        ground_truth_replan_required: bool | None = None,
        ground_truth_event_class: str | None = None,
        ground_truth_use_in_metrics: bool = True,
    ) -> SyntheticDynamicFrame:
        frame_metadata = dict(metadata or {})
        if ground_truth_replan_required is not None:
            frame_metadata["ground_truth_replan_required"] = bool(ground_truth_replan_required)
        if ground_truth_event_class is not None:
            frame_metadata["ground_truth_event_class"] = str(ground_truth_event_class)
        if ground_truth_replan_required is None and ground_truth_event_class is not None:
            frame_metadata["ground_truth_oracle_status"] = "ambiguous"
        frame_metadata["ground_truth_use_in_metrics"] = bool(ground_truth_use_in_metrics)
        request = self._request(
            occupied_rects=tuple(occupied),
            target_rects=tuple(target),
            unknown_rects=tuple(unknown),
            occluded_rects=tuple(occluded),
            inflation_m=float(self.inflation_m if inflation_m is None else inflation_m),
            source=str(source),
            sequence_id=int(frame_index),
        )
        return SyntheticDynamicFrame(
            frame_index=int(frame_index),
            label=str(label),
            request=request,
            current_s_fraction=float(s_frac),
            expected_event=str(expected),
            target_changed_event=bool(target_changed),
            notes=str(notes),
            metadata=frame_metadata,
        )

    def _request(
        self,
        *,
        occupied: Iterable[Rect] = (),
        target: Iterable[Rect] = (),
        unknown: Iterable[Rect] = (),
        occluded: Iterable[Rect] = (),
        occupied_rects: Iterable[Rect] | None = None,
        target_rects: Iterable[Rect] | None = None,
        unknown_rects: Iterable[Rect] | None = None,
        occluded_rects: Iterable[Rect] | None = None,
        inflation_m: float | None = None,
        source: str,
        sequence_id: int,
    ) -> MapUpdateRequest:
        occ = tuple(occupied if occupied_rects is None else occupied_rects)
        tgt = tuple(target if target_rects is None else target_rects)
        unk = tuple(unknown if unknown_rects is None else unknown_rects)
        occd = tuple(occluded if occluded_rects is None else occluded_rects)
        return MapUpdateRequest(
            occupied_rects=occ,
            target_rects=tgt,
            unknown_rects=unk,
            occluded_rects=occd,
            inflation_m=float(self.inflation_m if inflation_m is None else inflation_m),
            map_update_hz=float(self.map_update_hz),
            source=str(source),
            timestamp_s=float(sequence_id) / max(float(self.map_update_hz), 1.0e-12),
            sequence_id=int(sequence_id),
        )
