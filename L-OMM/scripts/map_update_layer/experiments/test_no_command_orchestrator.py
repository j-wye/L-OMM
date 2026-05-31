from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


from map_update_layer.experiments.no_command_orchestrator import (  # noqa: E402
    EXECUTION_DT_S,
    NoCommandOrchestrator,
    build_reference_along_z,
    build_synthetic_handle,
    execution_candidate_from_reference,
    run_orchestrator,
)


def _tmp_out() -> str:
    return tempfile.mkdtemp(prefix="ag_no_command_orch_")


class NoCommandOrchestratorSeamTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = run_orchestrator(out=_tmp_out())

    # -- full seam + per-frame invariants -------------------------------------

    def test_full_seam_runs_and_snapshot_valid_every_frame(self) -> None:
        self.assertGreaterEqual(len(self.result.frames), 3)
        self.assertTrue(self.result.invariants["snapshot_valid_every_frame"])

    def test_sticky_is_applied_not_skipped(self) -> None:  # TRAP 1
        self.assertTrue(self.result.invariants["sticky_applied_every_frame"])
        for frame in self.result.frames:
            self.assertEqual(float(frame["sticky_runtime_applied"]), 1.0)

    def test_reference_capture_increments_only_on_accept_or_successful_replan(self) -> None:  # TRAP 2
        frames = self.result.frames
        # frame 0 is the initial accept -> exactly one capture.
        self.assertEqual(int(frames[0]["reference_capture_count"]), 1)
        # the non-critical frame must not capture again.
        self.assertEqual(int(frames[1]["reference_capture_count"]), 1)
        # the accepted replan captures a second reference.
        accept_frames = [f for f in frames if f["supervisor_action"] == "switched_to_accepted_reference"]
        self.assertTrue(accept_frames, "expected at least one accepted replan")
        self.assertEqual(int(accept_frames[0]["reference_capture_count"]), 2)
        # rejected replan / retry-prevented / current-pose-unsafe frames must NOT capture.
        for f in frames:
            if f["supervisor_action"] in {
                "recovery_request",
                "same_snapshot_retry_prevented",
                "safety_recovery_boundary",
            }:
                self.assertEqual(int(f["reference_capture_count"]), 2)
        # capture count is monotone non-decreasing and never exceeds the accepted total.
        counts = [int(f["reference_capture_count"]) for f in frames]
        self.assertEqual(counts, sorted(counts))
        self.assertEqual(max(counts), 2)

    def test_sticky_fov_source_is_cell_projection_fov(self) -> None:  # TRAP 3
        handle = build_synthetic_handle()
        orch = NoCommandOrchestrator(handle, use_path_collision_monitor=False)
        from map_update_layer.experiments.no_command_orchestrator import FrameSpec

        raw = orch._build_snapshot(FrameSpec(label="probe"), 0)
        self.assertIn("cell_projection_fov", raw.layer_masks)
        # _apply_sticky must fail fast if the cell-projection FOV layer is absent.
        stripped = dict(raw.layer_masks)
        stripped.pop("cell_projection_fov", None)
        object.__setattr__(raw, "layer_masks", stripped)
        with self.assertRaises(ValueError):
            orch._apply_sticky(raw)

    def test_execution_candidate_dt_is_retimed_to_0_1(self) -> None:  # TRAP 4
        handle = build_synthetic_handle()
        ref = build_reference_along_z(handle, ix=handle.shape[0] // 2, iz_lo=2, iz_hi=8)
        candidate = execution_candidate_from_reference(ref)
        self.assertEqual(float(candidate.dt_s), EXECUTION_DT_S)
        self.assertAlmostEqual(float(candidate.dt_s), 0.1)

    def test_require_explicit_transform_and_q_grid_enforced(self) -> None:  # TRAP 5
        handle = build_synthetic_handle()
        orch = NoCommandOrchestrator(handle, use_path_collision_monitor=False)
        self.assertTrue(bool(orch.bridge.require_explicit_transform))
        # q_grid is mandatory.
        from control_module.map_handle import MapHandle

        no_q = MapHandle(
            map_path=handle.map_path,
            meta_path=handle.meta_path,
            mu_grid=handle.mu_grid,
            pitch_grid=handle.pitch_grid,
            q_grid=None,
            meta=handle.meta,
            resolution_m=handle.resolution_m,
            x0=handle.x0,
            z0=handle.z0,
            target_y=handle.target_y,
            tag=handle.tag,
        )
        with self.assertRaises(ValueError):
            NoCommandOrchestrator(no_q, use_path_collision_monitor=False)

    def test_target_blocked_by_camera_evidence_on_box_free_map(self) -> None:  # SAFETY TRANSFER
        self.assertTrue(self.result.invariants["target_blocked_by_camera_evidence"])
        self.assertTrue(any(bool(f["target_blocked_by_camera_evidence"]) for f in self.result.frames))

    def test_no_real_action_client_created(self) -> None:
        self.assertTrue(self.result.invariants["no_real_action_client"])
        self.assertFalse(bool(self.result.summary["real_action_client_created"]))
        self.assertGreaterEqual(int(self.result.summary["fake_goal_count"]), 1)

    def test_cell_projection_dense_nonzero_under_prefilter(self) -> None:
        self.assertTrue(self.result.invariants["cell_projection_dense_nonzero"])
        dense = sum(
            int(f["cell_occupied_cells"]) + int(f["cell_occluded_cells"]) + int(f["cell_free_cells"])
            for f in self.result.frames
        )
        self.assertGreater(dense, 0)

    def test_dynamic_scenario_covers_all_event_outcomes(self) -> None:  # acceptance #9
        events = [str(f["event_class"]) for f in self.result.frames]
        actions = [str(f["supervisor_action"]) for f in self.result.frames]
        # 1. first accept
        self.assertIn("INITIAL_ACCEPT", events)
        self.assertIn("started", actions)
        # 2. non-critical change with no replan
        self.assertIn("continue_current_reference", actions)
        # 3. a replan-triggering blockage with an accepted replan (reference switch)
        self.assertTrue(any(e in {"REFERENCE_BLOCKED", "GOAL_CORRIDOR_BLOCKED"} for e in events))
        self.assertIn("switched_to_accepted_reference", actions)
        # 4. a rejected replan -> recovery handoff (UNGRASPABLE path)
        self.assertIn("recovery_request", actions)
        self.assertTrue(any(bool(f["recovery_request_present"]) for f in self.result.frames))
        # 5. same-snapshot retry prevention (no infinite replan)
        self.assertIn("same_snapshot_retry_prevented", actions)
        # 6. current pose unsafe -> safety recovery boundary
        self.assertIn("CURRENT_POSE_UNSAFE", events)
        self.assertIn("safety_recovery_boundary", actions)


if __name__ == "__main__":
    unittest.main()
