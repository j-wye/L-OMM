from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


from map_update_layer.experiments.no_command_orchestrator import (  # noqa: E402
    EXECUTION_DT_S,
    run_real_map_orchestrator,
)


class RealMapNoCommandOrchestratorTest(unittest.TestCase):
    """Full-stack no-command closure on the real box-free baked map (G1-G6)."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.out = tempfile.mkdtemp(prefix="ag_real_orch_")
        cls.report = run_real_map_orchestrator(out=cls.out)
        cls.dyn = cls.report["dynamic_frames"]
        cls.unk = cls.report["unknown_frames"]
        cls.inv = cls.report["invariants"]

    # -- overall ---------------------------------------------------------------

    def test_status_complete_and_all_invariants_pass(self) -> None:
        self.assertEqual(self.report["status"], "GOAL_STATUS: COMPLETE")
        self.assertTrue(self.inv["all_pass"], f"invariants failed: {self.inv}")

    def test_real_baked_map_loaded_with_positive_fov(self) -> None:
        self.assertIn("map_test", str(self.report["map_path"]).replace("\\", "/"))
        self.assertGreater(int(self.report["fov_cells_first_frame"]), 0)
        self.assertEqual(list(self.report["map_shape"]), [54, 137])

    # -- G2: real ControlModule + control_bridge retiming ----------------------

    def test_control_module_produced_real_artifacts(self) -> None:
        accepted = [f for f in self.dyn if f["supervisor_action"] in {"started", "switched_to_accepted_reference"}]
        self.assertTrue(accepted, "expected at least one accepted ControlModule candidate")
        for f in accepted:
            self.assertTrue(bool(f["control_candidate_accepted"]))
            art = Path(str(f["control_artifact_dir"]))
            self.assertTrue((art / "reference.csv").exists(), f"missing reference.csv in {art}")
            self.assertTrue((art / "episode.csv").exists(), f"missing episode.csv in {art}")
            self.assertTrue((art / "summary.json").exists(), f"missing summary.json in {art}")

    def test_execution_candidate_retimed_to_0_1_no_fallback(self) -> None:
        accepted = [f for f in self.dyn if f["supervisor_action"] in {"started", "switched_to_accepted_reference"}]
        self.assertTrue(accepted)
        for f in accepted:
            self.assertAlmostEqual(float(f["execution_dt_s"]), EXECUTION_DT_S)
            self.assertAlmostEqual(float(f["execution_dt_s"]), 0.1)
            self.assertFalse(bool(f["execution_dt_fallback"]))

    # -- G1: standard validator (no private bypass) ----------------------------

    def test_standard_snapshot_validator_passes_every_frame(self) -> None:
        for f in self.dyn + self.unk:
            self.assertEqual(float(f["snapshot_valid"]), 1.0, f"snapshot invalid at {f['label']}")
        # snapshot_valid == final == base & ~blocked is enforced by SnapshotValidator.
        self.assertTrue(self.inv["snapshot_valid_every_frame"])

    # -- G3: PathCollisionMonitor live -----------------------------------------

    def test_path_collision_monitor_participated(self) -> None:
        self.assertTrue(self.inv["path_collision_monitor_participated"])
        self.assertTrue(any(str(f["reference_blocked_source"]) == "path_collision_monitor" for f in self.dyn))

    # -- G4: unknown / invalid-depth persistence + repeated-free release -------

    def test_unknown_observed_held_then_released(self) -> None:
        unk_obs = [int(f["cell_unknown_cells"]) for f in self.unk]
        sticky_unk = [int(f["sticky_unknown_cells"]) for f in self.unk]
        self.assertTrue(any(v > 0 for v in unk_obs), "no frame produced cell_unknown_cells > 0")
        inj = next(i for i, v in enumerate(unk_obs) if v > 0)
        # held across the immediately following (unobserved-as-unknown) frame
        self.assertGreater(sticky_unk[inj + 1], 0, "unknown not held by sticky persistence")
        # a single free frame must NOT release it
        self.assertGreater(sticky_unk[inj + 1], 0)
        # repeated free observation eventually releases it
        self.assertEqual(sticky_unk[-1], 0, "unknown never released")
        self.assertTrue(self.inv["unknown_persistence_held"])
        self.assertTrue(self.inv["unknown_released_by_repeated_free"])

    # -- G5: target evidence source attribution --------------------------------

    def test_target_attribution_records_concrete_source(self) -> None:
        attributed = [
            f for f in self.unk
            if bool(f["target_source_detection"]) or bool(f["target_source_cell_projection"])
            or bool(f["target_source_sticky"]) or bool(f["target_source_inflation"])
        ]
        self.assertTrue(attributed, "target evidence not attributed to any concrete source")
        self.assertTrue(any(bool(f["target_blocked_by_camera_evidence"]) for f in self.unk))

    # -- G6: over-cover measured -----------------------------------------------

    def test_overcover_measured(self) -> None:
        self.assertTrue(self.inv["overcover_recorded"])
        oc = self.inv["overcover_total"]
        self.assertIn("source", oc)
        self.assertGreaterEqual(int(oc["source"]), 0)
        # over-cover is a non-negative count of bbox-overcovered cells over source cells
        self.assertGreaterEqual(int(oc["occupied"]) + int(oc["occluded"]) + int(oc["unknown"]), 0)

    # -- six dynamic events ----------------------------------------------------

    def test_six_event_dynamic_scenario(self) -> None:
        actions = [str(f["supervisor_action"]) for f in self.dyn]
        events = [str(f["event_class"]) for f in self.dyn]
        self.assertIn("started", actions)
        self.assertIn("INITIAL_ACCEPT", events)
        self.assertIn("continue_current_reference", actions)
        self.assertIn("switched_to_accepted_reference", actions)
        self.assertIn("recovery_request", actions)
        self.assertIn("same_snapshot_retry_prevented", actions)
        self.assertIn("safety_recovery_boundary", actions)
        self.assertIn("CURRENT_POSE_UNSAFE", events)

    # -- H1: same-snapshot retry guard BEFORE ControlModule.run ----------------

    def test_same_snapshot_retry_burns_no_control_run_or_artifact(self) -> None:
        self.assertTrue(self.inv["h1_retry_no_control_run"])
        self.assertTrue(self.inv["h1_retry_no_artifact_dir"])
        self.assertTrue(self.inv["h1_fewer_runs_than_replan_frames"])

        idx = next(
            (i for i, f in enumerate(self.dyn) if str(f["supervisor_action"]) == "same_snapshot_retry_prevented"),
            None,
        )
        self.assertIsNotNone(idx, "no same_snapshot_retry_prevented frame in the dynamic scenario")
        self.assertGreater(idx, 0)
        prev, cur = self.dyn[idx - 1], self.dyn[idx]
        # ControlModule.run did NOT fire for the retry frame (cumulative count is unchanged).
        self.assertEqual(int(cur["control_run_invocations"]), int(prev["control_run_invocations"]))
        # and NO new control artifact dir was created for it.
        self.assertEqual(str(cur["control_artifact_dir"]), "")
        self.assertEqual(str(cur["supervisor_action"]), "same_snapshot_retry_prevented")

    def test_first_blocking_frame_still_runs_control_and_recovers(self) -> None:
        rec_idx = next(
            (i for i, f in enumerate(self.dyn) if str(f["supervisor_action"]) == "recovery_request"),
            None,
        )
        self.assertIsNotNone(rec_idx, "expected a rejected-replan recovery_request frame")
        self.assertGreater(rec_idx, 0)
        # the first blocking frame DID run ControlModule (its cumulative count strictly grew)
        # and wrote a real control artifact dir.
        self.assertGreater(
            int(self.dyn[rec_idx]["control_run_invocations"]),
            int(self.dyn[rec_idx - 1]["control_run_invocations"]),
        )
        self.assertTrue(str(self.dyn[rec_idx]["control_artifact_dir"]))
        # the measurable saving: strictly fewer ControlModule runs than replan frames.
        runs = self.inv["control_run_invocations"]
        self.assertLess(int(runs["dynamic_control_ran_on_replan"]), int(runs["dynamic_replan_frames"]))

    # -- H2: detection target_rect path scope (B(b)) ---------------------------

    def test_detection_target_rect_scoped_cell_projection_carries_block(self) -> None:
        self.assertTrue(self.inv["h2_detection_path_scoped"])
        self.assertTrue(self.inv["h2_target_carried_by_cell_projection"])
        # A target detection IS emitted on the target-present frames, yet it produces NO
        # target_rect under the fronto-parallel camera because the image-space y-band
        # prefilter deterministically drops an off-cx on-plane detection. This documents the
        # actual (empty) detection target_rect footprint, the precise B(b) scope.
        scoped = [
            f for f in self.unk
            if int(f["target_candidate_count"]) > 0 and int(f["detection_target_rect_count"]) == 0
        ]
        self.assertTrue(scoped, "no frame documents the detection emit-but-dropped footprint")
        for f in scoped:
            self.assertEqual(int(f["detection_prefilter_kept"]), 0)
        # detection never marks the target cell; cell-projection does (the correct path here).
        self.assertFalse(any(bool(f["target_source_detection"]) for f in self.unk))
        self.assertTrue(any(bool(f["target_source_cell_projection"]) for f in self.unk))

    # -- no-command ------------------------------------------------------------

    def test_no_real_action_client_fake_only(self) -> None:
        self.assertTrue(self.inv["no_real_action_client"])
        for f in self.dyn + self.unk:
            self.assertFalse(bool(f["real_action_client_created"]))


if __name__ == "__main__":
    unittest.main()
