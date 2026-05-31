from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys
import unittest
import uuid
from unittest import mock

import numpy as np


TEST_TMP_DIR = Path(__file__).resolve().parent / "_tmp_test_runs"


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
EXPERIMENTS_ROOT = SCRIPTS_ROOT / "map_update_layer" / "experiments"
if str(EXPERIMENTS_ROOT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


class LiveSnapshotPassiveDecisionTest(unittest.TestCase):
    def _tmpdir(self) -> Path:
        path = TEST_TMP_DIR / f"case_{uuid.uuid4().hex}"
        shutil.rmtree(path, ignore_errors=True)
        path.mkdir(parents=True, exist_ok=True)
        self.addCleanup(lambda: shutil.rmtree(path, ignore_errors=True))
        return path

    def test_saved_frame_run_invokes_classifier_without_live_loop_control_recall(self) -> None:
        from live_snapshot_passive_decision import run_passive_decision, save_synthetic_frame

        tmp_path = self._tmpdir()
        frame_path = tmp_path / "frame.npz"
        out = tmp_path / "out"
        save_synthetic_frame(frame_path)

        summary = run_passive_decision(out=str(out), frames=2, use_saved_frame=str(frame_path))

        self.assertTrue(summary["control_module_called_before_loop"])
        self.assertFalse(summary["control_module_called_in_live_loop"])
        self.assertEqual(summary["frame_processed_count"], 2)
        self.assertTrue(summary["classifier_invoked_per_frame"])
        self.assertTrue((out / "events.csv").exists())
        self.assertTrue((out / "accepted_reference.npz").exists())

    def test_saved_frame_rich_schema_and_fov_smoke(self) -> None:
        from live_goal_corridor_source_attribution import REQUIRED_FRAME_MASKS
        from live_snapshot_passive_decision import run_passive_decision, save_synthetic_frame

        required_metadata = {
            "robot_command_endpoint_touched",
            "control_module_called_in_live_loop",
            "map_geometry",
            "snapshot_stats",
            "result_stats",
            "classifier_report",
            "goal_diagnostics",
            "fov_stats",
        }

        tmp_path = self._tmpdir()
        frame_path = tmp_path / "frame.npz"
        out = tmp_path / "out"
        save_synthetic_frame(frame_path)

        run_passive_decision(out=str(out), frames=2, use_saved_frame=str(frame_path))

        with np.load(out / "frames" / "frame_0000.npz", allow_pickle=False) as frame_npz:
            missing_masks = [name for name in REQUIRED_FRAME_MASKS if name not in frame_npz.files]
        self.assertEqual(missing_masks, [])

        metadata_path = out / "frames" / "frame_0000.json"
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        missing_metadata = sorted(required_metadata.difference(metadata.keys()))
        self.assertEqual(missing_metadata, [])

        goal_diagnostics = metadata["goal_diagnostics"]
        self.assertIn("goal_corridor_cells", goal_diagnostics)
        self.assertIn("goal_status_neighborhood_cells", goal_diagnostics)
        self.assertIn("current_pose_cells", goal_diagnostics)

        fov_results = []
        fov_errors = []
        for frame_metadata_path in sorted((out / "frames").glob("frame_*.json")):
            frame_metadata = json.loads(frame_metadata_path.read_text(encoding="utf-8"))
            fov_stats = frame_metadata["fov_stats"]
            fov_results.append(bool(fov_stats.get("fov_mask_available", False)))
            if not fov_results[-1]:
                fov_errors.append(str(fov_stats.get("fov_mask_error", "")))
        self.assertTrue(any(fov_results), f"FOV unavailable for every synthetic frame: {fov_errors}")

    def test_saved_frame_exports_raw_depth_audit_schema(self) -> None:
        from live_snapshot_passive_decision import run_passive_decision, save_synthetic_frame

        tmp_path = self._tmpdir()
        frame_path = tmp_path / "frame.npz"
        out = tmp_path / "out"
        save_synthetic_frame(frame_path)

        run_passive_decision(out=str(out), frames=1, use_saved_frame=str(frame_path))

        with np.load(out / "frames" / "frame_0000.npz", allow_pickle=False) as frame_npz:
            self.assertIn("depth_m", frame_npz.files)
            self.assertIn("unknown_candidate_mask", frame_npz.files)
            self.assertEqual(frame_npz["depth_m"].shape, frame_npz["unknown_candidate_mask"].shape)
            self.assertEqual(frame_npz["unknown_candidate_mask"].dtype, np.dtype("bool"))

        metadata = json.loads((out / "frames" / "frame_0000.json").read_text(encoding="utf-8"))
        intrinsics = metadata["intrinsics"]
        cfg = metadata["depth_geometry_config"]
        self.assertGreater(float(intrinsics["fx"]), 0.0)
        self.assertGreater(int(intrinsics["width"]), 0)
        self.assertGreater(float(cfg["pixel_band_half_width_px"]), 0.0)

    def test_default_camera_namespace_avoids_double_slash_topic_join(self) -> None:
        from live_snapshot_passive_decision import parse_args

        with mock.patch.object(sys, "argv", ["prog", "--out", "unused"]):
            args = parse_args()

        self.assertEqual(args.camera_ns, "gripper_camera")
        self.assertFalse(str(args.camera_ns).startswith("/"))


if __name__ == "__main__":
    unittest.main()
