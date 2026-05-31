from __future__ import annotations

import csv
import json
from pathlib import Path
import shutil
import sys
import unittest
import uuid

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


class InvalidDepthUnknownGeometryAuditTest(unittest.TestCase):
    def _tmpdir(self) -> Path:
        path = TEST_TMP_DIR / f"case_{uuid.uuid4().hex}"
        shutil.rmtree(path, ignore_errors=True)
        path.mkdir(parents=True, exist_ok=True)
        self.addCleanup(lambda: shutil.rmtree(path, ignore_errors=True))
        return path

    def test_analyzer_writes_projection_temporal_and_overlap_artifacts(self) -> None:
        from invalid_depth_unknown_geometry_audit import analyze_run

        tmp_path = self._tmpdir()
        run_dir = self._write_single_frame_run(tmp_path / "run", min_unknown_component_pixels=1)

        summary = analyze_run(run_dir=run_dir, out_dir=run_dir / "audit")

        self.assertEqual(summary["frames_analyzed"], 1)
        self.assertTrue((run_dir / "audit" / "invalid_depth_unknown_geometry_per_frame.csv").exists())
        self.assertTrue((run_dir / "audit" / "invalid_depth_unknown_geometry_summary.json").exists())
        self.assertTrue((run_dir / "audit" / "temporal_stability.json").exists())
        self.assertTrue((run_dir / "audit" / "capsule_aware_overlap.json").exists())

    def test_analyzer_applies_production_unknown_component_threshold(self) -> None:
        from invalid_depth_unknown_geometry_audit import analyze_run

        tmp_path = self._tmpdir()
        run_dir = self._write_single_frame_run(tmp_path / "run", min_unknown_component_pixels=64)

        analyze_run(run_dir=run_dir, out_dir=run_dir / "audit")

        csv_path = run_dir / "audit" / "invalid_depth_unknown_geometry_per_frame.csv"
        with csv_path.open("r", newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(len(rows), 1)
        self.assertEqual(int(rows[0]["invalid_pixel_count"]), 0)
        self.assertEqual(int(rows[0]["prod_rect_cell_count"]), 0)
        self.assertEqual(rows[0]["production_consistency_check_ok"], "True")

    def _write_single_frame_run(self, run_dir: Path, *, min_unknown_component_pixels: int) -> Path:
        frames_dir = run_dir / "frames"
        frames_dir.mkdir(parents=True)
        depth = np.full((12, 12), 0.4, dtype=np.float32)
        depth[4:8, 5:7] = 0.0
        unknown_candidate_mask = np.zeros_like(depth, dtype=bool)
        unknown_candidate_mask[4:8, 5:7] = True
        blocked_mask = np.zeros((20, 20), dtype=bool)
        final_active_mask = np.ones((20, 20), dtype=bool)
        base_feasible_mask = np.ones((20, 20), dtype=bool)
        unknown_layer = np.zeros((20, 20), dtype=bool)
        goal_cells = [[10, 6], [10, 7], [10, 8]]
        for _, iz in goal_cells:
            blocked_mask[10, iz] = True
            unknown_layer[10, iz] = True
        np.savez(
            frames_dir / "frame_0000.npz",
            depth_m=depth,
            unknown_candidate_mask=unknown_candidate_mask,
            T_base_cam=np.eye(4, dtype=np.float64),
            blocked_mask=blocked_mask,
            final_active_mask=final_active_mask,
            base_feasible_mask=base_feasible_mask,
            layer_unknown=unknown_layer,
        )
        metadata = {
            "frame_id": 0,
            "intrinsics": {"fx": 10.0, "fy": 10.0, "cx": 6.0, "cy": 6.0, "width": 12, "height": 12},
            "depth_geometry_config": {
                "pixel_band_half_width_px": 3.0,
                "depth_min_m": 0.05,
                "depth_max_m": 0.75,
                "y_plane": 0.047,
                "delta_y_static_m": 0.091,
                "k_sigma": 2.5,
                "min_unknown_component_pixels": min_unknown_component_pixels,
            },
            "map_geometry": {
                "shape": [20, 20],
                "resolution_m": 0.05,
                "x0": -0.5,
                "z0": 0.0,
                "target_y": 0.047,
                "tag": "test",
            },
            "goal_diagnostics": {"goal_corridor_cells": goal_cells},
        }
        (frames_dir / "frame_0000.json").write_text(json.dumps(metadata), encoding="utf-8")
        with (run_dir / "events.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=["frame_id", "frame_artifact", "frame_metadata_artifact"])
            writer.writeheader()
            writer.writerow(
                {
                    "frame_id": 0,
                    "frame_artifact": "frame_0000.npz",
                    "frame_metadata_artifact": "frame_0000.json",
                }
            )
        return run_dir


if __name__ == "__main__":
    unittest.main()
