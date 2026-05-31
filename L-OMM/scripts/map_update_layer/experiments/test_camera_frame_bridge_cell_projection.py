from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


def make_handle():
    from map_handle import MapHandle

    return MapHandle(
        map_path="synthetic",
        meta_path="synthetic",
        mu_grid=np.ones((3, 4), dtype=np.float64),
        pitch_grid=np.zeros((3, 4), dtype=np.float64),
        q_grid=None,
        meta={},
        resolution_m=0.1,
        x0=-0.1,
        z0=0.9,
        target_y=0.0,
        tag="synthetic_bridge_cell_projection",
    )


def make_bridge(*, speed_prefilter_enabled: bool = False):
    from map_update_layer.camera_frame_bridge import CameraFrameMapUpdateBridge
    from map_update_layer.cell_projection_occupancy import (
        CellProjectionOccupancyConfig,
        CellProjectionOccupancyEstimator,
    )
    from map_update_layer.perception_to_map import PerceptionToMapAdapter

    estimator = CellProjectionOccupancyEstimator(
        CellProjectionOccupancyConfig(
            y_plane=0.0,
            depth_min_m=0.1,
            depth_max_m=1.3,
            footprint_radius_px=0,
            transform_slack_m=0.002,
            cell_size_slack_scale=0.5,
        )
    )
    return CameraFrameMapUpdateBridge(
        adapter=PerceptionToMapAdapter(y_plane=0.0, delta_y_static_m=0.01, sensor_inflation_floor_m=0.0),
        depth_geometry_enabled=False,
        speed_prefilter_enabled=bool(speed_prefilter_enabled),
        cell_projection=estimator,
        cell_projection_enabled=True,
    )


def frame(depth: np.ndarray):
    from map_update_layer.perception_to_map import CameraIntrinsics

    return {
        "depth": depth,
        "intrinsics": CameraIntrinsics(fx=10.0, fy=10.0, cx=2.0, cy=2.0, width=5, height=5),
        "timestamp_s": 1.0,
    }


class CameraFrameBridgeCellProjectionTest(unittest.TestCase):
    def test_detection_target_is_preserved_while_cell_projection_is_unioned(self) -> None:
        bridge = make_bridge()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        detections = [{"semantic_type": "target", "bbox": (2, 2, 2, 2), "score": 1.0}]

        result = bridge.process(
            frame(depth),
            detections,
            handle,
            mu_min=0.0,
            T_base_cam=np.eye(4),
            sequence_id=3,
        )

        self.assertGreater(len(result.request.target_rects), 0)
        self.assertGreater(len(result.request.occupied_rects), 0)
        self.assertIn("cell_projection_occupied", result.snapshot.layer_masks)
        self.assertIn("cell_projection_fov", result.snapshot.layer_masks)
        self.assertTrue(result.snapshot.layer_masks["target"].any())

    def test_cell_occluded_and_unknown_reach_blocked_mask_and_free_is_diagnostic_only(self) -> None:
        bridge = make_bridge()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64) * 1.2
        depth[2, 2] = 1.0
        depth[2, 3] = 0.0

        result = bridge.process(
            frame(depth),
            [],
            handle,
            mu_min=0.0,
            T_base_cam=np.eye(4),
            sequence_id=4,
        )

        layers = result.snapshot.layer_masks
        self.assertTrue(layers["cell_projection_occluded"][1, 2])
        self.assertTrue(layers["cell_projection_unknown"].any())
        self.assertTrue(result.snapshot.blocked_mask[1, 2])
        self.assertIn("cell_projection_free", layers)
        self.assertFalse(np.any(layers["cell_projection_free"] & layers["cell_projection_occupied"]))
        self.assertFalse(np.any(layers["cell_projection_free"] & layers["cell_projection_occluded"]))
        self.assertFalse(np.any(layers["cell_projection_free"] & layers["cell_projection_unknown"]))
        self.assertTrue(np.array_equal(result.snapshot.final_active_mask, result.snapshot.base_feasible_mask & ~result.snapshot.blocked_mask))

    def test_default_speed_prefilter_keeps_valid_depth_available_to_cell_projection(self) -> None:
        bridge = make_bridge(speed_prefilter_enabled=True)
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[2, 3] = 0.0

        result = bridge.process(
            frame(depth),
            [],
            handle,
            mu_min=0.0,
            T_base_cam=np.eye(4),
            sequence_id=5,
        )

        layers = result.snapshot.layer_masks
        dense_valid = (
            int(result.stats["cell_projection_occupied_cells"])
            + int(result.stats["cell_projection_occluded_cells"])
            + int(result.stats["cell_projection_free_cells"])
        )
        self.assertTrue(bool(result.stats["speed_prefilter_enabled"]))
        self.assertGreater(dense_valid, 0)
        self.assertGreater(int(result.stats["cell_projection_occupied_cells"]), 0)
        self.assertGreater(int(result.stats["cell_projection_unknown_cells"]), 0)
        self.assertIn("cell_projection_fov", layers)
        self.assertIn("cell_projection_free", layers)
        self.assertIn("cell_projection_occupied", layers)
        self.assertIn("cell_projection_occluded", layers)
        self.assertIn("cell_projection_unknown", layers)
        self.assertTrue(np.array_equal(result.snapshot.final_active_mask, result.snapshot.base_feasible_mask & ~result.snapshot.blocked_mask))


if __name__ == "__main__":
    unittest.main()
