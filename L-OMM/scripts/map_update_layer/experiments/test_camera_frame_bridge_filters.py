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


class CameraFrameBridgeFiltersTest(unittest.TestCase):
    def _depth_geometry_fixture(self):
        from map_update_layer.depth_geometry_evidence import DepthGeometryConfig, DepthGeometryEvidenceBuilder
        from map_update_layer.perception_to_map import CameraIntrinsics

        class Handle:
            shape = (80, 80)
            resolution_m = 0.02
            x0 = -0.8
            z0 = 0.0

        builder = DepthGeometryEvidenceBuilder(
            DepthGeometryConfig(
                pixel_band_half_width_px=10.0,
                depth_min_m=0.10,
                depth_max_m=0.80,
                min_unknown_component_pixels=2,
                min_occupied_component_cells=1,
            )
        )
        intrinsics = CameraIntrinsics(fx=100.0, fy=100.0, cx=3.5, cy=2.5, width=8, height=6)
        return builder, intrinsics, np.eye(4, dtype=np.float64), Handle()

    def test_depth_cap_zeroes_irrelevant_depth(self) -> None:
        from map_update_layer.camera_frame_bridge import apply_depth_cap

        depth = np.array([[0.2, 1.5], [0.0, np.nan]], dtype=np.float64)

        filtered, stats = apply_depth_cap(depth, max_depth_m=1.0)

        self.assertEqual(filtered[0, 0], 0.2)
        self.assertEqual(filtered[0, 1], 0.0)
        self.assertEqual(filtered[1, 0], 0.0)
        self.assertEqual(filtered[1, 1], 0.0)
        self.assertAlmostEqual(stats["depth_cap_drop_ratio"], 0.75)

    def test_lateral_crop_preserves_center_columns(self) -> None:
        from map_update_layer.camera_frame_bridge import apply_lateral_crop

        image = np.ones((4, 8), dtype=np.float64)

        cropped, stats = apply_lateral_crop(image, half_pixels=2)

        self.assertEqual(cropped.shape, image.shape)
        self.assertTrue(np.all(cropped[:, 2:6] == 1.0))
        self.assertTrue(np.all(cropped[:, :2] == 0.0))
        self.assertTrue(np.all(cropped[:, 6:] == 0.0))
        self.assertAlmostEqual(stats["lateral_crop_keep_ratio"], 0.5)

    def test_speed_prefilter_reports_drop_ratio(self) -> None:
        from map_update_layer.camera_frame_bridge import apply_speed_prefilter

        rgb = np.ones((2, 8, 3), dtype=np.uint8)
        depth = np.ones((2, 8), dtype=np.float64) * 2.0
        depth[:, 3:5] = 0.5

        rgb_filtered, depth_filtered, stats = apply_speed_prefilter(
            rgb,
            depth,
            max_depth_m=1.0,
            crop_half_pixels=2,
        )

        self.assertEqual(rgb_filtered.shape, rgb.shape)
        self.assertEqual(depth_filtered.shape, depth.shape)
        self.assertTrue(np.all(depth_filtered[:, 3:5] == 0.5))
        self.assertEqual(float(depth_filtered[:, :2].sum()), 0.0)
        self.assertGreater(stats["speed_prefilter_total_drop_ratio"], 0.0)

    def test_depth_cap_ignored_pixels_do_not_create_unknown_geometry(self) -> None:
        from map_update_layer.camera_frame_bridge import (
            apply_speed_prefilter,
            speed_prefilter_unknown_candidate_mask,
        )

        builder, intrinsics, T_base_cam, handle = self._depth_geometry_fixture()
        depth = np.ones((6, 8), dtype=np.float64) * 2.0

        _, filtered_depth, _ = apply_speed_prefilter(
            None,
            depth,
            max_depth_m=1.0,
            crop_half_pixels=4,
        )
        unknown_candidate = speed_prefilter_unknown_candidate_mask(
            depth,
            max_depth_m=1.0,
            crop_half_pixels=4,
        )
        evidence = builder.build(
            depth_m=filtered_depth,
            intrinsics=intrinsics,
            T_base_cam=T_base_cam,
            handle=handle,
            unknown_candidate_mask=unknown_candidate,
        )

        self.assertEqual(int(evidence.stats["depth_geometry_unknown_rect_count"]), 0)

    def test_lateral_crop_ignored_columns_do_not_create_unknown_geometry(self) -> None:
        from map_update_layer.camera_frame_bridge import (
            apply_speed_prefilter,
            speed_prefilter_unknown_candidate_mask,
        )

        builder, intrinsics, T_base_cam, handle = self._depth_geometry_fixture()
        depth = np.ones((6, 8), dtype=np.float64) * 0.5

        _, filtered_depth, _ = apply_speed_prefilter(
            None,
            depth,
            max_depth_m=1.0,
            crop_half_pixels=1,
        )
        unknown_candidate = speed_prefilter_unknown_candidate_mask(
            depth,
            max_depth_m=1.0,
            crop_half_pixels=1,
        )
        evidence = builder.build(
            depth_m=filtered_depth,
            intrinsics=intrinsics,
            T_base_cam=T_base_cam,
            handle=handle,
            unknown_candidate_mask=unknown_candidate,
        )

        self.assertEqual(int(evidence.stats["depth_geometry_unknown_rect_count"]), 0)

    def test_true_invalid_depth_inside_kept_region_still_creates_unknown_geometry(self) -> None:
        from map_update_layer.camera_frame_bridge import (
            apply_speed_prefilter,
            speed_prefilter_unknown_candidate_mask,
        )

        builder, intrinsics, T_base_cam, handle = self._depth_geometry_fixture()
        depth = np.ones((6, 8), dtype=np.float64) * 0.5
        depth[:, 3:5] = 0.0

        _, filtered_depth, _ = apply_speed_prefilter(
            None,
            depth,
            max_depth_m=1.0,
            crop_half_pixels=2,
        )
        unknown_candidate = speed_prefilter_unknown_candidate_mask(
            depth,
            max_depth_m=1.0,
            crop_half_pixels=2,
        )
        evidence = builder.build(
            depth_m=filtered_depth,
            intrinsics=intrinsics,
            T_base_cam=T_base_cam,
            handle=handle,
            unknown_candidate_mask=unknown_candidate,
        )

        self.assertGreater(int(evidence.stats["depth_geometry_unknown_rect_count"]), 0)

    def test_build_request_speed_prefilter_uses_adapter_depth(self) -> None:
        from map_update_layer.camera_frame_bridge import CameraFrameMapUpdateBridge
        from map_update_layer.perception_to_map import CameraIntrinsics

        bridge = CameraFrameMapUpdateBridge(
            depth_geometry_enabled=False,
            speed_prefilter_enabled=True,
            depth_relevant_max_m=1.0,
            crop_half_pixels=2,
        )
        frame = {
            "depth": np.ones((4, 8), dtype=np.float64) * 0.5,
            "intrinsics": CameraIntrinsics(fx=100.0, fy=100.0, cx=3.5, cy=1.5, width=8, height=4),
            "timestamp_s": 1.0,
        }

        request = bridge.build_request(frame, [], sequence_id=7)

        self.assertEqual(int(request.sequence_id), 7)
        self.assertTrue(bool(request.adapter_stats["speed_prefilter_enabled"]))
        self.assertIn("speed_prefilter_total_drop_ratio", request.adapter_stats)


if __name__ == "__main__":
    unittest.main()
