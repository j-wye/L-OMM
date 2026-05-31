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


def make_handle(*, shape=(3, 4), x0=-0.1, z0=0.9, resolution_m=0.1, target_y=0.0):
    from map_handle import MapHandle

    return MapHandle(
        map_path="synthetic",
        meta_path="synthetic",
        mu_grid=np.ones(shape, dtype=np.float64),
        pitch_grid=np.zeros(shape, dtype=np.float64),
        q_grid=None,
        meta={},
        resolution_m=float(resolution_m),
        x0=float(x0),
        z0=float(z0),
        target_y=float(target_y),
        tag="synthetic_cell_projection",
    )


def intrinsics():
    from map_update_layer.perception_to_map import CameraIntrinsics

    return CameraIntrinsics(fx=10.0, fy=10.0, cx=2.0, cy=2.0, width=5, height=5)


def estimator(**kwargs):
    from map_update_layer.cell_projection_occupancy import (
        CellProjectionOccupancyConfig,
        CellProjectionOccupancyEstimator,
    )

    config = CellProjectionOccupancyConfig(
        y_plane=kwargs.pop("y_plane", 0.0),
        depth_min_m=kwargs.pop("depth_min_m", 0.1),
        depth_max_m=kwargs.pop("depth_max_m", 1.3),
        footprint_radius_px=kwargs.pop("footprint_radius_px", 0),
        min_valid_coverage=kwargs.pop("min_valid_coverage", 0.5),
        transform_slack_m=kwargs.pop("transform_slack_m", 0.002),
        cell_size_slack_scale=kwargs.pop("cell_size_slack_scale", 0.5),
        **kwargs,
    )
    return CellProjectionOccupancyEstimator(config)


class CellProjectionOccupancyTest(unittest.TestCase):
    def test_projected_cell_outside_image_produces_no_update(self) -> None:
        est = estimator()
        handle = make_handle(x0=10.0)
        depth = np.ones((5, 5), dtype=np.float64)

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertEqual(int(np.count_nonzero(evidence.fov_mask)), 0)
        self.assertEqual(int(np.count_nonzero(evidence.occupied_mask)), 0)
        self.assertEqual(int(np.count_nonzero(evidence.occluded_mask)), 0)
        self.assertEqual(int(np.count_nonzero(evidence.unknown_mask)), 0)

    def test_cell_outside_expected_depth_range_produces_no_update(self) -> None:
        est = estimator(depth_max_m=0.95)
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertFalse(evidence.fov_mask[1, 1])
        self.assertFalse(evidence.free_mask[1, 1])
        self.assertFalse(evidence.unknown_mask[1, 1])

    def test_invalid_depth_inside_fov_produces_unknown(self) -> None:
        est = estimator()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[2, 2] = 0.0

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.fov_mask[1, 1])
        self.assertTrue(evidence.unknown_mask[1, 1])
        self.assertFalse(evidence.free_mask[1, 1])

    def test_on_plane_hit_at_expected_depth_produces_occupied(self) -> None:
        est = estimator()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[2, 2] = 1.0

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.occupied_mask[1, 1])
        self.assertFalse(evidence.occluded_mask[1, 1])

    def test_on_plane_closer_hit_makes_query_cell_occluded_not_occupied(self) -> None:
        est = estimator()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[2, 2] = 1.0

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.occupied_mask[1, 1])
        self.assertTrue(evidence.occluded_mask[1, 2])
        self.assertFalse(evidence.occupied_mask[1, 2])

    def test_off_plane_closer_hit_produces_occluded(self) -> None:
        est = estimator(y_plane=0.1)
        handle = make_handle(target_y=0.1)
        depth = np.ones((5, 5), dtype=np.float64)
        depth[3, 2] = 0.8

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.occluded_mask[1, 1])
        self.assertFalse(evidence.occupied_mask[1, 1])

    def test_off_plane_hit_at_or_behind_expected_depth_produces_free(self) -> None:
        est = estimator(y_plane=0.1)
        handle = make_handle(target_y=0.1)
        depth = np.ones((5, 5), dtype=np.float64) * 1.2

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.free_mask[1, 1])
        self.assertFalse(evidence.occupied_mask[1, 1])
        self.assertFalse(evidence.occluded_mask[1, 1])

    def test_free_evidence_requires_valid_footprint_coverage(self) -> None:
        est = estimator(y_plane=0.1, footprint_radius_px=1, min_valid_coverage=0.6)
        handle = make_handle(target_y=0.1)
        depth = np.zeros((5, 5), dtype=np.float64)
        depth[3, 2] = 1.2

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.unknown_mask[1, 1])
        self.assertFalse(evidence.free_mask[1, 1])

    def test_single_noisy_pixel_does_not_flip_robust_footprint(self) -> None:
        est = estimator(footprint_radius_px=1, min_valid_coverage=0.6, robust_depth_percentile=20.0)
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[1:4, 1:4] = 1.0
        depth[1, 1] = 0.4

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertTrue(evidence.occupied_mask[1, 1])
        self.assertFalse(evidence.occluded_mask[1, 1])

    def test_tolerance_reports_depth_noise_and_cell_size_slack(self) -> None:
        est = estimator(transform_slack_m=0.003, cell_size_slack_scale=0.5)
        handle = make_handle(resolution_m=0.1)
        depth = np.ones((5, 5), dtype=np.float64)

        evidence = est.build(depth_m=depth, intrinsics=intrinsics(), T_base_cam=np.eye(4), handle=handle)

        self.assertGreater(float(evidence.stats["cell_projection_depth_tolerance_m_max"]), 0.05)
        self.assertAlmostEqual(float(evidence.stats["cell_projection_transform_slack_m"]), 0.003)

    def test_valid_depth_classifies_even_when_unknown_candidate_mask_is_false(self) -> None:
        est = estimator()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        unknown_candidate = np.zeros_like(depth, dtype=bool)

        evidence = est.build(
            depth_m=depth,
            intrinsics=intrinsics(),
            T_base_cam=np.eye(4),
            handle=handle,
            unknown_candidate_mask=unknown_candidate,
        )

        self.assertTrue(evidence.fov_mask[1, 1])
        self.assertTrue(evidence.occupied_mask[1, 1])
        self.assertGreater(int(evidence.stats["cell_projection_finite_footprint_pixels"]), 0)
        self.assertGreater(int(evidence.stats["cell_projection_measured_valid_cells"]), 0)

    def test_invalid_depth_without_unknown_candidate_is_no_update(self) -> None:
        est = estimator()
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[2, 2] = 0.0
        unknown_candidate = np.zeros_like(depth, dtype=bool)

        evidence = est.build(
            depth_m=depth,
            intrinsics=intrinsics(),
            T_base_cam=np.eye(4),
            handle=handle,
            unknown_candidate_mask=unknown_candidate,
        )

        self.assertFalse(evidence.fov_mask[1, 1])
        self.assertFalse(evidence.unknown_mask[1, 1])
        self.assertFalse(evidence.occupied_mask[1, 1])

    def test_mixed_footprint_uses_finite_depth_not_unknown_candidate_intersection(self) -> None:
        est = estimator(footprint_radius_px=1, min_valid_coverage=0.2)
        handle = make_handle()
        depth = np.ones((5, 5), dtype=np.float64)
        depth[1:4, 1:4] = 1.0
        depth[1, 1] = 0.0
        depth[1, 2] = 0.0
        unknown_candidate = np.zeros_like(depth, dtype=bool)
        unknown_candidate[1, 1] = True
        unknown_candidate[1, 2] = True

        evidence = est.build(
            depth_m=depth,
            intrinsics=intrinsics(),
            T_base_cam=np.eye(4),
            handle=handle,
            unknown_candidate_mask=unknown_candidate,
        )

        self.assertTrue(evidence.occupied_mask[1, 1])
        self.assertFalse(evidence.unknown_mask[1, 1])
        self.assertGreater(int(evidence.stats["cell_projection_unknown_candidate_pixels"]), 0)
        self.assertGreater(int(evidence.stats["cell_projection_finite_footprint_pixels"]), 0)


if __name__ == "__main__":
    unittest.main()
