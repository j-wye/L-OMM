from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
EXPERIMENTS_ROOT = SCRIPTS_ROOT / "map_update_layer" / "experiments"
if str(EXPERIMENTS_ROOT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

from map_handle import MapHandle  # noqa: E402
from map_update_layer.depth_geometry_evidence import (  # noqa: E402
    DepthGeometryConfig,
    _image_component_unknown_rect,
)
from map_update_layer.perception_to_map import CameraIntrinsics  # noqa: E402


class InvalidDepthUnknownAlternativeProjectionPrototypeTest(unittest.TestCase):
    def _handle(self) -> MapHandle:
        shape = (220, 140)
        return MapHandle(
            map_path="",
            meta_path="",
            mu_grid=np.ones(shape, dtype=np.float64),
            pitch_grid=np.zeros(shape, dtype=np.float64),
            q_grid=None,
            meta={},
            resolution_m=0.01,
            x0=-1.0,
            z0=0.0,
            target_y=0.047,
            tag="test",
        )

    def _inputs(self, *, invalid_pixels: int = 64, valid_depth: float = 0.32) -> tuple[np.ndarray, np.ndarray]:
        depth = np.full((32, 32), valid_depth, dtype=np.float32)
        mask = np.zeros_like(depth, dtype=bool)
        rows = int(np.ceil(invalid_pixels / 8.0))
        mask[12 : 12 + rows, 12:20] = True
        extra = rows * 8 - invalid_pixels
        if extra:
            mask[12 + rows - 1, 20 - extra : 20] = False
        depth[mask] = 0.0
        return depth, mask

    def _intrinsics(self) -> CameraIntrinsics:
        return CameraIntrinsics(fx=90.0, fy=90.0, cx=16.0, cy=16.0, width=32, height=32)

    def _cfg(self) -> DepthGeometryConfig:
        return DepthGeometryConfig(
            pixel_band_half_width_px=16.0,
            depth_min_m=0.05,
            depth_max_m=0.75,
            min_unknown_component_pixels=64,
        )

    def test_production_shell_matches_production_helper(self) -> None:
        from invalid_depth_unknown_alternative_projection_prototype import compute_alternative_rect

        depth, mask = self._inputs()
        record = compute_alternative_rect(
            depth_m=depth,
            intrinsics=self._intrinsics(),
            T_base_cam=np.eye(4, dtype=np.float64),
            handle=self._handle(),
            unknown_candidate_mask=mask,
            variant="production_shell",
            adjacency_radius_px=5,
            cfg=self._cfg(),
        )

        expected = _image_component_unknown_rect(
            handle=self._handle(),
            u_min=12,
            u_max=19,
            v_min=12,
            v_max=19,
            intrinsics=self._intrinsics(),
            T_base_cam=np.eye(4, dtype=np.float64),
            depth_min_m=0.05,
            depth_max_m=0.75,
        )
        self.assertEqual(record["rect"], expected)
        self.assertFalse(record["fallback_to_production_shell"])

    def test_min_depth_only_is_smaller_than_production_shell_with_flat_adjacent_depth(self) -> None:
        from invalid_depth_unknown_alternative_projection_prototype import compute_alternative_rect

        depth, mask = self._inputs(valid_depth=0.32)
        common = dict(
            depth_m=depth,
            intrinsics=self._intrinsics(),
            T_base_cam=np.eye(4, dtype=np.float64),
            handle=self._handle(),
            unknown_candidate_mask=mask,
            adjacency_radius_px=5,
            cfg=self._cfg(),
        )

        production = compute_alternative_rect(variant="production_shell", **common)
        min_depth = compute_alternative_rect(variant="min_depth_only", **common)

        self.assertIsNotNone(min_depth["rect"])
        self.assertGreater(production["rect_cell_count"], min_depth["rect_cell_count"])
        self.assertFalse(min_depth["fallback_to_production_shell"])

    def test_sparse_adjacency_falls_back_to_production_shell(self) -> None:
        from invalid_depth_unknown_alternative_projection_prototype import compute_alternative_rect

        depth, mask = self._inputs()
        depth[:] = 0.0
        common = dict(
            depth_m=depth,
            intrinsics=self._intrinsics(),
            T_base_cam=np.eye(4, dtype=np.float64),
            handle=self._handle(),
            unknown_candidate_mask=mask,
            adjacency_radius_px=5,
            cfg=self._cfg(),
        )

        fallback = compute_alternative_rect(variant="median_adjacent", **common)
        production = compute_alternative_rect(variant="production_shell", **common)

        self.assertTrue(fallback["fallback_to_production_shell"])
        self.assertEqual(fallback["rect"], production["rect"])

    def test_existence_gate_suppresses_subthreshold_invalid_components(self) -> None:
        from invalid_depth_unknown_alternative_projection_prototype import compute_alternative_rect

        depth, mask = self._inputs(invalid_pixels=63)
        record = compute_alternative_rect(
            depth_m=depth,
            intrinsics=self._intrinsics(),
            T_base_cam=np.eye(4, dtype=np.float64),
            handle=self._handle(),
            unknown_candidate_mask=mask,
            variant="median_adjacent",
            adjacency_radius_px=5,
            cfg=self._cfg(),
        )

        self.assertIsNone(record["rect"])
        self.assertEqual(record["rect_cell_count"], 0)
        self.assertFalse(record["fallback_to_production_shell"])


if __name__ == "__main__":
    unittest.main()
