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


def make_handle(shape=(8, 8)):
    from map_handle import MapHandle

    mu = np.ones(shape, dtype=np.float64)
    return MapHandle(
        map_path="synthetic",
        meta_path="synthetic",
        mu_grid=mu,
        pitch_grid=np.zeros(shape, dtype=np.float64),
        q_grid=None,
        meta={},
        resolution_m=0.01,
        x0=0.0,
        z0=0.0,
        target_y=0.047,
        tag="synthetic",
    )


class ActiveMapSnapshot2ClassTest(unittest.TestCase):
    def test_existing_snapshot_creation_stays_backward_compatible(self) -> None:
        from map_update_layer.active_map_snapshot import ActiveMapSnapshot

        handle = make_handle()
        base = np.ones(handle.shape, dtype=bool)
        blocked = np.zeros(handle.shape, dtype=bool)

        snapshot = ActiveMapSnapshot(
            handle=handle,
            base_feasible_mask=base,
            final_active_mask=base,
            blocked_mask=blocked,
        )

        self.assertIsNone(snapshot.occupied_mask)
        self.assertEqual(snapshot.layer_masks, {})

    def test_map_update_layer_snapshot_exposes_target_as_occupied(self) -> None:
        from map_update_layer.map_update_layer import MapUpdateLayer
        from map_update_layer.map_update_request import MapUpdateRequest

        handle = make_handle()
        request = MapUpdateRequest(
            occupied_rects=((0.01, 0.03, 0.02, 0.02),),
            target_rects=((0.04, 0.05, 0.05, 0.04),),
            inflation_m=0.0,
        )

        snapshot = MapUpdateLayer().build(handle, mu_min=0.0, request=request)
        expected = snapshot.layer_masks["occupied"] | snapshot.layer_masks["target"]

        self.assertIsNotNone(snapshot.occupied_mask)
        self.assertTrue(np.array_equal(snapshot.occupied_mask, expected))

    def test_blocked_mask_is_superset_of_occupied_and_unknown(self) -> None:
        from map_update_layer.map_update_layer import MapUpdateLayer
        from map_update_layer.map_update_request import MapUpdateRequest

        handle = make_handle()
        request = MapUpdateRequest(
            occupied_rects=((0.01, 0.03, 0.02, 0.02),),
            unknown_rects=((0.04, 0.05, 0.05, 0.04),),
            inflation_m=0.0,
        )

        snapshot = MapUpdateLayer().build(handle, mu_min=0.0, request=request)
        occupied_or_unknown = snapshot.occupied_mask | snapshot.layer_masks["unknown"]

        self.assertTrue(np.all(snapshot.blocked_mask[occupied_or_unknown]))


if __name__ == "__main__":
    unittest.main()
