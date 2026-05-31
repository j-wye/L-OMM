from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


class StickyMapOcclusionPersistenceTest(unittest.TestCase):
    def test_occluded_observation_blocks_without_setting_occupied(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, conservative_unknown_persistence=3))
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        occluded = np.zeros((5, 5), dtype=bool)
        occluded[2, 2] = True

        state = manager.update(state, {"occluded_mask": occluded}, np.ones((5, 5), dtype=bool))

        self.assertFalse(state.occupied_mask[2, 2])
        self.assertTrue(state.blocked_mask[2, 2])
        self.assertTrue(hasattr(state, "occluded_persistence_counter"))
        self.assertEqual(int(state.occluded_persistence_counter[2, 2]), 3)

    def test_occluded_persistence_expires_and_repeated_free_clears_it(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(
            params=StickyMapParams(
                inflation_m=0.0,
                conservative_unknown_persistence=4,
                n_free_frames_to_unblock=2,
            )
        )
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        fov = np.ones((5, 5), dtype=bool)
        occluded = np.zeros((5, 5), dtype=bool)
        occluded[2, 2] = True
        free = np.zeros((5, 5), dtype=bool)
        free[2, 2] = True

        state = manager.update(state, {"occluded_mask": occluded}, fov)
        state = manager.update(state, {"free_mask": free}, fov)
        self.assertTrue(state.blocked_mask[2, 2])

        state = manager.update(state, {"free_mask": free}, fov)
        self.assertFalse(state.blocked_mask[2, 2])
        self.assertEqual(int(state.occluded_persistence_counter[2, 2]), 0)

    def test_outside_fov_free_does_not_clear_occluded_state(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, n_free_frames_to_unblock=1))
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        occluded = np.zeros((5, 5), dtype=bool)
        occluded[4, 4] = True
        state = manager.update(state, {"occluded_mask": occluded}, np.ones((5, 5), dtype=bool))

        free = np.ones((5, 5), dtype=bool)
        state = manager.update(state, {"free_mask": free}, np.zeros((5, 5), dtype=bool))

        self.assertTrue(state.blocked_mask[4, 4])

    def test_snapshot_layer_masks_fallback_and_diagnostics(self) -> None:
        from map_update_layer.active_map_snapshot import ActiveMapSnapshot
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, conservative_unknown_persistence=3))
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        occ = np.zeros((5, 5), dtype=bool)
        unk = np.zeros((5, 5), dtype=bool)
        occd = np.zeros((5, 5), dtype=bool)
        free = np.zeros((5, 5), dtype=bool)
        fov = np.ones((5, 5), dtype=bool)
        occ[1, 1] = True
        unk[2, 2] = True
        occd[3, 3] = True
        snapshot = ActiveMapSnapshot(
            handle=SimpleNamespace(shape=(5, 5)),
            base_feasible_mask=np.ones((5, 5), dtype=bool),
            final_active_mask=np.ones((5, 5), dtype=bool),
            blocked_mask=np.zeros((5, 5), dtype=bool),
            occupied_mask=np.zeros((5, 5), dtype=bool),
            layer_masks={
                "occupied": occ,
                "unknown": unk,
                "occluded": occd,
                "cell_projection_free": free,
                "cell_projection_fov": fov,
            },
        )

        state = manager.update(state, snapshot, None)
        runtime = manager.apply_to_snapshot(snapshot, state)

        self.assertTrue(state.occupied_mask[1, 1])
        self.assertTrue(state.blocked_mask[2, 2])
        self.assertTrue(state.blocked_mask[3, 3])
        self.assertIn("sticky_unknown", runtime.layer_masks)
        self.assertIn("sticky_occluded", runtime.layer_masks)
        self.assertTrue(runtime.layer_masks["sticky_occluded"][3, 3])
        self.assertFalse(runtime.occupied_mask[3, 3])


if __name__ == "__main__":
    unittest.main()
