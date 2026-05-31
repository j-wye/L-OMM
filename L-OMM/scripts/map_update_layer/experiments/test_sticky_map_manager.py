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


class StickyMapManagerTest(unittest.TestCase):
    def test_first_frame_initializes_from_snapshot_occupied_mask(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        observed = np.zeros((5, 5), dtype=bool)
        observed[2, 2] = True
        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0))

        state = manager.initialize({"occupied_mask": observed})

        self.assertTrue(state.occupied_mask[2, 2])
        self.assertTrue(state.blocked_mask[2, 2])
        self.assertEqual(state.last_update_frame_id, 0)

    def test_fov_obstacle_appears_blocked_in_one_frame(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0))
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        occ = np.zeros((5, 5), dtype=bool)
        occ[1, 1] = True

        next_state = manager.update(state, {"occupied_mask": occ}, np.ones((5, 5), dtype=bool))

        self.assertTrue(next_state.occupied_mask[1, 1])
        self.assertTrue(next_state.blocked_mask[1, 1])

    def test_obstacle_unblocks_only_after_n_free_frames(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, n_free_frames_to_unblock=3))
        occ = np.zeros((5, 5), dtype=bool)
        occ[2, 2] = True
        state = manager.initialize({"occupied_mask": occ})
        fov = np.ones((5, 5), dtype=bool)
        free = np.zeros((5, 5), dtype=bool)
        free[2, 2] = True

        for _ in range(2):
            state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool), "free_mask": free}, fov)
            self.assertTrue(state.occupied_mask[2, 2])
        state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool), "free_mask": free}, fov)

        self.assertFalse(state.occupied_mask[2, 2])

    def test_outside_fov_cell_preserves_previous_state(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, n_free_frames_to_unblock=1))
        occ = np.zeros((5, 5), dtype=bool)
        occ[4, 4] = True
        state = manager.initialize({"occupied_mask": occ})
        fov = np.zeros((5, 5), dtype=bool)
        free = np.ones((5, 5), dtype=bool)

        state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool), "free_mask": free}, fov)

        self.assertTrue(state.occupied_mask[4, 4])

    def test_sticky_state_can_be_applied_to_runtime_snapshot(self) -> None:
        from map_update_layer.active_map_snapshot import ActiveMapSnapshot
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, n_free_frames_to_unblock=1))
        occ = np.zeros((5, 5), dtype=bool)
        occ[4, 4] = True
        state = manager.initialize({"occupied_mask": occ})
        state = manager.update(
            state,
            {"occupied_mask": np.zeros((5, 5), dtype=bool), "free_mask": np.ones((5, 5), dtype=bool)},
            np.zeros((5, 5), dtype=bool),
        )
        current = ActiveMapSnapshot(
            handle=SimpleNamespace(shape=(5, 5)),
            base_feasible_mask=np.ones((5, 5), dtype=bool),
            final_active_mask=np.ones((5, 5), dtype=bool),
            blocked_mask=np.zeros((5, 5), dtype=bool),
            occupied_mask=np.zeros((5, 5), dtype=bool),
            source="current_frame",
        )

        runtime = manager.apply_to_snapshot(current, state)

        self.assertTrue(runtime.blocked_mask[4, 4])
        self.assertTrue(runtime.occupied_mask[4, 4])
        self.assertFalse(runtime.final_active_mask[4, 4])
        self.assertEqual(runtime.stats["sticky_runtime_applied"], 1.0)

    def test_standard_snapshot_validator_passes_on_sticky_applied_snapshot(self) -> None:
        # G1: apply_to_snapshot must keep layer_masks["blocked"] consistent with
        # blocked_mask so the standard SnapshotValidator.assert_valid passes (no
        # private bypass needed downstream).
        from map_update_layer.active_map_snapshot import ActiveMapSnapshot
        from map_update_layer.snapshot_validator import SnapshotValidator
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, n_free_frames_to_unblock=1))
        occ = np.zeros((6, 6), dtype=bool)
        occ[4, 4] = True
        occ[1, 2] = True
        state = manager.initialize({"occupied_mask": occ})
        # A current raw snapshot whose own "blocked" layer disagrees with the sticky
        # state (the historical false-fail scenario): pre-sticky blocked is empty.
        current = ActiveMapSnapshot(
            handle=SimpleNamespace(shape=(6, 6)),
            base_feasible_mask=np.ones((6, 6), dtype=bool),
            final_active_mask=np.ones((6, 6), dtype=bool),
            blocked_mask=np.zeros((6, 6), dtype=bool),
            occupied_mask=np.zeros((6, 6), dtype=bool),
            source="current_frame",
            layer_masks={"blocked": np.zeros((6, 6), dtype=bool)},
        )

        runtime = manager.apply_to_snapshot(current, state)

        # The cross-checked layer now equals the sticky blocked_mask.
        self.assertTrue(np.array_equal(runtime.layer_masks["blocked"], runtime.blocked_mask))
        # The standard validator passes without raising (no _core_invariant_ok bypass).
        validator = SnapshotValidator()
        metrics = validator.validate(runtime)
        self.assertEqual(metrics["snapshot_valid"], 1.0)
        self.assertEqual(metrics["blocked_matches_layers"], 1.0)
        validator.assert_valid(runtime)
        # Sticky STATE is unchanged by the relabel: blocked cells still present.
        self.assertTrue(runtime.blocked_mask[4, 4])
        self.assertTrue(runtime.blocked_mask[1, 2])

    def test_flicker_resets_free_streak_counter(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, n_free_frames_to_unblock=2))
        occ = np.zeros((5, 5), dtype=bool)
        occ[2, 2] = True
        state = manager.initialize({"occupied_mask": occ})
        fov = np.ones((5, 5), dtype=bool)
        free = np.zeros((5, 5), dtype=bool)
        free[2, 2] = True

        state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool), "free_mask": free}, fov)
        state = manager.update(state, {"occupied_mask": occ, "free_mask": np.zeros((5, 5), dtype=bool)}, fov)
        state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool), "free_mask": free}, fov)

        self.assertTrue(state.occupied_mask[2, 2])
        self.assertEqual(int(state.free_streak_counter[2, 2]), 1)

    def test_inflation_blocks_neighboring_cells(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.01, resolution_m=0.01))
        occ = np.zeros((5, 5), dtype=bool)
        occ[2, 2] = True

        state = manager.initialize({"occupied_mask": occ})

        self.assertTrue(state.blocked_mask[2, 2])
        self.assertTrue(state.blocked_mask[2, 3])
        self.assertTrue(state.blocked_mask[3, 2])

    def test_unknown_observation_persists_then_expires(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, conservative_unknown_persistence=3))
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        fov = np.ones((5, 5), dtype=bool)
        unknown = np.zeros((5, 5), dtype=bool)
        unknown[2, 2] = True

        state = manager.update(state, {"unknown_mask": unknown}, fov)
        self.assertFalse(state.occupied_mask[2, 2])
        self.assertTrue(state.blocked_mask[2, 2])

        for _ in range(2):
            state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool)}, fov)
            self.assertTrue(state.blocked_mask[2, 2])

        state = manager.update(state, {"occupied_mask": np.zeros((5, 5), dtype=bool)}, fov)
        self.assertFalse(state.blocked_mask[2, 2])

    def test_unknown_reobservation_refreshes_persistence_window(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        manager = StickyMapManager(params=StickyMapParams(inflation_m=0.0, conservative_unknown_persistence=3))
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        fov = np.ones((5, 5), dtype=bool)
        unknown = np.zeros((5, 5), dtype=bool)
        unknown[1, 3] = True
        empty = {"occupied_mask": np.zeros((5, 5), dtype=bool)}

        state = manager.update(state, {"unknown_mask": unknown}, fov)
        state = manager.update(state, empty, fov)
        state = manager.update(state, {"unknown_mask": unknown}, fov)
        state = manager.update(state, empty, fov)
        state = manager.update(state, empty, fov)

        self.assertTrue(state.blocked_mask[1, 3])

        state = manager.update(state, empty, fov)
        self.assertFalse(state.blocked_mask[1, 3])


if __name__ == "__main__":
    unittest.main()
