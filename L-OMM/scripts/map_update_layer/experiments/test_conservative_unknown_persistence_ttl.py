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


class ConservativeUnknownPersistenceTTLTest(unittest.TestCase):
    def test_unknown_cell_counter_decrements_and_expires_from_blocked_mask(self) -> None:
        from map_update_layer.sticky_map_manager import StickyMapManager, StickyMapParams

        ttl = 3
        manager = StickyMapManager(
            params=StickyMapParams(inflation_m=0.0, conservative_unknown_persistence=ttl)
        )
        state = manager.initialize({"occupied_mask": np.zeros((5, 5), dtype=bool)})
        fov = np.ones((5, 5), dtype=bool)
        unknown = np.zeros((5, 5), dtype=bool)
        unknown[2, 2] = True

        state = manager.update(state, {"unknown_mask": unknown}, fov)
        self.assertEqual(int(state.unknown_persistence_counter[2, 2]), ttl)
        self.assertTrue(state.blocked_mask[2, 2])

        empty_observation = {"occupied_mask": np.zeros((5, 5), dtype=bool)}
        state = manager.update(state, empty_observation, fov)
        self.assertEqual(int(state.unknown_persistence_counter[2, 2]), ttl - 1)
        self.assertTrue(state.blocked_mask[2, 2])

        state = manager.update(state, empty_observation, fov)
        self.assertEqual(int(state.unknown_persistence_counter[2, 2]), ttl - 2)
        self.assertTrue(state.blocked_mask[2, 2])

        state = manager.update(state, empty_observation, fov)
        self.assertEqual(int(state.unknown_persistence_counter[2, 2]), 0)
        self.assertFalse(state.blocked_mask[2, 2])


if __name__ == "__main__":
    unittest.main()
