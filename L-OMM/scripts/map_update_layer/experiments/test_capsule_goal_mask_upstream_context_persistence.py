from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT_PATH = Path(__file__).with_name("capsule_goal_mask_upstream_context_persistence.py")


def load_module():
    spec = importlib.util.spec_from_file_location("upstream_context", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class UpstreamContextPersistenceTest(unittest.TestCase):
    def test_rect_mask_uses_cell_area_intersection(self) -> None:
        module = load_module()
        geometry = {"shape": [10, 10], "x0": 0.0, "z0": 0.0, "resolution_m": 1.0}

        mask = module.rect_mask(geometry, [2.1, 2.1, 2.9, 2.9], margin_m=0.0)

        self.assertTrue(mask[2, 2])
        self.assertFalse(mask[1, 2])
        self.assertTrue(mask[3, 2])
        self.assertEqual(int(np.count_nonzero(mask)), 4)

    def test_partial_planning_context_keeps_cegis_missing(self) -> None:
        module = load_module()
        row = {
            "case": "case_a",
            "planning_inflation_blocked_at_goal_cell": "True",
            "planning_cegis_extra_at_goal_cell": module.UNAVAILABLE,
            "planning_blocked_at_goal_cell": "True",
            "final_active_at_goal_cell": "False",
            "missing_context_fields": ["planning_cegis_extra_mask_per_row"],
        }

        self.assertEqual(module.row_context_status(row), module.CONTEXT_PARTIAL_PLANNING_ONLY)
        self.assertEqual(module.bool_to_tensor_value(row["planning_inflation_blocked_at_goal_cell"]), 1)
        self.assertEqual(module.bool_to_tensor_value(row["planning_cegis_extra_at_goal_cell"]), -1)

    def test_pack_mask_stack_round_trip_shape(self) -> None:
        module = load_module()
        shape = (4, 5)
        masks = [np.eye(4, 5, dtype=bool), np.zeros(shape, dtype=bool)]

        packed = module.pack_mask_stack(masks, shape)
        unpacked = np.unpackbits(packed, axis=1)[:, : shape[0] * shape[1]].reshape(2, *shape)

        self.assertTrue(np.array_equal(unpacked[0], masks[0]))
        self.assertTrue(np.array_equal(unpacked[1], masks[1]))


if __name__ == "__main__":
    unittest.main()
