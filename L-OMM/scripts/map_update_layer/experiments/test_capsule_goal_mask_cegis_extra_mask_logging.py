from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest

import numpy as np


SCRIPT_PATH = Path(__file__).with_name("capsule_goal_mask_cegis_extra_mask_logging.py")


def load_module():
    spec = importlib.util.spec_from_file_location("cegis_extra_logging", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CegisExtraMaskLoggingTest(unittest.TestCase):
    def test_goal_cell_membership_comes_from_dense_masks(self) -> None:
        module = load_module()
        shape = (4, 5)
        zeros = np.zeros(shape, dtype=bool)
        planning_inflation = zeros.copy()
        cegis_extra = zeros.copy()
        planning_blocked = zeros.copy()
        final_active = np.ones(shape, dtype=bool)
        cegis_extra[2, 3] = True
        planning_blocked[2, 3] = True
        final_active[2, 3] = False

        row = module.build_manifest_row_from_masks(
            row_id=7,
            source_row={
                "case": "case_a",
                "seed_id": "pair_001",
                "current_fraction": 0.3,
                "obstacle_fraction": 0.7,
                "trial": 11,
                "requested_goal_cell": [2, 3],
                "requested_goal_xz": [0.2, 0.3],
                "goal_full_capsule_blocked_count": 46,
                "sensor_goal_blocked_count": 0,
                "planning_extra_blocked_cells": 9,
                "capsule_cegis_added_planning_cells": 1,
                "capsule_cegis_iteration_count": 2,
                "invalid_reason_code": "goal_final_mask_blocked",
                "row_level_root_cause_class": "CEGIS_FINAL_GOAL_CONTRACT_INTERACTION",
            },
            geometry={"shape": [4, 5], "x0": 0.0, "z0": 0.0, "resolution_m": 0.01},
            layer_masks={
                "occupied": zeros,
                "target": zeros,
                "unknown": zeros,
                "occluded": zeros,
                "sensor_inflated": zeros,
                "sensor_inflation_added": zeros,
                "occlusion_inflated": zeros,
                "occlusion_inflation_added": zeros,
                "sensor_blocked_mask": zeros,
                "planning_inflation_blocked_mask": planning_inflation,
                "planning_cegis_extra_mask": cegis_extra,
                "planning_blocked_mask": planning_blocked,
                "planning_extra_blocked_mask": planning_blocked,
                "final_active_mask": final_active,
            },
            refs={
                "sensor_event_snapshot_ref": "synthetic",
                "active_map_snapshot_layer_masks_ref": "layer_masks_per_row.npz",
                "semantic_layer_mask_ref": "layer_masks_per_row.npz",
                "sensor_inflation_layer_mask_ref": "layer_masks_per_row.npz",
                "occlusion_inflation_layer_mask_ref": "layer_masks_per_row.npz",
                "base_feasible_mask_ref": "implicit_all_true_synthetic_fixture_base",
                "planning_inflation_blocked_mask_ref": "layer_masks_per_row.npz",
                "planning_cegis_extra_mask_ref": "cegis_extra_masks_per_row.npz",
                "planning_blocked_mask_ref": "planning_blocked_masks_per_row.npz",
                "planning_extra_blocked_mask_ref": "planning_extra_blocked_masks_per_row.npz",
                "final_active_mask_ref": "final_active_masks_per_row.npz",
            },
            dx=0.0,
            dz=-0.01,
        )

        self.assertEqual(row["context_completeness_status"], module.CONTEXT_COMPLETE)
        self.assertEqual(row["planning_cegis_extra_at_goal_cell"], True)
        self.assertEqual(row["final_active_at_goal_cell"], False)
        self.assertEqual(row["layer_source_class"], "CEGIS_EXTRA_DIRECT")
        self.assertEqual(row["missing_context_fields"], [])

    def test_marker_scan_requires_removable_logging_marker(self) -> None:
        module = load_module()

        scan = module.logging_marker_scan([SCRIPT_PATH])

        self.assertEqual(scan["logging_marker_present"], "PASS")
        self.assertEqual(scan["logging_semantics_read_only"], "PASS")


if __name__ == "__main__":
    unittest.main()
