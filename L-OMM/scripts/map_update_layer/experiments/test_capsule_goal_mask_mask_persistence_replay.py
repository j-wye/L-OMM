from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


SCRIPT_PATH = Path(__file__).with_name("capsule_goal_mask_mask_persistence_replay.py")


def load_module():
    spec = importlib.util.spec_from_file_location("mask_persistence_replay", SCRIPT_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class LayerSourceClassificationTest(unittest.TestCase):
    def test_classification_uses_goal_cell_mask_precedence(self) -> None:
        module = load_module()

        membership = {
            "goal_cell_inside_map": True,
            "base_feasible_at_goal_cell": True,
            "occupied_at_goal_cell": False,
            "target_at_goal_cell": False,
            "unknown_at_goal_cell": False,
            "occluded_at_goal_cell": False,
            "sensor_inflated_at_goal_cell": False,
            "occlusion_inflated_at_goal_cell": False,
            "planning_inflation_blocked_at_goal_cell": True,
            "planning_cegis_extra_at_goal_cell": True,
            "planning_blocked_at_goal_cell": True,
            "final_active_at_goal_cell": False,
        }

        label, evidence = module.classify_layer_source(membership)

        self.assertEqual(label, "PLANNING_INFLATION_DIRECT")
        self.assertIn("planning_inflation_blocked_at_goal_cell", evidence)

    def test_missing_direct_masks_keep_row_unavailable(self) -> None:
        module = load_module()

        row = {
            "case": "case_a",
            "seed_id": "pair_001",
            "current_fraction": 0.3,
            "obstacle_fraction": 0.7,
            "trial": 12,
            "requested_goal_cell": [155, 105],
            "requested_goal_xz": [0.45, 0.45],
            "goal_full_capsule_blocked_count": 46,
            "sensor_goal_blocked_count": 0,
            "planning_extra_blocked_cells": 638,
            "capsule_cegis_added_planning_cells": 44,
            "capsule_cegis_iteration_count": 4,
            "row_level_root_cause_class": "CEGIS_FINAL_GOAL_CONTRACT_INTERACTION",
        }

        persisted = module.build_unavailable_membership_row(row, dx=0.0, dz=-0.01)

        self.assertEqual(persisted["layer_source_class"], "LAYER_SOURCE_UNAVAILABLE")
        self.assertEqual(persisted["goal_cell_inside_map"], "UNAVAILABLE")
        self.assertEqual(persisted["sensor_goal_blocked_count"], 0)
        self.assertEqual(
            persisted["row_level_root_cause_from_previous_audit"],
            "CEGIS_FINAL_GOAL_CONTRACT_INTERACTION",
        )

    def test_partial_upstream_context_can_classify_planning_inflation_direct(self) -> None:
        module = load_module()

        row = {
            "case": "case_b",
            "seed_id": "pair_002",
            "requested_goal_cell": [155, 105],
            "goal_cell_inside_map": True,
            "base_feasible_at_goal_cell": True,
            "occupied_at_goal_cell": False,
            "target_at_goal_cell": False,
            "unknown_at_goal_cell": False,
            "occluded_at_goal_cell": False,
            "sensor_inflated_at_goal_cell": False,
            "occlusion_inflated_at_goal_cell": False,
            "planning_inflation_blocked_at_goal_cell": True,
            "planning_cegis_extra_at_goal_cell": "UNAVAILABLE",
            "planning_blocked_at_goal_cell": True,
            "final_active_at_goal_cell": False,
        }

        persisted = module.build_membership_row(row, dx=0.0, dz=-0.01)

        self.assertEqual(persisted["layer_source_class"], "PLANNING_INFLATION_DIRECT")
        self.assertIn("planning_inflation_blocked_at_goal_cell", persisted["layer_source_evidence"])

    def test_loads_cegis_extra_logging_manifest_root(self) -> None:
        module = load_module()

        root = Path("path") / "_tmp_replay_manifest_test"
        manifest = root / "stage_d_logged_rows" / "logged_context_manifest.jsonl"
        manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.write_text(
            json.dumps({"case": "case_logged", "planning_cegis_extra_at_goal_cell": True}) + "\n",
            encoding="utf-8",
        )

        rows = module.load_upstream_context_rows(root)

        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["case"], "case_logged")
        self.assertEqual(rows[0]["planning_cegis_extra_at_goal_cell"], True)


if __name__ == "__main__":
    unittest.main()
