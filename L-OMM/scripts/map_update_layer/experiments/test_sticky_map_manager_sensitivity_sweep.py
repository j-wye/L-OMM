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
EXPERIMENTS_ROOT = Path(__file__).resolve().parent
if str(EXPERIMENTS_ROOT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS_ROOT))


class StickyMapManagerSensitivitySweepTest(unittest.TestCase):
    def test_known_unblock_latency_matches_free_threshold(self) -> None:
        from sticky_map_manager_sensitivity_sweep import known_unblock_latency

        self.assertEqual(known_unblock_latency(3), 3)

    def test_flicker_pattern_creates_false_unblock_for_threshold_one(self) -> None:
        from sticky_map_manager_sensitivity_sweep import known_false_unblock_count

        self.assertGreaterEqual(known_false_unblock_count(1), 1)

    def test_inflation_overhead_counts_neighbor_cells(self) -> None:
        from sticky_map_manager_sensitivity_sweep import inflation_overhead_cells

        occ = np.zeros((5, 5), dtype=bool)
        occ[2, 2] = True

        self.assertEqual(inflation_overhead_cells(occupied_mask=occ, inflation_m=0.01, resolution_m=0.01), 8)

    def test_sweep_config_parse_accepts_inline_json(self) -> None:
        from sticky_map_manager_sensitivity_sweep import load_sweep_config

        config = load_sweep_config('{"n_free_frames_to_unblock":[2],"inflation_m":[0.01]}')

        self.assertEqual(config["n_free_frames_to_unblock"], [2.0])
        self.assertEqual(config["inflation_m"], [0.01])
        self.assertIn("conservative_unknown_persistence", config)

    def test_unknown_persistence_changes_sweep_output(self) -> None:
        from sticky_map_manager_sensitivity_sweep import run_one

        short = run_one(
            n_free_frames_to_unblock=3,
            conservative_unknown_persistence=1,
            inflation_m=0.0,
            n_frames=16,
            seed=4,
        )
        long = run_one(
            n_free_frames_to_unblock=3,
            conservative_unknown_persistence=5,
            inflation_m=0.0,
            n_frames=16,
            seed=4,
        )

        self.assertNotEqual(
            short["unknown_persistence_observed_effect"],
            "none_in_current_manager_contract",
        )
        self.assertGreater(
            int(long["unknown_persistence_blocked_frames"]),
            int(short["unknown_persistence_blocked_frames"]),
        )


if __name__ == "__main__":
    unittest.main()
