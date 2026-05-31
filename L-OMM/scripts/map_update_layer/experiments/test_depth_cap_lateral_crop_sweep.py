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


class DepthCapLateralCropSweepTest(unittest.TestCase):
    def test_depth_cap_drop_ratio_for_small_matrix(self) -> None:
        from depth_cap_lateral_crop_sweep import run_sweep

        depth = np.array([[0.5, 1.5], [0.0, 0.8]], dtype=np.float64)
        rows = run_sweep(depth=depth, depth_caps=[1.0], crop_values=[2])

        self.assertAlmostEqual(rows[0]["drop_ratio_depth"], 0.5)

    def test_lateral_crop_combined_drop_ratio_for_small_matrix(self) -> None:
        from depth_cap_lateral_crop_sweep import run_sweep

        depth = np.ones((2, 4), dtype=np.float64)
        rows = run_sweep(depth=depth, depth_caps=[2.0], crop_values=[1])

        self.assertAlmostEqual(rows[0]["drop_ratio_lateral"], 0.5)
        self.assertAlmostEqual(rows[0]["drop_ratio_combined"], 0.5)

    def test_sweep_parse_helpers(self) -> None:
        from depth_cap_lateral_crop_sweep import float_list, int_list

        self.assertEqual(float_list("0.5,1.0"), [0.5, 1.0])
        self.assertEqual(int_list("100,200"), [100, 200])


if __name__ == "__main__":
    unittest.main()
