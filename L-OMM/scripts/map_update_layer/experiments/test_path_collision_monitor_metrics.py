from __future__ import annotations

from pathlib import Path
import sys
import unittest


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))
EXPERIMENTS_ROOT = Path(__file__).resolve().parent
if str(EXPERIMENTS_ROOT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS_ROOT))


class PathCollisionMonitorMetricsTest(unittest.TestCase):
    def test_latency_distribution_reports_percentiles(self) -> None:
        from path_collision_monitor_metrics import latency_distribution

        stats = latency_distribution([1.0, 2.0, 3.0, 4.0])

        self.assertEqual(stats["sample_count"], 4)
        self.assertGreaterEqual(stats["latency_ms_p99"], stats["latency_ms_p50"])

    def test_detection_latency_frames_from_inserted_obstacle(self) -> None:
        from path_collision_monitor_metrics import detection_latency_rows

        rows = [
            {"frame_index": 3, "obstacle_inserted": True, "reported_collision": False},
            {"frame_index": 4, "obstacle_inserted": True, "reported_collision": False},
            {"frame_index": 5, "obstacle_inserted": True, "reported_collision": True},
        ]

        latency = detection_latency_rows(rows)[0]

        self.assertEqual(latency["detection_latency_frames"], 2)

    def test_confusion_matrix_known_tp_fp_fn(self) -> None:
        from path_collision_monitor_metrics import confusion_matrix

        rows = [
            {"truth_collision": True, "reported_collision": True},
            {"truth_collision": False, "reported_collision": True},
            {"truth_collision": True, "reported_collision": False},
            {"truth_collision": False, "reported_collision": False},
        ]
        cm = confusion_matrix(rows)

        self.assertEqual(cm["tp"], 1)
        self.assertEqual(cm["fp"], 1)
        self.assertEqual(cm["fn"], 1)
        self.assertEqual(cm["tn"], 1)


if __name__ == "__main__":
    unittest.main()
