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


class FakeCapsule:
    def __init__(self, hit_threshold: float = 0.5) -> None:
        self.hit_threshold = float(hit_threshold)

    def q_hits_mask(self, q_active, handle, mask, dilated_masks=None):
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        hit = bool(q[0] >= self.hit_threshold)
        return hit, int(hit), 1

    def trajectory_metrics(self, q_traj, handle, mask):
        q_arr = np.asarray(q_traj, dtype=np.float64).reshape(-1, 3)
        colliding = int(np.count_nonzero(q_arr[:, 0] >= self.hit_threshold))
        return {
            "capsule_proxy_collision_free": 1.0 if colliding == 0 else 0.0,
            "capsule_proxy_violation_count": float(colliding),
            "capsule_proxy_checked_samples": float(q_arr.shape[0]),
            "capsule_proxy_colliding_ticks": float(colliding),
            "capsule_proxy_total_ticks": float(q_arr.shape[0]),
        }


class FakeHandle:
    def __init__(self) -> None:
        self.resolution_m = 0.01
        self.x0 = 0.0
        self.z0 = 0.0
        self.q_grid = np.zeros((4, 4, 3), dtype=np.float64)
        self.q_grid[2, 2, 0] = 0.8

    @property
    def shape(self):
        return self.q_grid.shape[:2]


class PathCollisionMonitorTest(unittest.TestCase):
    def test_clear_q_trajectory_reports_no_collision(self) -> None:
        from map_update_layer.path_collision_monitor import PathCollisionMonitor

        monitor = PathCollisionMonitor(FakeCapsule(), handle=FakeHandle())
        report = monitor.check({"q_traj": np.zeros((3, 3))}, np.zeros((4, 4), dtype=bool))

        self.assertFalse(report.is_collision)
        self.assertIsNone(report.first_collision_point)
        self.assertEqual(report.colliding_cells_count, 0)

    def test_single_q_trajectory_collision_is_reported(self) -> None:
        from map_update_layer.path_collision_monitor import PathCollisionMonitor

        monitor = PathCollisionMonitor(FakeCapsule(), handle=FakeHandle())
        q_traj = np.array([[0.0, 0.0, 0.0], [0.9, 0.0, 0.0]], dtype=np.float64)
        report = monitor.check({"q_traj": q_traj}, np.zeros((4, 4), dtype=bool))

        self.assertTrue(report.is_collision)
        self.assertEqual(report.colliding_cells_count, 1)
        self.assertEqual(report.checked_sample_count, 2)

    def test_multi_q_trajectory_collision_counts_colliding_ticks(self) -> None:
        from map_update_layer.path_collision_monitor import PathCollisionMonitor

        monitor = PathCollisionMonitor(FakeCapsule(), handle=FakeHandle())
        q_traj = np.array(
            [[0.6, 0.0, 0.0], [0.1, 0.0, 0.0], [0.7, 0.0, 0.0]],
            dtype=np.float64,
        )
        report = monitor.check({"q_traj": q_traj}, np.zeros((4, 4), dtype=bool))

        self.assertTrue(report.is_collision)
        self.assertEqual(report.colliding_cells_count, 2)
        self.assertEqual(report.checked_sample_count, 3)

    def test_xz_path_uses_handle_q_grid_at_boundary(self) -> None:
        from map_update_layer.path_collision_monitor import PathCollisionMonitor

        monitor = PathCollisionMonitor(FakeCapsule(), handle=FakeHandle())
        xz_path = np.array([[0.0, 0.0], [0.02, 0.02], [1.0, 1.0]], dtype=np.float64)
        report = monitor.check({"xz_path": xz_path}, np.zeros((4, 4), dtype=bool))

        self.assertTrue(report.is_collision)
        self.assertEqual(report.first_collision_point, (0.02, 0.02))
        self.assertEqual(report.colliding_cells_count, 1)

    def test_blockage_classifier_uses_path_collision_monitor_trigger(self) -> None:
        from map_update_layer.active_map_snapshot import ActiveMapSnapshot
        from map_update_layer.blockage_classifier import ReferenceBlockageClassifier
        from map_update_layer.path_collision_monitor import CollisionReport

        class FakeMonitor:
            def check(self, existing_path, blocked_mask):
                return CollisionReport(True, (0.02, 0.02), 2, 4)

        handle = FakeHandle()
        blocked = np.zeros(handle.shape, dtype=bool)
        final = np.ones(handle.shape, dtype=bool)
        snapshot = ActiveMapSnapshot(
            handle=handle,
            base_feasible_mask=final,
            final_active_mask=final,
            blocked_mask=blocked,
            occupied_mask=blocked,
        )
        classifier = ReferenceBlockageClassifier(path_collision_monitor=FakeMonitor())

        report = classifier.classify(
            snapshot,
            s_table=np.array([0.0, 0.02], dtype=np.float64),
            xz_table=np.array([[0.0, 0.0], [0.02, 0.02]], dtype=np.float64),
            current_s_m=0.0,
            accepted_reference={"xz_path": np.array([[0.0, 0.0], [0.02, 0.02]])},
        )

        self.assertEqual(report.event_class, ReferenceBlockageClassifier.REFERENCE_BLOCKED)
        self.assertTrue(report.replanning_triggered)
        self.assertEqual(report.reference_blocked_count, 2)
        self.assertEqual(report.reason, "path_collision_monitor_current_blocked_mask_collision")


if __name__ == "__main__":
    unittest.main()
