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


class FakeHandle:
    resolution_m = 1.0
    x0 = 0.0
    z0 = 0.0

    @property
    def shape(self):
        return (5, 5)


def make_snapshot():
    from map_update_layer.active_map_snapshot import ActiveMapSnapshot

    handle = FakeHandle()
    blocked = np.zeros(handle.shape, dtype=bool)
    final = np.ones(handle.shape, dtype=bool)
    return ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=final,
        final_active_mask=final,
        blocked_mask=blocked,
        occupied_mask=blocked.copy(),
    )


def reference_tables():
    s = np.array([0.0, 1.0, 2.0, 3.0], dtype=np.float64)
    xz = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]], dtype=np.float64)
    q = np.column_stack([s, np.zeros_like(s), np.zeros_like(s)])
    return s, xz, q


class RecordingMonitor:
    def __init__(self, colliding_x_values: set[float]) -> None:
        self.colliding_x_values = {float(v) for v in colliding_x_values}
        self.received_xz: np.ndarray | None = None
        self.received_q: np.ndarray | None = None

    def check(self, existing_path, blocked_mask):
        from map_update_layer.path_collision_monitor import CollisionReport

        self.received_xz = np.asarray(existing_path.get("xz_path"), dtype=np.float64).reshape(-1, 2)
        q = existing_path.get("q_traj", existing_path.get("q_samples"))
        self.received_q = None if q is None else np.asarray(q, dtype=np.float64).reshape(-1, 3)
        hit_indices = [
            idx for idx, pt in enumerate(self.received_xz)
            if float(pt[0]) in self.colliding_x_values
        ]
        if not hit_indices:
            return CollisionReport(False, None, 0, int(self.received_xz.shape[0]))
        first = self.received_xz[hit_indices[0]]
        return CollisionReport(True, (float(first[0]), float(first[1])), len(hit_indices), int(self.received_xz.shape[0]))


class FutureOnlyPathCollisionTriggerTest(unittest.TestCase):
    def classify_with_monitor(self, monitor: RecordingMonitor, *, current_s_m: float = 2.0, q_override=None):
        from map_update_layer.blockage_classifier import ReferenceBlockageClassifier

        s, xz, q = reference_tables()
        accepted_reference = {"xz_path": xz, "q_traj": q if q_override is None else q_override}
        classifier = ReferenceBlockageClassifier(
            path_collision_monitor=monitor,
            corridor_radius_m=0.0,
            current_pose_radius_m=0.0,
            lookahead_m=2.0,
        )
        return classifier.classify(
            make_snapshot(),
            s_table=s,
            xz_table=xz,
            current_s_m=current_s_m,
            accepted_reference=accepted_reference,
        )

    def test_past_obstacle_does_not_trigger_and_monitor_receives_future_only_reference(self) -> None:
        monitor = RecordingMonitor({1.0})

        report = self.classify_with_monitor(monitor)

        self.assertIn(report.event_class, {"NO_RELEVANT_CHANGE", "MASK_CHANGED_NONCRITICAL"})
        self.assertFalse(report.replanning_triggered)
        self.assertIsNotNone(monitor.received_xz)
        self.assertGreaterEqual(float(monitor.received_xz[0, 0]), 2.0)
        self.assertIsNotNone(monitor.received_q)
        self.assertGreaterEqual(float(monitor.received_q[0, 0]), 2.0)
        self.assertFalse(report.as_dict().get("reference_future_slice_fallback", True))

    def test_future_obstacle_triggers_reference_blocked(self) -> None:
        monitor = RecordingMonitor({3.0})

        report = self.classify_with_monitor(monitor)

        self.assertEqual(report.event_class, "REFERENCE_BLOCKED")
        self.assertTrue(report.replanning_triggered)
        self.assertEqual(report.reason, "path_collision_monitor_current_blocked_mask_collision")
        self.assertGreaterEqual(float(monitor.received_xz[0, 0]), 2.0)

    def test_straddling_obstacle_only_future_portion_triggers(self) -> None:
        monitor = RecordingMonitor({1.0, 2.0})

        report = self.classify_with_monitor(monitor)

        self.assertEqual(report.event_class, "REFERENCE_BLOCKED")
        self.assertEqual(report.reference_blocked_count, 1)
        self.assertTrue(np.all(monitor.received_xz[:, 0] >= 2.0))

    def test_shape_mismatch_falls_back_to_full_reference_and_reports_it(self) -> None:
        monitor = RecordingMonitor(set())
        mismatched_q = np.zeros((3, 3), dtype=np.float64)

        report = self.classify_with_monitor(monitor, q_override=mismatched_q)

        self.assertEqual(report.event_class, "NO_RELEVANT_CHANGE")
        self.assertEqual(monitor.received_xz.shape[0], 4)
        self.assertTrue(report.as_dict().get("reference_future_slice_fallback", False))


if __name__ == "__main__":
    unittest.main()
