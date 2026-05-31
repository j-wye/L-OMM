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


def make_snapshot(*, blocked_cells=()):
    from map_update_layer.active_map_snapshot import ActiveMapSnapshot

    handle = FakeHandle()
    blocked = np.zeros(handle.shape, dtype=bool)
    for ix, iz in blocked_cells:
        blocked[int(ix), int(iz)] = True
    final = np.ones(handle.shape, dtype=bool) & (~blocked)
    return ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=np.ones(handle.shape, dtype=bool),
        final_active_mask=final,
        blocked_mask=blocked,
        occupied_mask=blocked.copy(),
    )


def make_diff_report(*, violated: bool):
    from map_update_layer.reference_snapshot_diff import DiffReport

    mask = np.zeros((5, 5), dtype=bool)
    if violated:
        mask[2, 0] = True
    return DiffReport(
        new_blocked_mask=mask,
        vanished_blocked_mask=np.zeros_like(mask),
        new_obstacle_mask=mask.copy(),
        vanished_obstacle_mask=np.zeros_like(mask),
        fov_mask=np.ones_like(mask),
        new_blocked_cells_count=int(np.count_nonzero(mask)),
        vanished_blocked_cells_count=0,
        new_obstacle_cells=int(np.count_nonzero(mask)),
        vanished_obstacle_cells=0,
        fov_observed_cells=int(mask.size),
        fov_coverage_ratio=1.0,
        largest_new_cluster_size=int(np.count_nonzero(mask)),
        largest_reference_corridor_cluster_size=int(np.count_nonzero(mask)),
        new_cells_in_reference_corridor=int(np.count_nonzero(mask)),
        new_cells_in_current_capsule=0,
        new_cells_in_goal_window=0,
        is_corridor_violated=bool(violated),
        tau_diff_cells=1,
        tau_cluster_cells=1,
    )


class FixedMonitor:
    def __init__(self, collision: bool) -> None:
        self.collision = bool(collision)

    def check(self, existing_path, blocked_mask):
        from map_update_layer.path_collision_monitor import CollisionReport

        return CollisionReport(self.collision, (2.0, 0.0) if self.collision else None, 3 if self.collision else 0, 4)


def classify(*, monitor=None, snapshot=None, diff=None):
    from map_update_layer.blockage_classifier import ReferenceBlockageClassifier

    s = np.array([0.0, 1.0, 2.0, 3.0], dtype=np.float64)
    xz = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0], [3.0, 0.0]], dtype=np.float64)
    classifier = ReferenceBlockageClassifier(
        path_collision_monitor=monitor,
        corridor_radius_m=0.0,
        current_pose_radius_m=0.0,
        lookahead_m=2.0,
    )
    return classifier.classify(
        snapshot or make_snapshot(),
        s_table=s,
        xz_table=xz,
        current_s_m=1.0,
        accepted_reference={"xz_path": xz},
        reference_snapshot_diff=diff,
    )


class ReferenceBlockedSourceProvenanceTest(unittest.TestCase):
    def test_monitor_collision_is_authoritative_source(self) -> None:
        report = classify(monitor=FixedMonitor(True))

        self.assertEqual(report.event_class, "REFERENCE_BLOCKED")
        self.assertEqual(report.as_dict().get("reference_blocked_source"), "path_collision_monitor")
        self.assertEqual(report.reference_blocked_count, 3)

    def test_reference_snapshot_diff_source_when_monitor_is_clear(self) -> None:
        report = classify(monitor=FixedMonitor(False), diff=make_diff_report(violated=True))

        self.assertEqual(report.event_class, "REFERENCE_BLOCKED")
        self.assertEqual(report.as_dict().get("reference_blocked_source"), "reference_snapshot_diff")

    def test_corridor_radius_fallback_source_when_monitor_absent(self) -> None:
        report = classify(monitor=None, snapshot=make_snapshot(blocked_cells=[(2, 0)]))

        self.assertEqual(report.event_class, "REFERENCE_BLOCKED")
        self.assertEqual(report.as_dict().get("reference_blocked_source"), "corridor_radius_fallback")

    def test_no_source_when_monitor_clear_and_no_diff_or_corridor_hit(self) -> None:
        report = classify(monitor=FixedMonitor(False))

        self.assertNotEqual(report.event_class, "REFERENCE_BLOCKED")
        self.assertEqual(report.as_dict().get("reference_blocked_source"), "none")


if __name__ == "__main__":
    unittest.main()
