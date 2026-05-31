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


def make_handle(shape=(5, 5)):
    from map_handle import MapHandle

    return MapHandle(
        map_path="synthetic",
        meta_path="synthetic",
        mu_grid=np.ones(shape, dtype=np.float64),
        pitch_grid=np.zeros(shape, dtype=np.float64),
        q_grid=None,
        meta={},
        resolution_m=0.01,
        x0=0.0,
        z0=0.0,
        target_y=0.047,
        tag="synthetic",
    )


def make_snapshot(handle, blocked):
    from map_update_layer.active_map_snapshot import ActiveMapSnapshot

    base = np.ones(handle.shape, dtype=bool)
    blocked_mask = np.asarray(blocked, dtype=bool)
    return ActiveMapSnapshot(
        handle=handle,
        base_feasible_mask=base,
        final_active_mask=base & (~blocked_mask),
        blocked_mask=blocked_mask,
        occupied_mask=blocked_mask.copy(),
    )


class ReferenceSnapshotDiff2ClassTest(unittest.TestCase):
    def test_equal_snapshots_have_zero_2class_diff(self) -> None:
        from map_update_layer.perception_to_map import CameraIntrinsics
        from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff

        handle = make_handle()
        blocked = np.zeros(handle.shape, dtype=bool)
        diff = ReferenceSnapshotDiff(
            handle=handle,
            intrinsics=CameraIntrinsics(1.0, 1.0, 0.0, 0.0, 5, 5),
            y_plane=0.047,
        )
        diff.capture(make_snapshot(handle, blocked), q_act=None, T_base_cam=None)

        report = diff.evaluate(make_snapshot(handle, blocked), None)

        self.assertEqual(report.new_blocked_cells_count, 0)
        self.assertEqual(report.vanished_blocked_cells_count, 0)

    def test_current_added_cell_counts_as_new_blocked(self) -> None:
        from map_update_layer.perception_to_map import CameraIntrinsics
        from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff

        handle = make_handle()
        ref = np.zeros(handle.shape, dtype=bool)
        cur = ref.copy()
        cur[2, 2] = True
        diff = ReferenceSnapshotDiff(
            handle=handle,
            intrinsics=CameraIntrinsics(1.0, 1.0, 0.0, 0.0, 5, 5),
            y_plane=0.047,
        )
        diff.capture(make_snapshot(handle, ref), q_act=None, T_base_cam=None)

        report = diff.evaluate(make_snapshot(handle, cur), None)

        self.assertTrue(report.new_blocked_mask[2, 2])
        self.assertEqual(report.new_blocked_cells_count, 1)

    def test_reference_removed_cell_counts_as_vanished_blocked(self) -> None:
        from map_update_layer.perception_to_map import CameraIntrinsics
        from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff

        handle = make_handle()
        ref = np.zeros(handle.shape, dtype=bool)
        ref[1, 1] = True
        cur = np.zeros(handle.shape, dtype=bool)
        diff = ReferenceSnapshotDiff(
            handle=handle,
            intrinsics=CameraIntrinsics(1.0, 1.0, 0.0, 0.0, 5, 5),
            y_plane=0.047,
        )
        diff.capture(make_snapshot(handle, ref), q_act=None, T_base_cam=None)

        report = diff.evaluate(make_snapshot(handle, cur), None)

        self.assertTrue(report.vanished_blocked_mask[1, 1])
        self.assertEqual(report.vanished_blocked_cells_count, 1)

    def test_mixed_diff_reports_both_directions(self) -> None:
        from map_update_layer.perception_to_map import CameraIntrinsics
        from map_update_layer.reference_snapshot_diff import ReferenceSnapshotDiff

        handle = make_handle()
        ref = np.zeros(handle.shape, dtype=bool)
        cur = np.zeros(handle.shape, dtype=bool)
        ref[1, 1] = True
        cur[3, 3] = True
        diff = ReferenceSnapshotDiff(
            handle=handle,
            intrinsics=CameraIntrinsics(1.0, 1.0, 0.0, 0.0, 5, 5),
            y_plane=0.047,
        )
        diff.capture(make_snapshot(handle, ref), q_act=None, T_base_cam=None)

        report = diff.evaluate(make_snapshot(handle, cur), None)

        self.assertEqual(report.new_blocked_cells_count, 1)
        self.assertEqual(report.vanished_blocked_cells_count, 1)


if __name__ == "__main__":
    unittest.main()
