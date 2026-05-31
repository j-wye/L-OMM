from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
EXPERIMENTS_ROOT = SCRIPTS_ROOT / "map_update_layer" / "experiments"
if str(EXPERIMENTS_ROOT) not in sys.path:
    sys.path.insert(0, str(EXPERIMENTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

from map_handle import MapHandle  # noqa: E402


class InvalidDepthUnknownAlternativeProjectionHardeningTest(unittest.TestCase):
    def _handle(self) -> MapHandle:
        shape = (12, 12)
        q_grid = np.zeros(shape + (3,), dtype=np.float64)
        return MapHandle(
            map_path="",
            meta_path="",
            mu_grid=np.ones(shape, dtype=np.float64),
            pitch_grid=np.zeros(shape, dtype=np.float64),
            q_grid=q_grid,
            meta={},
            resolution_m=0.01,
            x0=0.0,
            z0=0.0,
            target_y=0.047,
            tag="test",
        )

    def test_rect_specific_substitution_preserves_unrelated_unknown_cells(self) -> None:
        from invalid_depth_unknown_alternative_projection_hardening import substitute_unknown_layer_hardened

        blocked = np.zeros((12, 12), dtype=bool)
        unknown = np.zeros_like(blocked)
        blocked[1:4, 1:4] = True
        unknown[1:4, 1:4] = True
        blocked[8, 8] = True
        unknown[8, 8] = True
        production_shell_cells = {(ix, iz) for ix in range(1, 4) for iz in range(1, 4)}
        prototype_cells = {(2, 2)}

        full = substitute_unknown_layer_hardened(
            original_blocked=blocked,
            original_unknown=unknown,
            prototype_rect_cells=prototype_cells,
            production_shell_rect_cells=production_shell_cells,
            shape=blocked.shape,
            substitution_scope="full_unknown_layer",
        )
        rect = substitute_unknown_layer_hardened(
            original_blocked=blocked,
            original_unknown=unknown,
            prototype_rect_cells=prototype_cells,
            production_shell_rect_cells=production_shell_cells,
            shape=blocked.shape,
            substitution_scope="rect_specific",
        )

        self.assertFalse(full.blocked[8, 8])
        self.assertTrue(rect.blocked[8, 8])
        self.assertGreater(full.removed_unknown_cells, rect.removed_unknown_cells)
        self.assertEqual(rect.substitution_scope, "rect_specific")

    def test_reason_code_prefers_path_monitor_unavailable_over_radius_fallback(self) -> None:
        from invalid_depth_unknown_alternative_projection_hardening import _hardening_reason_code

        reason = _hardening_reason_code(
            event_class="REFERENCE_BLOCKED",
            path_collision_monitor_requested=True,
            path_collision_monitor_available=False,
            path_collision_is_collision=False,
            reference_blocked_source="corridor_radius_fallback",
            reference_future_slice_fallback=False,
            goal_feasible_neighborhood_count=3,
            goal_connectivity_ok=True,
            substitution_removed_non_unknown_cells=0,
        )

        self.assertEqual(reason, "path_collision_monitor_unavailable")

    def test_reason_code_prefers_true_capsule_collision_when_available(self) -> None:
        from invalid_depth_unknown_alternative_projection_hardening import _hardening_reason_code

        reason = _hardening_reason_code(
            event_class="REFERENCE_BLOCKED",
            path_collision_monitor_requested=True,
            path_collision_monitor_available=True,
            path_collision_is_collision=True,
            reference_blocked_source="path_collision_monitor",
            reference_future_slice_fallback=False,
            goal_feasible_neighborhood_count=3,
            goal_connectivity_ok=True,
            substitution_removed_non_unknown_cells=0,
        )

        self.assertEqual(reason, "true_capsule_future_reference_collision")

    def test_resamples_episode_q_traj_to_reference_support(self) -> None:
        from invalid_depth_unknown_alternative_projection_hardening import _resample_q_traj_to_reference

        q = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 2.0, 3.0],
                [2.0, 4.0, 6.0],
                [3.0, 6.0, 9.0],
                [4.0, 8.0, 12.0],
            ],
            dtype=np.float64,
        )

        aligned = _resample_q_traj_to_reference(q, reference_len=3)

        self.assertEqual(aligned.shape, (3, 3))
        np.testing.assert_allclose(aligned[0], q[0])
        np.testing.assert_allclose(aligned[1], q[2])
        np.testing.assert_allclose(aligned[2], q[4])

    def test_episode_resampled_mode_emits_aligned_q_without_slice_fallback(self) -> None:
        from invalid_depth_unknown_alternative_projection_hardening import build_accepted_reference_for_mode

        accepted = {
            "s": np.array([0.0, 0.5, 1.0], dtype=np.float64),
            "xz": np.zeros((3, 2), dtype=np.float64),
        }
        episode_q = np.arange(15, dtype=np.float64).reshape(5, 3)

        result = build_accepted_reference_for_mode(
            accepted_reference=accepted,
            accepted_reference_mode="episode_resampled",
            episode_q_traj=episode_q,
            handle=self._handle(),
        )

        self.assertEqual(result.mode, "episode_resampled")
        self.assertEqual(result.q_alignment_status, "resampled_to_reference_support")
        self.assertEqual(result.q_samples_before, 5)
        self.assertEqual(result.q_samples_after, 3)
        self.assertIn("q_traj", result.accepted_reference)
        self.assertFalse(result.expected_future_slice_fallback)

    def test_stored_npz_mode_reports_length_mismatch_fallback(self) -> None:
        from invalid_depth_unknown_alternative_projection_hardening import build_accepted_reference_for_mode

        accepted = {
            "s": np.array([0.0, 0.5, 1.0], dtype=np.float64),
            "xz": np.zeros((3, 2), dtype=np.float64),
            "q_traj": np.zeros((5, 3), dtype=np.float64),
        }

        result = build_accepted_reference_for_mode(
            accepted_reference=accepted,
            accepted_reference_mode="stored_npz",
            episode_q_traj=None,
            handle=self._handle(),
        )

        self.assertEqual(result.q_alignment_status, "stored_q_length_mismatch")
        self.assertTrue(result.expected_future_slice_fallback)


if __name__ == "__main__":
    unittest.main()
