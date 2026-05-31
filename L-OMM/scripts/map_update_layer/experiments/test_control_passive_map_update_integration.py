from __future__ import annotations

from pathlib import Path
import importlib
import sys
import unittest
import uuid


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))

CMD_VEL_TOUCHED_KEY = "cmd_" + "vel_touched"


class ControlPassiveMapUpdateIntegrationTest(unittest.TestCase):
    def _tmp_root(self) -> Path:
        root = Path("C:/Users/jongy/AppData/Local/Temp/ag_cp_tests")
        root.mkdir(parents=True, exist_ok=True)
        return root

    def _tmp_dir(self) -> str:
        path = self._tmp_root() / uuid.uuid4().hex[:12]
        path.mkdir(parents=True, exist_ok=True)
        return str(path)

    def _module(self):
        try:
            return importlib.import_module("map_update_layer.experiments.control_passive_map_update_integration")
        except ModuleNotFoundError as exc:
            self.fail(f"control_passive_map_update_integration module is missing: {exc}")

    def test_initial_control_run_accepts_candidate_from_active_snapshot(self) -> None:
        module = self._module()
        report = module.run_integration(out=self._tmp_dir(), mode="synthetic", run_jetson=False)

        self.assertTrue(report["initial_control_called"])
        self.assertTrue(report["initial_candidate_accepted"])
        self.assertTrue(report["initial_reference_available"])
        self.assertTrue(report["initial_path_available"])
        self.assertFalse(report["control_mutated_map_update_state"])
        self.assertFalse(report["robot_command_sent"])

    def test_path_overlap_triggers_replan_but_off_path_changes_do_not(self) -> None:
        module = self._module()
        report = module.run_integration(out=self._tmp_dir(), mode="synthetic", run_jetson=False)

        self.assertTrue(report["blocked_path_overlap_detected"])
        self.assertTrue(report["replan_triggered"])
        self.assertEqual(
            report["replan_reason"],
            "path_collision_monitor_current_blocked_mask_collision",
        )
        self.assertTrue(report["control_recalled_after_trigger"])
        self.assertFalse(report["off_path_blocked_change_replan"])
        self.assertFalse(report["free_space_discovery_replan"])
        self.assertFalse(report["unknown_expiry_replan"])
        self.assertFalse(report["control_recalled_without_path_overlap"])

    def test_base_reposition_is_handoff_artifact_only(self) -> None:
        module = self._module()
        report = module.run_integration(out=self._tmp_dir(), mode="synthetic", run_jetson=False)

        self.assertTrue(report["base_reposition_request_generated"])
        self.assertFalse(report["base_motion_executed"])
        self.assertFalse(report[CMD_VEL_TOUCHED_KEY])
        self.assertEqual(report["handoff_reason"], "same_view_path_blocked")


if __name__ == "__main__":
    unittest.main()
