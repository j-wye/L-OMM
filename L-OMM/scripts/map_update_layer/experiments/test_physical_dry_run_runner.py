from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


SCRIPTS_ROOT = Path(__file__).resolve().parents[2]
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))
CONTROL_MODULE_ROOT = SCRIPTS_ROOT / "control_module"
if str(CONTROL_MODULE_ROOT) not in sys.path:
    sys.path.insert(0, str(CONTROL_MODULE_ROOT))


from robot_execution.execution_supervisor import ExecutionCandidate  # noqa: E402
from robot_execution.fake_controller_feedback import (  # noqa: E402
    F_ABORT,
    F_BASE_COMMAND,
    F_FEEDBACK_TIMEOUT,
    F_FINGER_CLOSE,
    F_NONFINITE,
    F_OPERATOR_ABORT,
    F_REJECT,
    F_TRACKING_ERROR,
    ScriptedFakeController,
    ScriptedFaults,
)
from robot_execution.joint_mapping import ReducedJointMapper  # noqa: E402
from robot_execution.physical_dry_run_runner import (  # noqa: E402
    CHECK_ONLY_READY_NO_SEND,
    FAIL_CONTROLLER_REJECTED,
    FAIL_FEEDBACK_TIMEOUT,
    FAIL_OPERATOR_ABORT,
    FAIL_PRECHECK,
    FAIL_SAFETY_GATE,
    FAIL_TRACKING_ERROR,
    NOT_APPLICABLE_UNTIL_SEND,
    NOT_READY_POSTURE_FAIL,
    PASS_CONTROLLER_CONTRACT,
    SECOND_SEND_REFUSED,
    PhysicalDryRunAuthorization,
    PhysicalDryRunPlan,
    PhysicalDryRunRunner,
)
from robot_execution.pre_physical_preflight import PrePhysicalTrajectoryPreflight  # noqa: E402
from robot_execution.safety_gate import ExecutionSafetyConfig, ExecutionSafetyGate  # noqa: E402
from robot_execution.trajectory_builder import ReducedTrajectoryBuilder  # noqa: E402


SENTINEL = "TEST_SENTINEL_FAKE_TOKEN"  # never the production token


def _make_plan(n: int = 6):
    mapper = ReducedJointMapper()
    builder = ReducedTrajectoryBuilder(mapper)
    contract = mapper.contract
    q = np.stack([np.array([0.10, 0.20, 0.30]) + i * 0.002 for i in range(n)], axis=0)
    command = builder.from_active_trajectory(q, dt_s=0.1)
    xz = np.stack([[i * 0.005, 0.5] for i in range(n)], axis=0)  # short path < 0.1 m
    seg = np.linalg.norm(np.diff(xz, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    cand = ExecutionCandidate(
        q_active_trajectory=q, s_table=s, xz_table=xz, dt_s=0.1,
        candidate_metrics={"candidate_accepted": True},
    )
    record = PrePhysicalTrajectoryPreflight().evaluate(command, cand, contract=contract)
    plan = PhysicalDryRunPlan(
        candidate=cand, command=command, preflight_record=record, controller_contract=contract,
        planned_start_joint_names=tuple(command.joint_names),
        planned_start_positions=command.points[0].positions.copy(),
        candidate_id="unit_test", control_artifact_dir="",
    )
    return plan, contract


def _gate():
    return ExecutionSafetyGate(ExecutionSafetyConfig(mode="fake_action", required_armed_token=SENTINEL))


def _authorized():
    return PhysicalDryRunAuthorization(armed_token=SENTINEL, user_approved_single_send=True, allow_action_client=True)


def _closed():
    return PhysicalDryRunAuthorization()


class PhysicalDryRunRunnerTest(unittest.TestCase):
    def _run(self, *, authorization, controller, gate, plan):
        runner = PhysicalDryRunRunner()
        out = tempfile.mkdtemp(prefix="ag_dryrun_unit_")
        return runner.run(plan, authorization=authorization, controller=controller, safety_gate=gate, out_dir=out), runner, out

    def _controller(self, gate, plan, *, faults=None, start_names=None, start_positions=None):
        return ScriptedFakeController(
            safety_gate=gate, command=plan.command, faults=faults,
            start_joint_names=start_names, start_positions=start_positions,
        )

    # -- check-only default ----------------------------------------------------

    def test_check_only_default_does_not_send(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()
        ctrl = self._controller(gate, plan)
        res, _, out = self._run(authorization=_closed(), controller=ctrl, gate=gate, plan=plan)
        self.assertEqual(res.runner_readiness_verdict, CHECK_ONLY_READY_NO_SEND)
        self.assertEqual(res.contract_verdict, NOT_APPLICABLE_UNTIL_SEND)
        self.assertFalse(res.goal_sent)
        self.assertEqual(ctrl.fake_goal_count, 0)
        self.assertFalse(res.real_action_client_created)
        self.assertEqual(res.feedback_source, "none")
        self.assertFalse(res.physical)
        # header-only feedback CSV (no send, no feedback)
        lines = (Path(res.log_dir) / "feedback_joint_trajectory.csv").read_text(encoding="utf-8").splitlines()
        self.assertEqual(len(lines), 1)

    # -- authorized happy path (fake) -----------------------------------------

    def test_authorized_happy_path_is_fake_pass_not_physical(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()
        ctrl = self._controller(gate, plan, faults=ScriptedFaults("none"))
        res, _, _ = self._run(authorization=_authorized(), controller=ctrl, gate=gate, plan=plan)
        self.assertEqual(res.contract_verdict, PASS_CONTROLLER_CONTRACT)
        self.assertEqual(res.feedback_source, "fake_controller_simulated")
        self.assertFalse(res.physical)  # never a physical claim
        self.assertTrue(res.goal_sent)
        self.assertTrue(res.controller_succeeded)
        self.assertEqual(ctrl.fake_goal_count, 1)
        self.assertFalse(res.real_action_client_created)
        # all five CSVs present, feedback populated
        for name in ("commanded_joint_trajectory.csv", "feedback_joint_trajectory.csv",
                     "tracking_error.csv", "controller_events.csv", "safety_events.csv"):
            self.assertTrue((Path(res.log_dir) / name).exists())
        fb_lines = (Path(res.log_dir) / "feedback_joint_trajectory.csv").read_text(encoding="utf-8").splitlines()
        self.assertGreater(len(fb_lines), 1)

    # -- authorization separation (Codex 3.6) ---------------------------------

    def test_fake_gate_alone_is_not_authorization(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()  # fake gate would accept a fake send
        # all-but-one authorization flags: still NOT authorized -> check-only, no send.
        partials = [
            PhysicalDryRunAuthorization(armed_token=SENTINEL, user_approved_single_send=False, allow_action_client=True),
            PhysicalDryRunAuthorization(armed_token=SENTINEL, user_approved_single_send=True, allow_action_client=False),
            PhysicalDryRunAuthorization(armed_token="", user_approved_single_send=True, allow_action_client=True),
            PhysicalDryRunAuthorization(armed_token="WRONG", user_approved_single_send=True, allow_action_client=True),
        ]
        for auth in partials:
            ctrl = self._controller(gate, plan)
            res, _, _ = self._run(authorization=auth, controller=ctrl, gate=gate, plan=plan)
            self.assertEqual(res.runner_readiness_verdict, CHECK_ONLY_READY_NO_SEND)
            self.assertFalse(res.goal_sent)
            self.assertEqual(ctrl.fake_goal_count, 0)

    # -- name-based start posture (Codex 3.5) ----------------------------------

    def test_reordered_joint_names_same_posture_passes(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()
        names = list(plan.command.joint_names)
        pos = plan.planned_start_positions.copy()
        order = [6, 0, 5, 1, 4, 2, 3]  # arbitrary permutation
        reordered_names = [names[i] for i in order]
        reordered_pos = pos[order]
        ctrl = self._controller(gate, plan, start_names=reordered_names, start_positions=reordered_pos)
        res, _, _ = self._run(authorization=_authorized(), controller=ctrl, gate=gate, plan=plan)
        # posture passed despite reordering -> it proceeded to send and succeeded
        self.assertNotEqual(res.runner_readiness_verdict, NOT_READY_POSTURE_FAIL)
        self.assertTrue(res.goal_sent)

    def test_start_posture_mismatch_blocks_send(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()
        bad = plan.planned_start_positions.copy()
        bad[1] += 0.5  # arm_joint_2 off by 0.5 rad
        ctrl = self._controller(gate, plan, start_positions=bad)
        res, _, _ = self._run(authorization=_authorized(), controller=ctrl, gate=gate, plan=plan)
        self.assertEqual(res.runner_readiness_verdict, NOT_READY_POSTURE_FAIL)
        self.assertEqual(res.contract_verdict, FAIL_PRECHECK)
        self.assertFalse(res.goal_sent)
        self.assertEqual(ctrl.fake_goal_count, 0)

    # -- abort classes A-D -----------------------------------------------------

    def _run_fault(self, kind):
        plan, _ = _make_plan()
        gate = _gate()
        ctrl = self._controller(gate, plan, faults=ScriptedFaults(kind, at_index=1))
        res, _, _ = self._run(authorization=_authorized(), controller=ctrl, gate=gate, plan=plan)
        return res, ctrl

    def test_tracking_error_cancels_class_c(self) -> None:
        res, ctrl = self._run_fault(F_TRACKING_ERROR)
        self.assertEqual(res.contract_verdict, FAIL_TRACKING_ERROR)
        self.assertEqual(res.abort_class, "C")
        self.assertTrue(res.cancel_sent)
        self.assertGreater(res.max_tracking_error_rad, 0.05)

    def test_feedback_timeout_class_b(self) -> None:
        res, _ = self._run_fault(F_FEEDBACK_TIMEOUT)
        self.assertEqual(res.contract_verdict, FAIL_FEEDBACK_TIMEOUT)
        self.assertEqual(res.abort_class, "B")
        self.assertTrue(res.cancel_sent)

    def test_controller_abort_class_b(self) -> None:
        res, _ = self._run_fault(F_ABORT)
        self.assertEqual(res.contract_verdict, FAIL_CONTROLLER_REJECTED)
        self.assertEqual(res.abort_class, "B")

    def test_controller_reject_at_send_class_b(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()
        ctrl = self._controller(gate, plan, faults=ScriptedFaults(F_REJECT))
        res, _, _ = self._run(authorization=_authorized(), controller=ctrl, gate=gate, plan=plan)
        self.assertEqual(res.contract_verdict, FAIL_CONTROLLER_REJECTED)
        self.assertEqual(res.abort_class, "B")
        self.assertTrue(res.goal_sent)
        self.assertFalse(res.goal_accepted)

    def test_nonfinite_class_c(self) -> None:
        res, _ = self._run_fault(F_NONFINITE)
        self.assertEqual(res.contract_verdict, FAIL_TRACKING_ERROR)
        self.assertEqual(res.abort_class, "C")

    def test_finger_close_class_d(self) -> None:
        res, _ = self._run_fault(F_FINGER_CLOSE)
        self.assertEqual(res.contract_verdict, FAIL_SAFETY_GATE)
        self.assertEqual(res.abort_class, "D")

    def test_base_command_class_d(self) -> None:
        res, _ = self._run_fault(F_BASE_COMMAND)
        self.assertEqual(res.contract_verdict, FAIL_SAFETY_GATE)
        self.assertEqual(res.abort_class, "D")

    def test_operator_abort_class_a(self) -> None:
        res, _ = self._run_fault(F_OPERATOR_ABORT)
        self.assertEqual(res.contract_verdict, FAIL_OPERATOR_ABORT)
        self.assertEqual(res.abort_class, "A")

    # -- one-send rule ---------------------------------------------------------

    def test_one_send_rule_refuses_second_send(self) -> None:
        plan, _ = _make_plan()
        gate = _gate()
        ctrl = self._controller(gate, plan, faults=ScriptedFaults("none"))
        runner = PhysicalDryRunRunner()
        out1 = tempfile.mkdtemp(prefix="ag_dryrun_send1_")
        out2 = tempfile.mkdtemp(prefix="ag_dryrun_send2_")
        first = runner.run(plan, authorization=_authorized(), controller=ctrl, safety_gate=gate, out_dir=out1)
        second = runner.run(plan, authorization=_authorized(), controller=ctrl, safety_gate=gate, out_dir=out2)
        self.assertTrue(first.goal_sent)
        self.assertEqual(second.runner_readiness_verdict, SECOND_SEND_REFUSED)
        self.assertEqual(ctrl.fake_goal_count, 1)  # exactly one send

    # -- no-command guarantees -------------------------------------------------

    def test_no_real_action_client_in_any_path(self) -> None:
        plan, _ = _make_plan()
        for auth in (_closed(), _authorized()):
            gate = _gate()
            ctrl = self._controller(gate, plan, faults=ScriptedFaults("none"))
            res, _, _ = self._run(authorization=auth, controller=ctrl, gate=gate, plan=plan)
            self.assertFalse(res.real_action_client_created)

    def test_production_token_literal_absent_from_new_files_only(self) -> None:
        # assembled from fragments so this scanner file itself never holds the contiguous literal
        token = "I_UNDERSTAND" + "_MOVING_ARM_RISK"
        new_files = [
            SCRIPTS_ROOT / "robot_execution" / "physical_dry_run_runner.py",
            SCRIPTS_ROOT / "robot_execution" / "fake_controller_feedback.py",
            SCRIPTS_ROOT / "map_update_layer" / "experiments" / "physical_dry_run_runner_driver.py",
            SCRIPTS_ROOT / "map_update_layer" / "experiments" / "test_physical_dry_run_runner.py",
        ]
        for path in new_files:
            self.assertNotIn(token, path.read_text(encoding="utf-8"), f"production token literal leaked into {path.name}")


class PhysicalDryRunDriverRealMapTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from map_update_layer.experiments.physical_dry_run_runner_driver import run_physical_dry_run_check_only

        cls.out = tempfile.mkdtemp(prefix="ag_dryrun_driver_")
        cls.report = run_physical_dry_run_check_only(out=cls.out)
        cls.inv = cls.report["invariants"]

    def test_status_complete_and_all_invariants(self) -> None:
        self.assertEqual(self.report["status"], "GOAL_STATUS: COMPLETE")
        self.assertTrue(self.inv["all_pass"], f"invariants: {self.inv}")

    def test_check_only_no_send_no_real_client(self) -> None:
        self.assertTrue(self.inv["readiness_is_check_only"])
        self.assertTrue(self.inv["contract_not_applicable_until_send"])
        self.assertTrue(self.inv["no_send"])
        self.assertTrue(self.inv["no_real_action_client"])

    def test_feedback_header_only_and_all_csvs(self) -> None:
        self.assertTrue(self.inv["feedback_csv_header_only"])
        self.assertTrue(self.inv["all_csvs_present"])


if __name__ == "__main__":
    unittest.main()
