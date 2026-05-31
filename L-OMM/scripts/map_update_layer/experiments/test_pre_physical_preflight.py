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
from robot_execution.joint_mapping import ReducedJointMapper  # noqa: E402
from robot_execution.pre_physical_preflight import (  # noqa: E402
    PrePhysicalPreflightConfig,
    PrePhysicalTrajectoryPreflight,
    default_no_token_gate,
)
from robot_execution.safety_gate import (  # noqa: E402
    ExecutionSafetyConfig,
    ExecutionSafetyGate,
    SafetyViolation,
)
from robot_execution.trajectory_builder import (  # noqa: E402
    JointTrajectoryCommand,
    ReducedTrajectoryBuilder,
    TrajectoryPoint,
)
from robot_execution.ros2_trajectory_client import SafeTrajectoryClient  # noqa: E402

try:
    from control_module.constants import Q1_FIXED, Q4_FIXED, Q6_FIXED
except Exception:  # pragma: no cover
    from constants import Q1_FIXED, Q4_FIXED, Q6_FIXED  # type: ignore


ACTION = "/arm_controller/follow_joint_trajectory"


def _full(q2: float, q3: float, q5: float, finger: float = 0.0):
    # contract order: arm_joint_1..6, right_finger_bottom_joint
    return [float(Q1_FIXED), float(q2), float(q3), float(Q4_FIXED), float(q5), float(Q6_FIXED), float(finger)]


def _manual_command(joint_names, positions, times):
    pts = tuple(
        TrajectoryPoint(np.asarray(p, dtype=np.float64), float(t)) for p, t in zip(positions, times)
    )
    return JointTrajectoryCommand(joint_names=tuple(joint_names), points=pts, action_name=ACTION)


def _candidate(q, xz, *, dt=0.1):
    q = np.asarray(q, dtype=np.float64).reshape(-1, 3)
    xz = np.asarray(xz, dtype=np.float64).reshape(-1, 2)
    seg = np.linalg.norm(np.diff(xz, axis=0), axis=1) if xz.shape[0] > 1 else np.zeros(0)
    s = np.concatenate([[0.0], np.cumsum(seg)]) if xz.shape[0] > 1 else np.zeros(1)
    return ExecutionCandidate(
        q_active_trajectory=q,
        s_table=s,
        xz_table=xz,
        dt_s=dt,
        candidate_metrics={"candidate_accepted": True},
    )


class PrePhysicalPreflightUnitTest(unittest.TestCase):
    def setUp(self) -> None:
        self.mapper = ReducedJointMapper()
        self.builder = ReducedTrajectoryBuilder(self.mapper)
        self.contract = self.mapper.contract
        self.preflight = PrePhysicalTrajectoryPreflight()

    def _valid_q(self, n=5):
        base = np.array([0.10, 0.20, 0.30], dtype=np.float64)
        return np.stack([base + i * 0.002 for i in range(n)], axis=0)

    def _valid_xz(self, n=5, step=0.005):
        # short straight EE path (n-1)*step < 0.1 m
        return np.stack([[i * step, 0.5] for i in range(n)], axis=0)

    def test_valid_short_segment_passes_all_trajectory_checks(self) -> None:
        q = self._valid_q()
        cmd = self.builder.from_active_trajectory(q, dt_s=0.1)
        cand = _candidate(q, self._valid_xz())
        rec = self.preflight.evaluate(cmd, cand, contract=self.contract)
        self.assertTrue(rec.passed, f"unexpected failures: {rec.failures}")
        self.assertEqual(rec.verdict_eligibility, "PASS_CONTROLLER_CONTRACT_TRAJECTORY_PREFLIGHT")
        self.assertEqual(rec.finger_start, 0.0)
        self.assertEqual(rec.finger_end, 0.0)
        self.assertFalse(rec.uses_control_internal_dt_as_execution_dt)
        self.assertAlmostEqual(rec.execution_dt_s, 0.1)
        self.assertLessEqual(rec.estimated_ee_displacement_m, 0.1 + 1e-9)

    def test_control_internal_dt_is_rejected(self) -> None:
        q = self._valid_q()
        cmd = self.builder.from_active_trajectory(q, dt_s=0.01)  # control internal step
        cand = _candidate(q, self._valid_xz())
        rec = self.preflight.evaluate(cmd, cand, contract=self.contract)
        self.assertFalse(rec.checks["execution_dt_at_least_min"])
        self.assertFalse(rec.checks["not_control_internal_dt"])
        self.assertTrue(rec.uses_control_internal_dt_as_execution_dt)
        self.assertEqual(rec.verdict_eligibility, "FAIL_PRECHECK")

    def test_finger_closing_violates_object_free(self) -> None:
        names = self.contract.joint_names
        positions = [_full(0.10, 0.20, 0.30, finger=0.0), _full(0.101, 0.20, 0.30, finger=0.3)]
        cmd = _manual_command(names, positions, [0.0, 0.1])
        cand = _candidate([[0.10, 0.20, 0.30], [0.101, 0.20, 0.30]], [[0.0, 0.5], [0.004, 0.5]])
        rec = self.preflight.evaluate(cmd, cand, contract=self.contract)
        self.assertFalse(rec.checks["finger_hold_open_object_free"])
        self.assertEqual(rec.finger_start, 0.0)
        self.assertEqual(rec.finger_end, 0.3)
        self.assertEqual(rec.verdict_eligibility, "FAIL_PRECHECK")

    def test_over_velocity_is_rejected(self) -> None:
        names = self.contract.joint_names
        # q5 jumps 0.08 rad at dt=0.1 -> vel 0.8 > 0.5 cap, step 0.08 < 0.1 (isolates velocity)
        positions = [_full(0.10, 0.20, 0.30), _full(0.10, 0.20, 0.38)]
        cmd = _manual_command(names, positions, [0.0, 0.1])
        cand = _candidate([[0.10, 0.20, 0.30], [0.10, 0.20, 0.38]], [[0.0, 0.5], [0.004, 0.5]])
        rec = self.preflight.evaluate(cmd, cand, contract=self.contract)
        self.assertFalse(rec.checks["velocity_within_cap"])
        self.assertTrue(rec.checks["tracking_step_within_cap"])
        self.assertEqual(rec.verdict_eligibility, "FAIL_PRECHECK")

    def test_wrong_joint_names_or_order_is_rejected(self) -> None:
        wrong = ("arm_joint_2", "arm_joint_1", "arm_joint_3", "arm_joint_4",
                 "arm_joint_5", "arm_joint_6", "right_finger_bottom_joint")  # 1 and 2 swapped
        positions = [_full(0.10, 0.20, 0.30), _full(0.101, 0.20, 0.30)]
        cmd = _manual_command(wrong, positions, [0.0, 0.1])
        cand = _candidate([[0.10, 0.20, 0.30], [0.101, 0.20, 0.30]], [[0.0, 0.5], [0.004, 0.5]])
        rec = self.preflight.evaluate(cmd, cand, contract=self.contract)
        self.assertFalse(rec.checks["joint_names_match_contract"])
        self.assertEqual(rec.verdict_eligibility, "FAIL_PRECHECK")

    def test_ee_displacement_over_cap_is_rejected(self) -> None:
        q = self._valid_q(2)
        cmd = self.builder.from_active_trajectory(q, dt_s=0.1)
        cand = _candidate(q, [[0.0, 0.5], [0.2, 0.5]])  # 0.2 m EE path > 0.1 cap
        rec = self.preflight.evaluate(cmd, cand, contract=self.contract)
        self.assertFalse(rec.checks["ee_displacement_within_cap"])
        self.assertGreater(rec.estimated_ee_displacement_m, 0.1)
        self.assertEqual(rec.verdict_eligibility, "FAIL_PRECHECK")

    def test_conforming_leading_segment_truncates_to_cap(self) -> None:
        n = 31
        q = np.tile(np.array([0.10, 0.20, 0.30]), (n, 1))
        xz = np.stack([[i * 0.01, 0.5] for i in range(n)], axis=0)  # total path 0.30 m
        cand = _candidate(q, xz)
        res = self.preflight.conforming_leading_segment(cand, max_ee_displacement_m=0.1)
        self.assertIsNotNone(res.candidate)
        self.assertGreaterEqual(res.n_points, 2)
        self.assertLessEqual(res.path_length_m, 0.1 + 1e-9)
        self.assertEqual(res.reason, "ok")
        # the segment must itself pass the trajectory preflight
        seg_cmd = self.builder.from_active_trajectory(res.candidate.q_active_trajectory, dt_s=0.1)
        seg_rec = self.preflight.evaluate(seg_cmd, res.candidate, contract=self.contract)
        self.assertTrue(seg_rec.passed, f"segment failures: {seg_rec.failures}")

    def test_conforming_segment_rejects_when_first_step_exceeds_cap(self) -> None:
        q = np.tile(np.array([0.10, 0.20, 0.30]), (2, 1))
        xz = np.array([[0.0, 0.5], [0.5, 0.5]], dtype=np.float64)  # single 0.5 m step
        cand = _candidate(q, xz)
        res = self.preflight.conforming_leading_segment(cand, max_ee_displacement_m=0.1)
        self.assertIsNone(res.candidate)
        self.assertIn("exceeds the EE cap", res.reason)

    def test_offline_gate_blocks_real_send_without_token(self) -> None:
        proof = PrePhysicalTrajectoryPreflight.assert_no_command_offline(default_no_token_gate())
        self.assertTrue(proof["real_send_blocked_without_token"])
        self.assertTrue(proof["mode_is_not_armed"])
        self.assertTrue(proof["armed_token_not_hardcoded_as_armed"])
        self.assertTrue(proof["allow_real_action_client_false"])
        self.assertFalse(proof["agent_exported_token"])

    def test_real_action_client_creation_blocked_without_arming(self) -> None:
        gate = ExecutionSafetyGate(ExecutionSafetyConfig(mode="dry_run"))
        client = SafeTrajectoryClient(gate)
        created = {"called": False}

        def factory(node, name):  # must never be reached
            created["called"] = True
            return object()

        with self.assertRaises(SafetyViolation):
            client.create_real_action_client(action_client_factory=factory)
        self.assertFalse(created["called"])
        self.assertFalse(bool(client.as_dict()["real_action_client_created"]))


class PrePhysicalPreflightRunnerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        from map_update_layer.experiments.pre_physical_preflight_runner import run_pre_physical_preflight

        cls.out = tempfile.mkdtemp(prefix="ag_preflight_test_")
        cls.report = run_pre_physical_preflight(out=cls.out)
        cls.inv = cls.report["invariants"]

    def test_status_complete_and_all_invariants(self) -> None:
        self.assertEqual(self.report["status"], "GOAL_STATUS: COMPLETE")
        self.assertTrue(self.inv["all_pass"], f"invariants: {self.inv}")

    def test_full_candidate_fails_precheck_on_ee_displacement(self) -> None:
        fr = self.report["full_record"]
        self.assertEqual(fr["verdict_eligibility"], "FAIL_PRECHECK")
        self.assertFalse(fr["checks"]["ee_displacement_within_cap"])
        self.assertGreater(fr["estimated_ee_displacement_m"], 0.1)
        self.assertTrue(any("EE displacement" in f for f in fr["failures"]))

    def test_conforming_segment_is_pass_eligible_and_object_free(self) -> None:
        seg = self.report["conforming_segment"]
        self.assertIsNotNone(seg["record"])
        rec = seg["record"]
        self.assertEqual(rec["verdict_eligibility"], "PASS_CONTROLLER_CONTRACT_TRAJECTORY_PREFLIGHT")
        self.assertTrue(rec["passed"])
        self.assertLessEqual(rec["estimated_ee_displacement_m"], 0.1 + 1e-9)
        self.assertEqual(rec["finger_start"], 0.0)
        self.assertEqual(rec["finger_end"], 0.0)
        self.assertEqual(rec["finger_policy"], "hold")
        self.assertAlmostEqual(rec["execution_dt_s"], 0.1)
        self.assertFalse(rec["uses_control_internal_dt_as_execution_dt"])

    def test_offline_gate_and_no_real_client(self) -> None:
        self.assertTrue(self.inv["real_send_blocked_without_token"])
        self.assertTrue(self.inv["no_real_action_client"])
        self.assertTrue(self.inv["allow_real_action_client_false"])

    def test_artifacts_written(self) -> None:
        out = Path(self.out)
        self.assertTrue((out / "report.json").exists())
        self.assertTrue((out / "commanded_joint_trajectory.csv").exists())
        # csv header matches the log-schema commanded trajectory columns
        head = (out / "commanded_joint_trajectory.csv").read_text(encoding="utf-8").splitlines()[0]
        self.assertEqual(
            head,
            "sample_index,time_from_start_s,joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,finger",
        )


if __name__ == "__main__":
    unittest.main()
