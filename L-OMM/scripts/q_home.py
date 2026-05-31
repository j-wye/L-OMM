#!/usr/bin/env python3
"""Build a q_home table for the reduced active-plane arm mode.

This script solves one reduced IK problem per target height.  The model is the
Gen3 Lite chain used by the project, with arm_joint_1 removed from the active
map and the wrist posture fixed:

  active: arm_joint_2, arm_joint_3, arm_joint_5
  fixed : arm_joint_4 = pi/2, arm_joint_6 = -pi/2

The solved home state keeps the end-effector near the measured active plane
start pose and points the end-effector/tool forward axis toward the target
contact point.

Default output files are written in the repository root:

  q_home_table.npy
  q_home_table_reduced.npy
  q_home_success.npy
  q_home_table.json
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np


ARM_BASE_Z = 0.47971
TOOL_LENGTH_M = 0.130

# Full robot joint order for reporting.
FULL_JOINT_NAMES = (
    "arm_joint_1",
    "arm_joint_2",
    "arm_joint_3",
    "arm_joint_4",
    "arm_joint_5",
    "arm_joint_6",
)

# Active-map kinematic convention inherited from map_generate.py: this offset
# list starts at arm_joint_2 and ends at arm_joint_6.
JOINT_OFFSETS: Tuple[Tuple[float, float, float, float, float, float], ...] = (
    (0.0,    -0.03,   0.115,   math.pi / 2.0,  0.0,           0.0),
    (0.0,     0.28,   0.0,    -math.pi,        0.0,           0.0),
    (0.0,    -0.14,   0.02,    math.pi / 2.0,  0.0,           0.0),
    (0.0285,  0.0,    0.105,   0.0,            math.pi / 2.0, 0.0),
    (-0.105,  0.0,    0.0285,  0.0,           -math.pi / 2.0, 0.0),
)

# Limits for arm_joint_2..6.
JL_LO_5 = np.array([-2.69, -2.69, -2.59, -2.57, -2.59], dtype=np.float64)
JL_HI_5 = np.array([ 2.69,  2.69,  2.59,  2.57,  2.59], dtype=np.float64)

# Reduced active variables are [q2, q3, q5], corresponding to q5d indices
# [0, 1, 3].  q4 and q6 are fixed.
ACTIVE_INDICES_5D = np.array([0, 1, 3], dtype=np.int64)
JL_LO_REDUCED = JL_LO_5[ACTIVE_INDICES_5D]
JL_HI_REDUCED = JL_HI_5[ACTIVE_INDICES_5D]

Q1_FIXED = 0.0
Q4_FIXED = math.pi / 2.0
Q6_FIXED = -math.pi / 2.0

# TF-verified posture that gives EE approximately [0, 0.047, 0.798] and
# forward axis approximately +x when the target height equals the EE height.
CANONICAL_REDUCED = np.array(
    [1.4608791881300178, 2.2480904647980657, 0.784002030285052],
    dtype=np.float64,
)


@dataclass
class SolveRecord:
    height: float
    success: bool
    q_full: np.ndarray
    q_5d: np.ndarray
    q_reduced: np.ndarray
    ee_position: np.ndarray
    tool_position: np.ndarray
    forward_axis: np.ndarray
    desired_axis: np.ndarray
    target_contact: np.ndarray
    desired_pitch_rad: float
    actual_pitch_rad: float
    pos_xz_error_m: float
    y_actual_m: float
    y_error_from_nominal_m: float
    forward_angle_error_rad: float
    sigma_min: float
    manipulability_abs_det: float
    joint_margin_min_rad: float
    iterations: int
    seed_name: str
    fail_reason: str


class ReducedKinematics:
    """Forward kinematics for q2..q6 with q1 removed from the active map."""

    def __init__(self) -> None:
        self._static = [self._transform_xyz_rpy(*off) for off in JOINT_OFFSETS]
        self._base = np.eye(4, dtype=np.float64)
        self._base[2, 3] = ARM_BASE_Z

    @staticmethod
    def _rot_x(a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.array([[1.0, 0.0, 0.0],
                         [0.0,   c,  -s],
                         [0.0,   s,   c]], dtype=np.float64)

    @staticmethod
    def _rot_y(a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.array([[  c, 0.0,   s],
                         [0.0, 1.0, 0.0],
                         [ -s, 0.0,   c]], dtype=np.float64)

    @staticmethod
    def _rot_z(a: float) -> np.ndarray:
        c, s = math.cos(a), math.sin(a)
        return np.array([[  c,  -s, 0.0],
                         [  s,   c, 0.0],
                         [0.0, 0.0, 1.0]], dtype=np.float64)

    @classmethod
    def _transform_xyz_rpy(cls,
                           x: float, y: float, z: float,
                           roll: float, pitch: float, yaw: float) -> np.ndarray:
        t = np.eye(4, dtype=np.float64)
        t[:3, 3] = [x, y, z]
        t[:3, :3] = cls._rot_z(yaw) @ cls._rot_y(pitch) @ cls._rot_x(roll)
        return t

    @classmethod
    def _transform_rz(cls, q: float) -> np.ndarray:
        t = np.eye(4, dtype=np.float64)
        t[:3, :3] = cls._rot_z(float(q))
        return t

    @staticmethod
    def compose_q5d(q_reduced: np.ndarray) -> np.ndarray:
        q = np.zeros(5, dtype=np.float64)
        q[0] = float(q_reduced[0])
        q[1] = float(q_reduced[1])
        q[2] = Q4_FIXED
        q[3] = float(q_reduced[2])
        q[4] = Q6_FIXED
        return q

    @staticmethod
    def compose_q_full(q_reduced: np.ndarray) -> np.ndarray:
        q5 = ReducedKinematics.compose_q5d(q_reduced)
        return np.array([Q1_FIXED, q5[0], q5[1], q5[2], q5[3], q5[4]], dtype=np.float64)

    def fk_chain_5d(self, q5d: np.ndarray) -> np.ndarray:
        q = np.asarray(q5d, dtype=np.float64).reshape(5)
        out = np.empty((6, 4, 4), dtype=np.float64)
        out[0] = self._base
        t_local = np.eye(4, dtype=np.float64)
        for i in range(5):
            t_local = t_local @ (self._static[i] @ self._transform_rz(float(q[i])))
            out[i + 1] = self._base @ t_local
        return out

    def fk_from_reduced(self, q_reduced: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        q5d = self.compose_q5d(q_reduced)
        return q5d, self.fk_chain_5d(q5d)

    def task(self, q_reduced: np.ndarray) -> np.ndarray:
        _, chain = self.fk_from_reduced(q_reduced)
        t_ee = chain[-1]
        p = t_ee[:3, 3]
        z_axis = unit(t_ee[:3, 2])
        pitch = math.atan2(float(z_axis[0]), float(z_axis[2]))
        return np.array([p[0], p[2], pitch], dtype=np.float64)

    def diagnostics(self,
                    q_reduced: np.ndarray,
                    *,
                    target_contact: np.ndarray,
                    nominal_y: float) -> Dict[str, np.ndarray | float]:
        q5d, chain = self.fk_from_reduced(q_reduced)
        t_ee = chain[-1]
        ee = t_ee[:3, 3].copy()
        forward = unit(t_ee[:3, 2])
        tool = ee + TOOL_LENGTH_M * forward
        desired = unit(np.asarray(target_contact, dtype=np.float64) - ee)
        angle_err = angle_between(forward, desired)
        actual_pitch = math.atan2(float(forward[0]), float(forward[2]))
        jac = numerical_jacobian(self, q_reduced)
        sigma = sigma_min(jac)
        return {
            "q5d": q5d,
            "ee": ee,
            "tool": tool,
            "forward": forward,
            "desired": desired,
            "angle_error": float(angle_err),
            "actual_pitch": float(actual_pitch),
            "jac": jac,
            "sigma_min": float(sigma),
            "mu_abs_det": abs(float(np.linalg.det(jac))),
            "joint_margin": min_joint_margin_reduced(q_reduced),
            "y_error": float(ee[1] - nominal_y),
        }


class ReducedHomeSolver:
    def __init__(self,
                 *,
                 pos_tol_m: float,
                 pitch_tol_rad: float,
                 max_iter: int,
                 dq_max: float,
                 lam: float) -> None:
        self.kin = ReducedKinematics()
        self.pos_tol_m = float(pos_tol_m)
        self.pitch_tol_rad = float(pitch_tol_rad)
        self.max_iter = int(max_iter)
        self.dq_max = float(dq_max)
        self.lam = float(lam)
        self._i3 = np.eye(3, dtype=np.float64)

    def solve(self,
              height: float,
              seeds: Sequence[Tuple[str, np.ndarray]],
              *,
              start_x: float,
              start_z: float,
              target_contact_x: float,
              nominal_y: float) -> SolveRecord:
        target_contact = np.array([target_contact_x, nominal_y, height], dtype=np.float64)
        desired_pitch = math.atan2(target_contact_x - start_x, height - start_z)
        desired_task = np.array([start_x, start_z, desired_pitch], dtype=np.float64)
        best = None

        for seed_name, seed in seeds:
            q, it = self._iterate(seed, desired_task)
            rec = self._make_record(
                q,
                height=height,
                desired_task=desired_task,
                target_contact=target_contact,
                nominal_y=nominal_y,
                iterations=it,
                seed_name=seed_name,
            )
            task_score = (
                0 if rec.success else 1,
                rec.pos_xz_error_m / max(self.pos_tol_m, 1.0e-12)
                + rec.forward_angle_error_rad / max(self.pitch_tol_rad, 1.0e-12),
                abs(rec.y_error_from_nominal_m),
                -rec.sigma_min,
                -rec.joint_margin_min_rad,
            )
            if best is None or task_score < best[0]:
                best = (task_score, rec)

        assert best is not None
        return best[1]

    def _iterate(self, seed: np.ndarray, desired_task: np.ndarray) -> Tuple[np.ndarray, int]:
        q = np.clip(np.asarray(seed, dtype=np.float64).reshape(3), JL_LO_REDUCED, JL_HI_REDUCED)
        best_q = q.copy()
        best_norm = float("inf")

        for it in range(1, self.max_iter + 1):
            cur = self.kin.task(q)
            err = task_error(desired_task, cur)
            err_norm = float(np.linalg.norm(err))
            if err_norm < best_norm:
                best_norm = err_norm
                best_q = q.copy()
            if math.hypot(err[0], err[1]) <= self.pos_tol_m and abs(err[2]) <= self.pitch_tol_rad:
                return q, it

            jac = numerical_jacobian(self.kin, q)
            a = jac @ jac.T + (self.lam * self.lam) * self._i3
            try:
                y = np.linalg.solve(a, err)
            except np.linalg.LinAlgError:
                y = np.linalg.pinv(a) @ err
            dq = jac.T @ y
            dq_inf = float(np.max(np.abs(dq)))
            if dq_inf > self.dq_max:
                dq *= self.dq_max / max(dq_inf, 1.0e-12)
            q = np.clip(q + dq, JL_LO_REDUCED, JL_HI_REDUCED)

        return best_q, self.max_iter

    def _make_record(self,
                     q: np.ndarray,
                     *,
                     height: float,
                     desired_task: np.ndarray,
                     target_contact: np.ndarray,
                     nominal_y: float,
                     iterations: int,
                     seed_name: str) -> SolveRecord:
        diag = self.kin.diagnostics(q, target_contact=target_contact, nominal_y=nominal_y)
        q5d = np.asarray(diag["q5d"], dtype=np.float64)
        q_full = ReducedKinematics.compose_q_full(q)
        ee = np.asarray(diag["ee"], dtype=np.float64)
        task_cur = self.kin.task(q)
        err = task_error(desired_task, task_cur)
        pos_xz = float(math.hypot(err[0], err[1]))
        angle = float(diag["angle_error"])
        margin = float(diag["joint_margin"])
        success = bool(pos_xz <= self.pos_tol_m and angle <= self.pitch_tol_rad and margin >= 0.0)
        fail = "" if success else "reduced_ik_tolerance_not_met"
        return SolveRecord(
            height=float(height),
            success=success,
            q_full=q_full,
            q_5d=q5d,
            q_reduced=np.asarray(q, dtype=np.float64).copy(),
            ee_position=ee,
            tool_position=np.asarray(diag["tool"], dtype=np.float64),
            forward_axis=np.asarray(diag["forward"], dtype=np.float64),
            desired_axis=np.asarray(diag["desired"], dtype=np.float64),
            target_contact=np.asarray(target_contact, dtype=np.float64),
            desired_pitch_rad=float(desired_task[2]),
            actual_pitch_rad=float(diag["actual_pitch"]),
            pos_xz_error_m=pos_xz,
            y_actual_m=float(ee[1]),
            y_error_from_nominal_m=float(diag["y_error"]),
            forward_angle_error_rad=angle,
            sigma_min=float(diag["sigma_min"]),
            manipulability_abs_det=float(diag["mu_abs_det"]),
            joint_margin_min_rad=margin,
            iterations=int(iterations),
            seed_name=seed_name,
            fail_reason=fail,
        )


def numerical_jacobian(kin: ReducedKinematics,
                       q: np.ndarray,
                       eps: float = 1.0e-6) -> np.ndarray:
    q0 = np.asarray(q, dtype=np.float64).reshape(3)
    j = np.zeros((3, 3), dtype=np.float64)
    for i in range(3):
        step = np.zeros(3, dtype=np.float64)
        step[i] = eps
        tp = kin.task(q0 + step)
        tm = kin.task(q0 - step)
        diff = tp - tm
        diff[2] = wrap_angle(tp[2] - tm[2])
        j[:, i] = diff / (2.0 * eps)
    return j


def task_error(desired: np.ndarray, current: np.ndarray) -> np.ndarray:
    e = np.asarray(desired, dtype=np.float64) - np.asarray(current, dtype=np.float64)
    e[2] = wrap_angle(e[2])
    return e


def unit(v: np.ndarray) -> np.ndarray:
    a = np.asarray(v, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(a))
    if n <= 1.0e-12:
        return np.array([1.0, 0.0, 0.0], dtype=np.float64)
    return a / n


def angle_between(a: np.ndarray, b: np.ndarray) -> float:
    aa = unit(a)
    bb = unit(b)
    return float(math.acos(float(np.clip(np.dot(aa, bb), -1.0, 1.0))))


def wrap_angle(a: float) -> float:
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def sigma_min(jac: np.ndarray) -> float:
    try:
        s = np.linalg.svd(jac, compute_uv=False)
    except np.linalg.LinAlgError:
        return 0.0
    return float(s[-1]) if s.size else 0.0


def min_joint_margin_reduced(q: np.ndarray) -> float:
    qv = np.asarray(q, dtype=np.float64).reshape(3)
    return float(np.min(np.concatenate([qv - JL_LO_REDUCED, JL_HI_REDUCED - qv])))


def inclusive_range(start: float, stop: float, step: float, decimals: int = 6) -> List[float]:
    n = int(round((stop - start) / step))
    return [round(start + i * step, decimals) for i in range(n + 1)]


def parse_float_list(raw: str | None) -> List[float] | None:
    if raw is None:
        return None
    vals = []
    for token in raw.replace(";", ",").split(","):
        token = token.strip()
        if token:
            vals.append(float(token))
    return vals


def record_to_json(rec: SolveRecord) -> Dict[str, object]:
    return {
        "height": rec.height,
        "success": rec.success,
        "q_full_arm_joint_1_to_6": rec.q_full.tolist(),
        "q_5d_arm_joint_2_to_6": rec.q_5d.tolist(),
        "q_reduced_arm_joint_2_3_5": rec.q_reduced.tolist(),
        "ee_position_xyz_m": rec.ee_position.tolist(),
        "tool_position_xyz_m": rec.tool_position.tolist(),
        "forward_axis_ee_z_in_base": rec.forward_axis.tolist(),
        "desired_axis_to_target_contact": rec.desired_axis.tolist(),
        "target_contact_xyz_m": rec.target_contact.tolist(),
        "desired_pitch_rad": rec.desired_pitch_rad,
        "desired_pitch_deg": math.degrees(rec.desired_pitch_rad),
        "actual_pitch_rad": rec.actual_pitch_rad,
        "actual_pitch_deg": math.degrees(rec.actual_pitch_rad),
        "pos_xz_error_m": rec.pos_xz_error_m,
        "y_actual_m": rec.y_actual_m,
        "y_error_from_nominal_m": rec.y_error_from_nominal_m,
        "forward_angle_error_rad": rec.forward_angle_error_rad,
        "forward_angle_error_deg": math.degrees(rec.forward_angle_error_rad),
        "sigma_min": rec.sigma_min,
        "manipulability_abs_det": rec.manipulability_abs_det,
        "joint_margin_min_rad": rec.joint_margin_min_rad,
        "iterations": rec.iterations,
        "seed_name": rec.seed_name,
        "fail_reason": rec.fail_reason,
    }


def solve_height_table(args: argparse.Namespace) -> Tuple[List[SolveRecord], Dict[str, object]]:
    heights = parse_float_list(args.heights)
    if heights is None:
        heights = inclusive_range(args.h_min, args.h_max, args.h_step, 3)

    solver = ReducedHomeSolver(
        pos_tol_m=args.pos_tol,
        pitch_tol_rad=math.radians(args.pitch_tol_deg),
        max_iter=args.max_iter,
        dq_max=args.dq_max,
        lam=args.lam,
    )

    seeds_base = [
        ("canonical", CANONICAL_REDUCED),
        ("canonical_q5_minus_0p5", CANONICAL_REDUCED + np.array([0.0, 0.0, -0.5])),
        ("canonical_q5_plus_0p5", CANONICAL_REDUCED + np.array([0.0, 0.0, 0.5])),
        ("zero", np.zeros(3, dtype=np.float64)),
    ]

    records: List[SolveRecord] = []
    prev_q: np.ndarray | None = None
    for height in heights:
        seeds: List[Tuple[str, np.ndarray]] = []
        if prev_q is not None:
            seeds.append(("previous_height", prev_q))
        seeds.extend(seeds_base)
        rec = solver.solve(
            float(height),
            seeds,
            start_x=float(args.start_x),
            start_z=float(args.start_z),
            target_contact_x=float(args.target_x),
            nominal_y=float(args.nominal_y),
        )
        records.append(rec)
        prev_q = rec.q_reduced.copy()
        status = "OK" if rec.success else "FAIL"
        print(
            f"[{status}] h={rec.height:.3f} q235="
            f"[{rec.q_reduced[0]: .6f}, {rec.q_reduced[1]: .6f}, {rec.q_reduced[2]: .6f}] "
            f"ee=({rec.ee_position[0]:+.4f},{rec.ee_position[1]:+.4f},{rec.ee_position[2]:+.4f}) "
            f"y_err={rec.y_error_from_nominal_m*1000.0:+.2f}mm "
            f"xz_err={rec.pos_xz_error_m*1000.0:.4f}mm "
            f"gaze_err={math.degrees(rec.forward_angle_error_rad):.5f}deg "
            f"sigma={rec.sigma_min:.4e}"
        )

    success_count = sum(1 for r in records if r.success)
    y_values = np.array([r.y_actual_m for r in records], dtype=np.float64)
    meta = {
        "created_at_unix": time.time(),
        "description": "Reduced active-plane q_home table for fixed q4/q6 posture.",
        "joint_names_full": list(FULL_JOINT_NAMES),
        "joint_names_reduced": ["arm_joint_2", "arm_joint_3", "arm_joint_5"],
        "active_joint_policy": "solve arm_joint_2, arm_joint_3, arm_joint_5 only",
        "fixed_joint_policy": {
            "arm_joint_1": Q1_FIXED,
            "arm_joint_4": Q4_FIXED,
            "arm_joint_6": Q6_FIXED,
        },
        "arm_base_z": ARM_BASE_Z,
        "tool_length_m": TOOL_LENGTH_M,
        "start_pose_request": {
            "ee_x_m": float(args.start_x),
            "ee_y_nominal_m": float(args.nominal_y),
            "ee_z_m": float(args.start_z),
        },
        "target_contact_x_m": float(args.target_x),
        "heights": [float(h) for h in heights],
        "height_count": len(heights),
        "success_count": int(success_count),
        "success_ratio": float(success_count / max(len(records), 1)),
        "pos_tol_m": float(args.pos_tol),
        "pitch_tol_deg": float(args.pitch_tol_deg),
        "y_actual_min_m": float(np.min(y_values)) if y_values.size else None,
        "y_actual_max_m": float(np.max(y_values)) if y_values.size else None,
        "y_actual_mean_m": float(np.mean(y_values)) if y_values.size else None,
        "y_actual_range_m": float(np.max(y_values) - np.min(y_values)) if y_values.size else None,
        "q_table_shape": [len(records), 6],
        "q_reduced_table_shape": [len(records), 3],
    }
    return records, meta


def write_outputs(args: argparse.Namespace,
                  records: Sequence[SolveRecord],
                  meta: Dict[str, object]) -> None:
    out_dir = os.path.abspath(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    q_full = np.vstack([r.q_full for r in records]).astype(np.float64)
    q_reduced = np.vstack([r.q_reduced for r in records]).astype(np.float64)
    success = np.array([r.success for r in records], dtype=bool)

    np.save(os.path.join(out_dir, "q_home_table.npy"), q_full)
    np.save(os.path.join(out_dir, "q_home_table_reduced.npy"), q_reduced)
    np.save(os.path.join(out_dir, "q_home_success.npy"), success)

    payload = dict(meta)
    payload["records"] = [record_to_json(r) for r in records]
    with open(os.path.join(out_dir, "q_home_table.json"), "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate q_home table for reduced q2-q3-q5 active mode."
    )
    parser.add_argument("--out-dir", default=".")
    parser.add_argument("--h-min", type=float, default=0.40)
    parser.add_argument("--h-max", type=float, default=0.90)
    parser.add_argument("--h-step", type=float, default=0.01)
    parser.add_argument("--heights", default=None,
                        help="Optional comma-separated explicit target heights.")
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-z", type=float, default=0.8)
    parser.add_argument("--nominal-y", type=float, default=0.047)
    parser.add_argument("--target-x", type=float, default=0.53)
    parser.add_argument("--pos-tol", type=float, default=1.0e-7)
    parser.add_argument("--pitch-tol-deg", type=float, default=1.0e-4)
    parser.add_argument("--max-iter", type=int, default=300)
    parser.add_argument("--dq-max", type=float, default=0.08)
    parser.add_argument("--lam", type=float, default=1.0e-5)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    records, meta = solve_height_table(args)
    write_outputs(args, records, meta)
    print("=" * 80)
    print(f"Saved q_home outputs to: {os.path.abspath(args.out_dir)}")
    print(f"Success: {meta['success_count']} / {meta['height_count']} ({100.0 * meta['success_ratio']:.2f}%)")
    print(
        "y actual range: "
        f"{meta['y_actual_min_m']:.6f} .. {meta['y_actual_max_m']:.6f} m "
        f"(span {1000.0 * float(meta['y_actual_range_m']):.3f} mm)"
    )
    print("Files:")
    print("  q_home_table.npy")
    print("  q_home_table_reduced.npy")
    print("  q_home_success.npy")
    print("  q_home_table.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
