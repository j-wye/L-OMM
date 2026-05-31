#!/usr/bin/env python3
"""Reduced active-mode kinematics for q2-q3-q5 control."""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from constants import (
    ACTIVE_CHAIN_INDICES,
    ARM_BASE_Z,
    CAPSULE_PROXY_FRAME_INDICES,
    JOINT_OFFSETS,
    N_ACTIVE_JOINTS,
    N_CHAIN_JOINTS,
    Q4_FIXED,
    Q6_FIXED,
    Q_HOME_DEFAULT,
    Y_PLANE_FIXED,
)


class Kinematics:
    """Forward kinematics and reduced task Jacobian for the active submanifold."""

    def __init__(self) -> None:
        self._static_offsets = [self._transform_xyz_rpy(*off) for off in JOINT_OFFSETS]
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
        t[:3, :3] = cls._rot_z(q)
        return t

    @staticmethod
    def compose_chain_q(q_active: np.ndarray) -> np.ndarray:
        q = np.asarray(q_active, dtype=np.float64).reshape(-1)
        if q.shape[0] == N_ACTIVE_JOINTS:
            return np.array([q[0], q[1], Q4_FIXED, q[2], Q6_FIXED], dtype=np.float64)
        if q.shape[0] == N_CHAIN_JOINTS:
            return q.copy()
        raise ValueError(f"q_active must be length {N_ACTIVE_JOINTS}")

    def fk_chain(self, q_active: np.ndarray) -> np.ndarray:
        q_chain = self.compose_chain_q(q_active)
        out = np.empty((N_CHAIN_JOINTS + 1, 4, 4), dtype=np.float64)
        out[0] = self._base
        t_local = np.eye(4, dtype=np.float64)
        for i in range(N_CHAIN_JOINTS):
            t_local = t_local @ (self._static_offsets[i] @ self._transform_rz(float(q_chain[i])))
            out[i + 1] = self._base @ t_local
        return out

    def fk_ee_position(self, q_active: np.ndarray) -> np.ndarray:
        return self.fk_chain(q_active)[N_CHAIN_JOINTS, :3, 3].copy()

    def capsule_proxy_points_xz(self, q_active: np.ndarray) -> np.ndarray:
        """Return reduced planar capsule pivot points in the active xz plane."""
        chain = self.fk_chain(q_active)
        pts = chain[CAPSULE_PROXY_FRAME_INDICES, :3, 3][:, [0, 2]]
        return np.asarray(pts, dtype=np.float64)

    @staticmethod
    def ee_pitch_from_transform(t_ee: np.ndarray) -> float:
        forward = np.asarray(t_ee[:3, 2], dtype=np.float64)
        return float(math.atan2(float(forward[0]), float(forward[2])))

    def task_pose(self, q_active: np.ndarray) -> np.ndarray:
        t_ee = self.fk_chain(q_active)[N_CHAIN_JOINTS]
        p = t_ee[:3, 3]
        return np.array([p[0], p[2], self.ee_pitch_from_transform(t_ee)], dtype=np.float64)

    def fk_and_jacobian(self, q_active: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        chain = self.fk_chain(q_active)
        p_ee = chain[N_CHAIN_JOINTS, :3, 3]
        forward = chain[N_CHAIN_JOINTS, :3, 2]
        fx = float(forward[0])
        fz = float(forward[2])
        denom = max(fx * fx + fz * fz, 1.0e-10)
        jac = np.zeros((3, N_ACTIVE_JOINTS), dtype=np.float64)
        for out_col, chain_idx in enumerate(ACTIVE_CHAIN_INDICES.tolist()):
            axis = chain[chain_idx + 1, :3, 2]
            origin = chain[chain_idx + 1, :3, 3]
            lin = np.cross(axis, p_ee - origin)
            df = np.cross(axis, forward)
            dpitch = (fz * float(df[0]) - fx * float(df[2])) / denom
            jac[0, out_col] = lin[0]
            jac[1, out_col] = lin[2]
            jac[2, out_col] = dpitch
        return chain, jac

    @staticmethod
    def wrap_angle(a: float) -> float:
        return float((float(a) + math.pi) % (2.0 * math.pi) - math.pi)

    def task_error(self,
                   t_ee: np.ndarray,
                   xz_ref: np.ndarray,
                   pitch_ref: float) -> Tuple[np.ndarray, float, float]:
        xz = np.asarray(xz_ref, dtype=np.float64).reshape(2)
        p = t_ee[:3, 3]
        pitch_cur = self.ee_pitch_from_transform(t_ee)
        e = np.array([
            xz[0] - p[0],
            xz[1] - p[2],
            self.wrap_angle(float(pitch_ref) - pitch_cur),
        ], dtype=np.float64)
        return e, float(np.linalg.norm(e[:2])), float(abs(e[2]))

    def default_start_xz(self) -> Tuple[float, float]:
        p = self.fk_ee_position(np.asarray(Q_HOME_DEFAULT, dtype=np.float64))
        return float(p[0]), float(p[2])

    def default_start_xyz(self) -> np.ndarray:
        p = self.fk_ee_position(np.asarray(Q_HOME_DEFAULT, dtype=np.float64))
        p[1] = Y_PLANE_FIXED
        return p
