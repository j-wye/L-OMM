"""Side-grasp Decision Module.

Raw geometry is computed in ``gripper_camera_color_optical_frame``; state and
the published ``T_map_target`` live in map frame. The decision core consumes
ROI-ready GPU tensors prepared by an upstream thin boundary.
"""

import argparse
from typing import Tuple

import torch
import torch.nn.functional as F


class DecisionModule:
    """Side-grasp pose estimator (optical raw geometry + map-frame state)."""

    _STATUS_RESET = 0
    _STATUS_REJECTED = 1
    _STATUS_HELD = 2
    _STATUS_FRESH = 3

    _REASON_NONE = 0
    _REASON_INSUFFICIENT_POINTS = 1
    _REASON_APPROACH_PRIOR_DEGENERATE = 2
    _REASON_INITIAL_PHYSICAL_GATE = 3
    _REASON_INITIAL_HORIZONTAL_COMPONENT_MISSING = 4
    _REASON_TRACKING_PHYSICAL_GATE = 5

    @staticmethod
    @torch.jit.script
    def _guided_filter_impl(
        rgb_crop: torch.Tensor,
        depth_crop: torch.Tensor,
        radius: int,
        k_size: int,
        eps: float,
    ) -> torch.Tensor:
        guide = (
            0.299 * rgb_crop[0] + 0.587 * rgb_crop[1] + 0.114 * rgb_crop[2]
        ).unsqueeze(0).unsqueeze(0)
        p = depth_crop.unsqueeze(0).unsqueeze(0)

        mean_I = F.avg_pool2d(guide, k_size, 1, radius, count_include_pad=False)
        mean_p = F.avg_pool2d(p, k_size, 1, radius, count_include_pad=False)
        mean_Ip = F.avg_pool2d(guide * p, k_size, 1, radius, count_include_pad=False)
        mean_II = F.avg_pool2d(guide * guide, k_size, 1, radius, count_include_pad=False)

        a = (mean_Ip - mean_I * mean_p) / (mean_II - mean_I * mean_I + eps)
        b = mean_p - a * mean_I

        mean_a = F.avg_pool2d(a, k_size, 1, radius, count_include_pad=False)
        mean_b = F.avg_pool2d(b, k_size, 1, radius, count_include_pad=False)

        depth_est = mean_a * guide + mean_b
        return torch.where(p > 0.0, p, depth_est).squeeze()

    @staticmethod
    @torch.jit.script
    def _mask_guided_backprojection_impl(
        depth_guided: torch.Tensor,
        mask_crop: torch.Tensor,
        x_base_crop: torch.Tensor,
        y_base_crop: torch.Tensor,
        depth_band_m: float,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        valid = torch.logical_and(mask_crop > 0.5, depth_guided > 0.001)
        raw_count = torch.sum(valid.to(torch.int32))

        if depth_band_m > 0.0:
            z_all = depth_guided[valid]
            if z_all.shape[0] > 10:
                z_med = torch.median(z_all)
                band = torch.abs(depth_guided - z_med) <= depth_band_m
                valid = torch.logical_and(valid, band)

        band_count = torch.sum(valid.to(torch.int32))
        z = depth_guided[valid]
        if z.shape[0] < 10:
            empty = torch.empty((0, 3), device=depth_guided.device, dtype=depth_guided.dtype)
            return empty, raw_count.view(1), band_count.view(1)

        x = x_base_crop[valid] * z
        y = y_base_crop[valid] * z
        return torch.stack((x, y, z), dim=1), raw_count.view(1), band_count.view(1)

    @staticmethod
    @torch.jit.script
    def _weiszfeld_median_impl(points: torch.Tensor) -> torch.Tensor:
        c = torch.mean(points, dim=0, keepdim=True)
        for _ in range(5):
            d = points - c
            w = torch.rsqrt(torch.sum(d * d, dim=1, keepdim=True) + 1.0e-6)
            c = torch.sum(w * points, dim=0, keepdim=True) / (torch.sum(w) + 1.0e-6)
        return c.squeeze()

    @staticmethod
    @torch.jit.script
    def _plane_basis_impl(g: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        # Branchless RH basis (e1, e2) orthogonal to g.
        absg = torch.abs(g)
        mask = (absg == torch.min(absg)).to(g.dtype)
        ref = mask / (torch.sum(mask) + 1.0e-8)
        e1 = ref - torch.dot(ref, g) * g
        e1 = e1 / (torch.norm(e1) + 1.0e-8)
        e2 = torch.cross(g, e1, dim=0)
        e2 = e2 / (torch.norm(e2) + 1.0e-8)
        return e1, e2

    @staticmethod
    @torch.jit.script
    def _pca_2d_horizontal_impl(
        points: torch.Tensor,
        anchor: torch.Tensor,
        g: torch.Tensor,
        e1: torch.Tensor,
        e2: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        # Closed-form 2x2 PCA in the g-orthogonal plane; (h1, h2, g) right-handed.
        centered = points - anchor
        vertical = torch.matmul(centered, g).unsqueeze(1) * g.unsqueeze(0)
        horizontal = centered - vertical
        horizontal = horizontal - torch.mean(horizontal, dim=0, keepdim=True)
        u = torch.matmul(horizontal, e1)
        v = torch.matmul(horizontal, e2)
        n_m1 = float(points.shape[0] - 1)
        a = torch.sum(u * u) / n_m1
        c = torch.sum(v * v) / n_m1
        b = torch.sum(u * v) / n_m1
        tr = a + c
        disc = torch.sqrt((a - c) * (a - c) + 4.0 * b * b + 1.0e-20)
        lambda_max = 0.5 * (tr + disc)
        lambda_min = 0.5 * (tr - disc)
        theta2d = 0.5 * torch.atan2(2.0 * b, a - c)
        ct = torch.cos(theta2d)
        st = torch.sin(theta2d)
        h1 = ct * e1 + st * e2
        h1 = h1 / (torch.norm(h1) + 1.0e-8)
        h2 = torch.cross(g, h1, dim=0)
        h2 = h2 / (torch.norm(h2) + 1.0e-8)
        q_lambda = torch.clamp(
            (lambda_max - lambda_min) / (lambda_max + lambda_min + 1.0e-12), 0.0, 1.0
        )
        return h1, h2, lambda_max, lambda_min, q_lambda

    @staticmethod
    @torch.jit.script
    def _project_h_axes_quantiles_impl(
        centered: torch.Tensor,
        h1: torch.Tensor,
        h2: torch.Tensor,
        z: torch.Tensor,
        q_low: float,
        q_high: float,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        B = torch.stack((h1, h2, z), dim=1)
        Q = torch.matmul(centered, B)
        n = Q.shape[0]
        k_lo = max(1, min(n, int(q_low * float(n - 1)) + 1))
        k_hi = max(k_lo, min(n, int(q_high * float(n - 1)) + 1))
        q_lo, _ = torch.kthvalue(Q, k_lo, dim=0)
        q_hi, _ = torch.kthvalue(Q, k_hi, dim=0)
        return q_lo, q_hi

    @staticmethod
    @torch.jit.script
    def _extent_midpoint_impl(
        centered_points: torch.Tensor,
        r_cached: torch.Tensor,
        q_low: float,
        q_high: float,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        q_local = torch.matmul(centered_points, r_cached)
        n = q_local.shape[0]
        k_lo = max(1, min(n, int(q_low * float(n - 1)) + 1))
        k_hi = max(k_lo, min(n, int(q_high * float(n - 1)) + 1))

        q_lo, _ = torch.kthvalue(q_local, k_lo, dim=0)
        q_hi, _ = torch.kthvalue(q_local, k_hi, dim=0)

        mids = 0.5 * (q_lo + q_hi)
        half_extents = 0.5 * (q_hi - q_lo)
        width_close = q_hi[1] - q_lo[1]
        length_app = q_hi[0] - q_lo[0]
        front_shell_offset = q_lo[0]
        return width_close, length_app, front_shell_offset, mids, half_extents

    @staticmethod
    def _wrap_angle(a: torch.Tensor) -> torch.Tensor:
        return torch.atan2(torch.sin(a), torch.cos(a))

    _GRAVITY_AXIS_MAP: Tuple[float, float, float] = (0.0, 0.0, 1.0)

    def __init__(
        self,
        device: str = "cuda",
        fx: float = 911.454,
        fy: float = 910.185,
        cx: float = 650.843,
        cy: float = 367.997,
        ema_alpha: float = 0.20,
        axis_alpha: float = 0.20,
        quality_thr: float = 0.15,
        dtheta_max: float = 0.35,
        center_q_low: float = 0.03,
        center_q_high: float = 0.97,
        subsample_step: int = 2,
        subsample_min_points: int = 2048,
        gripper_width_limit: float = 0.080,
        gripper_length_limit: float = 0.04,
        finger_margin: float = 0.0,
        inward_fraction: float = 0.25,
        depth_band_m: float = 0.08,
        min_valid_points: int = 80,
        min_measurement_depth: float = 0.05,
        max_measurement_depth: float = 2.00,
        min_object_width: float = 0.003,
        max_object_width: float = 0.200,
        min_object_depth: float = 0.003,
        max_object_depth: float = 0.300,
        max_center_jump: float = 0.200,
        max_center_step: float = 0.050,
        max_extent_jump: float = 0.060,
        prefer_short_approach: bool = True,
        lambda_degenerate_thr: float = 0.10,
        lambda_recover_thr: float = 0.15,
        extent_degenerate_thr: float = 0.10,
        extent_recover_thr: float = 0.15,
        min_approach_prior_norm: float = 0.05,
    ):
        self.device = torch.device(device)

        self.crop_margin = 10
        self._cx = float(cx)
        self._cy = float(cy)
        self._inv_fx = 1.0 / float(fx)
        self._inv_fy = 1.0 / float(fy)

        self.gf_k_size = 5
        self.gf_radius = 2
        self.gf_eps = 1.0e-3

        g = torch.tensor(
            self._GRAVITY_AXIS_MAP, device=self.device, dtype=torch.float32
        )
        self.gravity_axis_map = g / (torch.norm(g) + 1.0e-8)

        # Optical-frame camera-forward axis (e_z_optical).
        self._e_z_optical = torch.tensor(
            [0.0, 0.0, 1.0], device=self.device, dtype=torch.float32
        )

        # Hardware width contract: W_safe = 0.080 m (margin-inclusive).
        self.gripper_width_limit = torch.tensor(float(gripper_width_limit), device=self.device)
        self.gripper_length_limit = torch.tensor(float(gripper_length_limit), device=self.device)
        self.finger_margin = torch.tensor(float(finger_margin), device=self.device)
        self.inward_fraction = torch.tensor(float(inward_fraction), device=self.device)

        self._ema_alpha = float(ema_alpha)
        self._axis_alpha = float(axis_alpha)
        self._quality_thr = float(quality_thr)
        self._dtheta_max = float(dtheta_max)
        self._q_low = float(center_q_low)
        self._q_high = float(center_q_high)
        self._subsample_step = max(1, int(subsample_step))
        self._subsample_min = max(64, int(subsample_min_points))
        self._prefer_short_approach = bool(prefer_short_approach)
        self.depth_band_m = float(depth_band_m)

        self._min_valid_points = max(10, int(min_valid_points))
        self._min_meas_z = torch.tensor(float(min_measurement_depth), device=self.device)
        self._max_meas_z = torch.tensor(float(max_measurement_depth), device=self.device)
        self._min_obj_w = torch.tensor(float(min_object_width), device=self.device)
        self._max_obj_w = torch.tensor(float(max_object_width), device=self.device)
        self._min_obj_d = torch.tensor(float(min_object_depth), device=self.device)
        self._max_obj_d = torch.tensor(float(max_object_depth), device=self.device)
        self._max_center_step_t = torch.tensor(float(max_center_step), device=self.device)
        self._max_center_jump_sq_t = torch.tensor(
            float(max_center_jump * max_center_jump), device=self.device
        )
        self._max_extent_jump_sq_t = torch.tensor(
            float(max_extent_jump * max_extent_jump), device=self.device
        )

        # Degeneracy hysteresis thresholds (enter / recover).
        self._lambda_deg_thr = torch.tensor(float(lambda_degenerate_thr), device=self.device)
        self._lambda_rec_thr = torch.tensor(float(lambda_recover_thr), device=self.device)
        self._extent_deg_thr = torch.tensor(float(extent_degenerate_thr), device=self.device)
        self._extent_rec_thr = torch.tensor(float(extent_recover_thr), device=self.device)
        self._min_approach_prior_norm = torch.tensor(
            float(min_approach_prior_norm), device=self.device
        )

        self._zero_pitch = torch.tensor(0.0, device=self.device)
        self._zero_scalar = torch.tensor(0.0, device=self.device)
        self._axis_alpha_t = torch.tensor(float(axis_alpha), device=self.device)
        self._T_map_target_buf = torch.eye(4, device=self.device)
        # Raw target pose lives in optical frame; distinct from the lifted map output.
        self._T_optical_target_raw_buf = torch.eye(4, device=self.device)
        self._status_reset_t = torch.tensor(
            [self._STATUS_RESET], dtype=torch.int32, device=self.device
        )
        self._status_rejected_t = torch.tensor(
            [self._STATUS_REJECTED], dtype=torch.int32, device=self.device
        )
        self._status_held_t = torch.tensor(
            [self._STATUS_HELD], dtype=torch.int32, device=self.device
        )
        self._status_fresh_t = torch.tensor(
            [self._STATUS_FRESH], dtype=torch.int32, device=self.device
        )
        self._reason_none_t = torch.tensor(
            [self._REASON_NONE], dtype=torch.int32, device=self.device
        )
        self._reason_insufficient_points_t = torch.tensor(
            [self._REASON_INSUFFICIENT_POINTS], dtype=torch.int32, device=self.device
        )
        self._reason_approach_prior_degenerate_t = torch.tensor(
            [self._REASON_APPROACH_PRIOR_DEGENERATE],
            dtype=torch.int32,
            device=self.device,
        )
        self._reason_initial_physical_gate_t = torch.tensor(
            [self._REASON_INITIAL_PHYSICAL_GATE],
            dtype=torch.int32,
            device=self.device,
        )
        self._reason_initial_horizontal_component_missing_t = torch.tensor(
            [self._REASON_INITIAL_HORIZONTAL_COMPONENT_MISSING],
            dtype=torch.int32,
            device=self.device,
        )
        self._reason_tracking_physical_gate_t = torch.tensor(
            [self._REASON_TRACKING_PHYSICAL_GATE],
            dtype=torch.int32,
            device=self.device,
        )
        self._true_t = torch.tensor([True], dtype=torch.bool, device=self.device)
        self._false_t = torch.tensor([False], dtype=torch.bool, device=self.device)

        self._init_state()

    def _init_state(self) -> None:
        dev = self.device
        self._axes_initialized = False
        self._frame_count = 0

        self._p_map = torch.zeros(3, device=dev)
        self._theta_map = torch.zeros((), device=dev)
        self._extent_cached = torch.zeros(3, device=dev)
        self._front_shell_offset = torch.zeros((), device=dev)

        self._last_center_gate = torch.zeros(1, dtype=torch.bool, device=dev)
        self._last_axis_gate = torch.zeros(1, dtype=torch.bool, device=dev)
        self._last_quality = torch.zeros(1, device=dev)
        self._last_axis_quality = torch.zeros(1, device=dev)
        self._last_extent_ratio = torch.zeros(1, device=dev)
        self._last_feasible = torch.zeros(1, dtype=torch.bool, device=dev)
        self._is_degenerate = torch.zeros(1, dtype=torch.bool, device=dev)
        self._has_valid_state = torch.zeros(1, dtype=torch.bool, device=dev)
        self._last_z_median = torch.zeros(1, device=dev)
        self._last_center_jump_sq = torch.zeros(1, device=dev)
        self._last_extent_jump_sq = torch.zeros(1, device=dev)
        self._last_theta_raw = torch.zeros((), device=dev)
        self._last_status_code = self._status_reset_t.clone()
        self._last_reason_code = self._reason_none_t.clone()
        self._last_output_valid = torch.zeros(1, dtype=torch.bool, device=dev)
        self._last_output_stale = torch.zeros(1, dtype=torch.bool, device=dev)
        self._last_n_points = torch.zeros(1, dtype=torch.int32, device=dev)
        self._last_point_count_raw = torch.zeros(1, dtype=torch.int32, device=dev)
        self._last_point_count_band = torch.zeros(1, dtype=torch.int32, device=dev)

    def reset(self) -> None:
        self._init_state()

    def update_intrinsics(self, fx: float, fy: float, cx: float, cy: float) -> None:
        self._inv_fx = 1.0 / float(fx)
        self._inv_fy = 1.0 / float(fy)
        self._cx = float(cx)
        self._cy = float(cy)

    def _device_matches(self, dev: torch.device) -> bool:
        if dev.type != self.device.type:
            return False
        if self.device.index is None:
            return True
        return dev.index == self.device.index

    @property
    def last_status_code(self) -> torch.Tensor:
        return self._last_status_code.clone()

    @property
    def last_reason_code(self) -> torch.Tensor:
        return self._last_reason_code.clone()

    @property
    def last_output_valid(self) -> torch.Tensor:
        return self._last_output_valid.clone()

    @property
    def last_output_stale(self) -> torch.Tensor:
        return self._last_output_stale.clone()

    @property
    def last_point_count_raw(self) -> torch.Tensor:
        return self._last_point_count_raw.clone()

    @property
    def last_point_count_band(self) -> torch.Tensor:
        return self._last_point_count_band.clone()

    def _set_point_counts(
        self, raw_count: torch.Tensor, band_count: torch.Tensor, n_points: int
    ) -> None:
        self._last_point_count_raw.copy_(raw_count.to(dtype=torch.int32))
        self._last_point_count_band.copy_(band_count.to(dtype=torch.int32))
        self._last_n_points.fill_(int(n_points))

    def _compute_R_map_target_cached(self) -> torch.Tensor:
        cs = torch.cos(self._theta_map)
        sn = torch.sin(self._theta_map)
        zero = torch.zeros_like(cs)
        x_map = torch.stack([cs, sn, zero])
        y_map = torch.stack([-sn, cs, zero])
        z_map = self.gravity_axis_map
        return torch.stack([x_map, y_map, z_map], dim=1)

    @staticmethod
    def _theta_from_x_map(x_map: torch.Tensor, g_map: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        x_h = x_map - torch.dot(x_map, g_map) * g_map
        norm = torch.norm(x_h)
        x_h_n = x_h / (norm + 1.0e-6)
        theta = torch.atan2(x_h_n[1], x_h_n[0])
        return theta, norm

    def _compute_approach_prior_optical(
        self, g_optical: torch.Tensor, fallback_axis: torch.Tensor
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        a_raw = self._e_z_optical - torch.dot(self._e_z_optical, g_optical) * g_optical
        a_norm = torch.norm(a_raw)
        ok = a_norm >= self._min_approach_prior_norm
        a_optical = a_raw / (a_norm + 1.0e-8)
        a_optical = torch.where(ok, a_optical, fallback_axis)
        return a_optical, ok

    def _depth_gate_tensor(self, points_optical: torch.Tensor) -> torch.Tensor:
        z_med = torch.median(points_optical[:, 2])
        self._last_z_median.copy_(z_med.view(1))
        return torch.logical_and(z_med >= self._min_meas_z, z_med <= self._max_meas_z)

    def _extent_gate_tensor(
        self, width: torch.Tensor, depth_obj: torch.Tensor, half_ext: torch.Tensor
    ) -> torch.Tensor:
        finite = torch.logical_and(
            torch.isfinite(width),
            torch.logical_and(torch.isfinite(depth_obj), torch.isfinite(half_ext).all()),
        )
        width_ok = torch.logical_and(width >= self._min_obj_w, width <= self._max_obj_w)
        depth_ok = torch.logical_and(depth_obj >= self._min_obj_d, depth_obj <= self._max_obj_d)
        return torch.logical_and(finite, torch.logical_and(width_ok, depth_ok))

    def _select_candidate_frame_tensor(
        self,
        h1: torch.Tensor,
        h2: torch.Tensor,
        z_optical: torch.Tensor,
        a_optical: torch.Tensor,
        q_lo: torch.Tensor,
        q_hi: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
        # Two candidates: y = h1 or y = h2; x_raw = y x z_optical, signed to
        # agree with a_optical. Ranking: approach-prior > width safety margin >
        # prefer-short (or prefer-wide) approach.
        dtype = h1.dtype
        dev = h1.device

        extent_h1 = q_hi[0] - q_lo[0]
        extent_h2 = q_hi[1] - q_lo[1]
        w1 = extent_h1
        w2 = extent_h2
        d1 = extent_h2
        d2 = extent_h1

        x1_raw = torch.cross(h1, z_optical, dim=0)
        x2_raw = torch.cross(h2, z_optical, dim=0)

        dot1 = torch.dot(x1_raw, a_optical)
        dot2 = torch.dot(x2_raw, a_optical)
        one = torch.ones((), device=dev, dtype=dtype)
        s1 = torch.where(dot1 >= 0.0, one, -one)
        s2 = torch.where(dot2 >= 0.0, one, -one)

        x1 = s1 * x1_raw
        y1 = s1 * h1
        x2 = s2 * x2_raw
        y2 = s2 * h2

        score1 = torch.abs(dot1)
        score2 = torch.abs(dot2)

        eff_gripper = torch.maximum(
            self.gripper_width_limit - self.finger_margin, self._zero_scalar
        )
        f1 = w1 <= eff_gripper
        f2 = w2 <= eff_gripper
        both = torch.logical_and(f1, f2)
        only1 = torch.logical_and(f1, torch.logical_not(f2))
        neither = torch.logical_and(torch.logical_not(f1), torch.logical_not(f2))

        prefer_1 = score1 >= score2
        prefer_1_width = w1 <= w2
        if self._prefer_short_approach:
            tertiary_1 = d1 <= d2
        else:
            tertiary_1 = d1 >= d2
        scores_equal = torch.abs(score1 - score2) < 1.0e-9
        widths_equal = torch.abs(w1 - w2) < 1.0e-9
        secondary_1 = torch.where(widths_equal, tertiary_1, prefer_1_width)
        prefer_1_pri = torch.where(scores_equal, secondary_1, prefer_1)

        use_1 = torch.logical_or(
            only1,
            torch.logical_or(
                torch.logical_and(both, prefer_1_pri),
                torch.logical_and(neither, prefer_1_width),
            ),
        )
        use_2 = torch.logical_not(use_1)

        g1 = use_1.to(dtype)
        g2 = use_2.to(dtype)
        x_sel = g1 * x1 + g2 * x2
        y_sel = g1 * y1 + g2 * y2
        r_optical = torch.stack([x_sel, y_sel, z_optical], dim=1)
        score_sel = g1 * score1 + g2 * score2

        feasible = torch.logical_or(
            torch.logical_and(use_1, f1), torch.logical_and(use_2, f2)
        )

        ext_max = torch.maximum(extent_h1, extent_h2)
        ext_min = torch.minimum(extent_h1, extent_h2)
        extent_ratio = (ext_max - ext_min) / (ext_max + 1.0e-8)

        return r_optical, score_sel, extent_ratio, feasible, use_1

    def _seed_frame_from_prior(
        self, a_optical: torch.Tensor, z_optical: torch.Tensor
    ) -> torch.Tensor:
        # Degenerate seed: +X from the approach prior, +Y = z x x, +Z = g.
        x_init = a_optical - torch.dot(a_optical, z_optical) * z_optical
        x_init = x_init / (torch.norm(x_init) + 1.0e-8)
        y_init = torch.cross(z_optical, x_init, dim=0)
        y_init = y_init / (torch.norm(y_init) + 1.0e-8)
        return torch.stack([x_init, y_init, z_optical], dim=1)

    def _update_degeneracy_state(
        self, q_lambda: torch.Tensor, extent_ratio: torch.Tensor
    ) -> torch.Tensor:
        # Enter when q_lambda OR extent_ratio below enter; exit only when BOTH recover.
        cur_deg = self._is_degenerate[0]
        below_lambda = q_lambda < self._lambda_deg_thr
        below_extent = extent_ratio < self._extent_deg_thr
        is_obs_deg = torch.logical_or(below_lambda, below_extent)

        recovered_lambda = q_lambda >= self._lambda_rec_thr
        recovered_extent = extent_ratio >= self._extent_rec_thr
        is_obs_rec = torch.logical_and(recovered_lambda, recovered_extent)

        stay_deg = torch.logical_and(cur_deg, torch.logical_not(is_obs_rec))
        new_deg = torch.logical_or(stay_deg, is_obs_deg)
        self._is_degenerate.copy_(new_deg.view(1))
        self._last_axis_quality.copy_(q_lambda.view(1))
        self._last_extent_ratio.copy_(extent_ratio.view(1))
        return new_deg

    def _result_from_cached_state(self) -> dict:
        R_map_target = self._compute_R_map_target_cached()
        approach_map = R_map_target[:, 0]

        width_out = 2.0 * self._extent_cached[1]
        length_app = 2.0 * self._extent_cached[0]
        safe_width_limit = torch.maximum(
            self.gripper_width_limit - self.finger_margin, self._zero_scalar
        )
        feasible_hard = width_out <= self.gripper_width_limit
        feasible_safe = width_out <= safe_width_limit
        width_margin = self.gripper_width_limit - width_out
        safe_width_margin = safe_width_limit - width_out
        budget = torch.maximum(self.gripper_length_limit - self.finger_margin, self._zero_scalar)
        penetration = torch.clamp(
            self.inward_fraction * length_app, min=self._zero_scalar, max=budget
        )
        p_front_shell_map = self._p_map + self._front_shell_offset * approach_map
        p_target_map = p_front_shell_map + penetration * approach_map

        self._T_map_target_buf[:3, :3].copy_(R_map_target)
        self._T_map_target_buf[:3, 3].copy_(p_target_map)

        return {
            "T_map_target": self._T_map_target_buf.clone(),
            "T_optical_target_raw": self._T_optical_target_raw_buf.clone(),
            "pitch": self._zero_pitch.clone(),
            "feasible": feasible_safe.clone(),
            "feasible_safe": feasible_safe.clone(),
            "feasible_hard": feasible_hard.clone(),
            "candidate_feasible": self._last_feasible[0].clone(),
            "measurement_valid": self._last_center_gate[0].clone(),
            "width": width_out.clone(),
            "width_margin": width_margin.clone(),
            "safe_width_margin": safe_width_margin.clone(),
            "gripper_width_limit": self.gripper_width_limit.clone(),
            "effective_gripper_width": safe_width_limit.clone(),
            "depth": length_app.clone(),
            "quality": self._last_quality.clone(),
            "quality_metric": self._last_quality.clone(),
            "axis_quality": self._last_axis_quality.clone(),
            "extent_ratio": self._last_extent_ratio.clone(),
            "is_degenerate": self._is_degenerate.clone(),
            "status_code": self._last_status_code.clone(),
            "reason_code": self._last_reason_code.clone(),
            "output_valid": self._last_output_valid.clone(),
            "stale": self._last_output_stale.clone(),
            "n_points": self._last_n_points.clone(),
            "point_count_raw": self._last_point_count_raw.clone(),
            "point_count_band": self._last_point_count_band.clone(),
            "p_map_raw": self._p_map.clone(),
            "theta_map_raw": self._last_theta_raw.clone(),
            "theta_map": self._theta_map.clone(),
        }

    def _initialize_from_points(
        self,
        pts_optical: torch.Tensor,
        R_mo: torch.Tensor,
        t_mo: torch.Tensor,
        g_optical: torch.Tensor,
        e1_optical: torch.Tensor,
        e2_optical: torch.Tensor,
        a_optical: torch.Tensor,
        approach_ok: torch.Tensor,
    ) -> None:
        # All geometry is in optical frame; the final center/axis are lifted
        # into map at the end of this method.
        n_pts = pts_optical.shape[0]
        p_seed_optical = self._weiszfeld_median_impl(pts_optical)

        h1_optical, h2_optical, _lam_max, _lam_min, q_lambda = (
            self._pca_2d_horizontal_impl(
                pts_optical, p_seed_optical, g_optical, e1_optical, e2_optical,
            )
        )
        centered_optical = pts_optical - p_seed_optical
        q_lo, q_hi = self._project_h_axes_quantiles_impl(
            centered_optical, h1_optical, h2_optical, g_optical,
            self._q_low, self._q_high,
        )
        r_optical_pca, _score_pca, extent_ratio, feasible_pca, _ = (
            self._select_candidate_frame_tensor(
                h1_optical, h2_optical, g_optical, a_optical, q_lo, q_hi,
            )
        )
        is_deg = self._update_degeneracy_state(q_lambda, extent_ratio)

        r_optical_seed = self._seed_frame_from_prior(a_optical, g_optical)
        deg_f = is_deg.to(r_optical_pca.dtype)
        r_optical = deg_f * r_optical_seed + (1.0 - deg_f) * r_optical_pca

        w_close, l_app, front_shell_optical, mids, half_ext = (
            self._extent_midpoint_impl(
                centered_optical, r_optical, self._q_low, self._q_high,
            )
        )
        p_geom_optical = p_seed_optical + torch.matmul(r_optical, mids)

        valid = torch.logical_and(
            torch.isfinite(pts_optical).all(), torch.isfinite(p_geom_optical).all()
        )
        valid = torch.logical_and(valid, self._depth_gate_tensor(pts_optical))
        valid = torch.logical_and(valid, self._extent_gate_tensor(w_close, l_app, half_ext))

        # Single optical -> map lift: ^{map}T_{target} = T_mo @ ^{optical}T_{target}.
        p_geom_map = torch.matmul(R_mo, p_geom_optical) + t_mo
        x_optical = r_optical[:, 0]
        x_map = torch.matmul(R_mo, x_optical)
        theta_raw, h_norm = self._theta_from_x_map(x_map, self.gravity_axis_map)
        lift_ok = h_norm > 1.0e-3
        init_ok = torch.logical_and(torch.logical_and(valid, lift_ok), approach_ok)
        gate_f = init_ok.to(p_geom_map.dtype)

        self._p_map.copy_((1.0 - gate_f) * self._p_map + gate_f * p_geom_map)
        self._theta_map.copy_((1.0 - gate_f) * self._theta_map + gate_f * theta_raw)
        self._extent_cached.copy_((1.0 - gate_f) * self._extent_cached + gate_f * half_ext)
        self._front_shell_offset.copy_(
            (1.0 - gate_f) * self._front_shell_offset + gate_f * front_shell_optical
        )
        self._last_axis_quality.copy_(q_lambda.view(1))
        self._last_extent_ratio.copy_(extent_ratio.view(1))
        self._last_quality.copy_(q_lambda.view(1))
        self._last_feasible.copy_(feasible_pca.view(1))
        self._last_theta_raw.copy_(theta_raw)
        self._frame_count = 0
        self._last_center_gate.copy_(init_ok.view(1))
        self._last_axis_gate.copy_(init_ok.view(1))
        self._has_valid_state.copy_(torch.logical_or(self._has_valid_state, init_ok.view(1)))

        reason_if_invalid = torch.where(
            approach_ok.view(()),
            torch.where(
                valid.view(()),
                self._reason_initial_horizontal_component_missing_t[0],
                self._reason_initial_physical_gate_t[0],
            ),
            self._reason_approach_prior_degenerate_t[0],
        ).view(1)
        self._last_status_code.copy_(
            torch.where(
                init_ok.view(1),
                self._status_fresh_t,
                self._status_rejected_t,
            )
        )
        self._last_reason_code.copy_(
            torch.where(
                init_ok.view(1),
                self._reason_none_t,
                reason_if_invalid,
            )
        )
        self._last_output_valid.copy_(init_ok.view(1))
        self._last_output_stale.copy_(self._false_t)

        self._T_optical_target_raw_buf[:3, :3].copy_(r_optical)
        self._T_optical_target_raw_buf[:3, 3].copy_(p_geom_optical)
        _ = n_pts

    def _track_one_step(
        self,
        pts_optical: torch.Tensor,
        R_mo: torch.Tensor,
        t_mo: torch.Tensor,
        g_optical: torch.Tensor,
        e1_optical: torch.Tensor,
        e2_optical: torch.Tensor,
        a_optical: torch.Tensor,
        approach_ok: torch.Tensor,
    ) -> None:
        n_pts = pts_optical.shape[0]
        self._frame_count += 1

        # Project cached map state back into optical (point transform / rotation).
        p_ema_optical = torch.matmul(R_mo.T, self._p_map - t_mo)
        R_map_target = self._compute_R_map_target_cached()
        R_optical_target = torch.matmul(R_mo.T, R_map_target)

        # Remeasure orientation every frame; degeneracy freezes only the EMA.
        h1_optical, h2_optical, _lam_max, _lam_min, q_lambda_new = (
            self._pca_2d_horizontal_impl(
                pts_optical, p_ema_optical, g_optical, e1_optical, e2_optical,
            )
        )
        centered_ref = pts_optical - p_ema_optical
        q_lo, q_hi = self._project_h_axes_quantiles_impl(
            centered_ref, h1_optical, h2_optical, g_optical,
            self._q_low, self._q_high,
        )
        r_raw_optical, _score_sel, extent_ratio_new, feasible_new, _ = (
            self._select_candidate_frame_tensor(
                h1_optical, h2_optical, g_optical, a_optical, q_lo, q_hi,
            )
        )
        is_deg = self._update_degeneracy_state(q_lambda_new, extent_ratio_new)
        has_state = self._has_valid_state[0]
        deg_f = is_deg.to(r_raw_optical.dtype)
        has_state_f = has_state.to(r_raw_optical.dtype)
        r_measure_optical = (
            has_state_f * (deg_f * R_optical_target + (1.0 - deg_f) * r_raw_optical)
            + (1.0 - has_state_f) * r_raw_optical
        )

        pts_fast = pts_optical
        step = self._subsample_step
        if n_pts >= self._subsample_min * 2:
            step = max(step, 3)
        if step > 1 and n_pts >= self._subsample_min:
            pts_fast = pts_optical[::step]

        centered_fast = pts_fast - p_ema_optical
        w_close, l_app, front_shell_optical, mids, half_ext = (
            self._extent_midpoint_impl(
                centered_fast, r_measure_optical, self._q_low, self._q_high,
            )
        )
        p_geom_optical = p_ema_optical + torch.matmul(r_measure_optical, mids)
        p_geom_map = torch.matmul(R_mo, p_geom_optical) + t_mo

        # Jumps gated in map frame; depth gate stays in optical (sensor) frame.
        center_delta = p_geom_map - self._p_map
        extent_delta = half_ext - self._extent_cached
        center_jump_sq = torch.sum(center_delta * center_delta)
        extent_jump_sq = torch.sum(extent_delta * extent_delta)
        self._last_center_jump_sq.copy_(center_jump_sq.view(1))
        self._last_extent_jump_sq.copy_(extent_jump_sq.view(1))

        jump_ok = center_jump_sq <= self._max_center_jump_sq_t
        extent_ok = extent_jump_sq <= self._max_extent_jump_sq_t
        finite = torch.logical_and(
            torch.isfinite(pts_optical).all(), torch.isfinite(p_geom_map).all()
        )
        physical = torch.logical_and(
            self._depth_gate_tensor(pts_optical),
            self._extent_gate_tensor(w_close, l_app, half_ext),
        )
        center_gate = torch.logical_and(
            torch.logical_and(finite, physical),
            torch.logical_and(jump_ok, extent_ok),
        )
        center_gate_effective = torch.logical_and(center_gate, approach_ok)

        # Rate-limited Euclidean EMA on R^3 (map-frame center).
        gate_f = center_gate_effective.to(p_geom_map.dtype)
        delta_norm = torch.norm(center_delta)
        step_scale = torch.clamp(self._max_center_step_t / (delta_norm + 1.0e-8), max=1.0)
        p_rate_limited = self._p_map + step_scale * center_delta

        has_state_f = has_state.to(p_geom_map.dtype)
        alpha_c = gate_f * (has_state_f * self._ema_alpha + (1.0 - has_state_f))
        p_source = has_state_f * p_rate_limited + (1.0 - has_state_f) * p_geom_map
        e_source = has_state_f * half_ext + (1.0 - has_state_f) * half_ext
        p_new = (1.0 - alpha_c) * self._p_map + alpha_c * p_source
        e_new = (1.0 - alpha_c) * self._extent_cached + alpha_c * e_source
        shell_new = (
            (1.0 - alpha_c) * self._front_shell_offset + alpha_c * front_shell_optical
        )
        self._p_map.copy_(p_new)
        self._extent_cached.copy_(e_new)
        self._front_shell_offset.copy_(shell_new)
        self._last_center_gate.copy_(center_gate_effective.view(1))

        # S^1 EMA on map-frame yaw; frozen under degeneracy or gate failure.
        x_raw_map = torch.matmul(R_mo, r_raw_optical[:, 0])
        theta_raw, h_norm = self._theta_from_x_map(x_raw_map, self.gravity_axis_map)
        dtheta_raw = self._wrap_angle(theta_raw - self._theta_map)
        energy_ok = h_norm > 1.0e-3
        pass_quality = q_lambda_new >= self._quality_thr
        pass_trust = torch.abs(dtheta_raw) <= self._dtheta_max
        not_degenerate = torch.logical_not(is_deg)
        axis_gate = torch.logical_and(
            torch.logical_and(energy_ok, pass_quality), pass_trust
        )
        axis_gate = torch.logical_and(axis_gate, center_gate_effective)
        axis_gate = torch.logical_and(axis_gate, not_degenerate)

        axis_gate_f = axis_gate.to(self._axis_alpha_t.dtype)
        theta_track = self._wrap_angle(self._theta_map + self._axis_alpha_t * axis_gate_f * dtheta_raw)
        theta_new = has_state_f * theta_track + (1.0 - has_state_f) * theta_raw
        self._theta_map.copy_(theta_new)
        self._last_theta_raw.copy_(theta_raw)
        self._last_quality.copy_(q_lambda_new.view(1))
        self._last_feasible.copy_(feasible_new.view(1))
        self._last_axis_gate.copy_(axis_gate.view(1))
        self._T_optical_target_raw_buf[:3, :3].copy_(r_measure_optical)
        self._T_optical_target_raw_buf[:3, 3].copy_(p_geom_optical)
        self._has_valid_state.copy_(
            torch.logical_or(self._has_valid_state, center_gate_effective.view(1))
        )

        held_vs_rejected = torch.where(
            has_state.view(1),
            self._status_held_t,
            self._status_rejected_t,
        )
        status_false = held_vs_rejected
        reason_false = self._reason_tracking_physical_gate_t
        reason_false = torch.where(
            approach_ok.view(1),
            reason_false,
            self._reason_approach_prior_degenerate_t,
        )
        stale_false = has_state.view(1)
        self._last_status_code.copy_(
            torch.where(
                center_gate_effective.view(1),
                self._status_fresh_t,
                status_false,
            )
        )
        self._last_reason_code.copy_(
            torch.where(
                center_gate_effective.view(1),
                self._reason_none_t,
                reason_false,
            )
        )
        self._last_output_valid.copy_(
            torch.where(
                center_gate_effective.view(1),
                self._true_t,
                has_state.view(1),
            )
        )
        self._last_output_stale.copy_(
            torch.where(
                center_gate_effective.view(1),
                self._false_t,
                stale_false,
            )
        )

    @torch.inference_mode()
    def run_full_pipeline(
        self,
        roi_rgb: torch.Tensor,
        roi_depth: torch.Tensor,
        roi_mask: torch.Tensor,
        roi_x_base: torch.Tensor,
        roi_y_base: torch.Tensor,
        T_map_optical: torch.Tensor,
    ) -> dict:
        # T_map_optical: numeric 4x4 ^{map}T_{optical}, supplied by upstream.
        for name, tensor in (
            ("roi_rgb", roi_rgb),
            ("roi_depth", roi_depth),
            ("roi_mask", roi_mask),
            ("roi_x_base", roi_x_base),
            ("roi_y_base", roi_y_base),
        ):
            if not isinstance(tensor, torch.Tensor):
                raise TypeError(f"{name} must be a torch.Tensor")
            if not self._device_matches(tensor.device):
                raise ValueError(
                    f"{name} must already live on {self.device}, got {tensor.device}"
                )
        if roi_rgb.ndim != 3:
            raise ValueError(f"roi_rgb must have shape (3, H, W), got {tuple(roi_rgb.shape)}")
        if roi_depth.ndim != 2:
            raise ValueError(f"roi_depth must have shape (H, W), got {tuple(roi_depth.shape)}")
        if roi_mask.ndim != 2:
            raise ValueError(f"roi_mask must have shape (H, W), got {tuple(roi_mask.shape)}")
        if roi_x_base.ndim != 2:
            raise ValueError(f"roi_x_base must have shape (H, W), got {tuple(roi_x_base.shape)}")
        if roi_y_base.ndim != 2:
            raise ValueError(f"roi_y_base must have shape (H, W), got {tuple(roi_y_base.shape)}")
        if roi_rgb.shape[0] != 3:
            raise ValueError(f"roi_rgb channel dimension must be 3, got {roi_rgb.shape[0]}")
        if tuple(roi_rgb.shape[1:]) != tuple(roi_depth.shape):
            raise ValueError(
                f"roi_rgb spatial shape {tuple(roi_rgb.shape[1:])} must match "
                f"roi_depth {tuple(roi_depth.shape)}"
            )
        if tuple(roi_mask.shape) != tuple(roi_depth.shape):
            raise ValueError(
                f"roi_mask shape {tuple(roi_mask.shape)} must match "
                f"roi_depth {tuple(roi_depth.shape)}"
            )
        if tuple(roi_x_base.shape) != tuple(roi_depth.shape):
            raise ValueError(
                f"roi_x_base shape {tuple(roi_x_base.shape)} must match "
                f"roi_depth {tuple(roi_depth.shape)}"
            )
        if tuple(roi_y_base.shape) != tuple(roi_depth.shape):
            raise ValueError(
                f"roi_y_base shape {tuple(roi_y_base.shape)} must match "
                f"roi_depth {tuple(roi_depth.shape)}"
            )
        if not isinstance(T_map_optical, torch.Tensor):
            raise TypeError(
                "T_map_optical must be a torch.Tensor (numeric 4x4 matrix)"
            )
        if not self._device_matches(T_map_optical.device):
            raise ValueError(
                f"T_map_optical must already live on {self.device}, got "
                f"{T_map_optical.device}"
            )
        if tuple(T_map_optical.shape) != (4, 4):
            raise ValueError(
                f"T_map_optical must have shape (4, 4), got {tuple(T_map_optical.shape)}"
            )
        if T_map_optical.dtype != torch.float32:
            raise ValueError(
                f"T_map_optical must be float32 to avoid hidden casts, got {T_map_optical.dtype}"
            )
        T_mo = T_map_optical
        R_mo = T_mo[:3, :3]
        t_mo = T_mo[:3, 3]

        # Rotate gravity (direction-only) into optical: g_optical = R_mo^T g_map.
        g_optical = torch.matmul(R_mo.T, self.gravity_axis_map)
        g_optical = g_optical / (torch.norm(g_optical) + 1.0e-8)
        e1_optical, e2_optical = self._plane_basis_impl(g_optical)
        a_optical, approach_ok = self._compute_approach_prior_optical(
            g_optical, e1_optical
        )

        depth_d = self._guided_filter_impl(
            roi_rgb, roi_depth, self.gf_radius, self.gf_k_size, self.gf_eps
        )
        pts_optical, raw_count, band_count = self._mask_guided_backprojection_impl(
            depth_d, roi_mask, roi_x_base, roi_y_base, self.depth_band_m,
        )
        n_points = int(pts_optical.shape[0])
        self._set_point_counts(raw_count, band_count, n_points)

        if n_points < self._min_valid_points:
            self._last_status_code.copy_(
                torch.where(
                    self._has_valid_state,
                    self._status_held_t,
                    self._status_rejected_t,
                )
            )
            self._last_reason_code.copy_(self._reason_insufficient_points_t)
            self._last_output_valid.copy_(self._has_valid_state)
            self._last_output_stale.copy_(self._has_valid_state)
            return self._result_from_cached_state()

        if not self._axes_initialized:
            self._initialize_from_points(
                pts_optical, R_mo, t_mo, g_optical,
                e1_optical, e2_optical, a_optical, approach_ok,
            )
            self._axes_initialized = True
        else:
            self._track_one_step(
                pts_optical, R_mo, t_mo, g_optical,
                e1_optical, e2_optical, a_optical, approach_ok,
            )

        return self._result_from_cached_state()


if __name__ == "__main__":
    import os
    import sys
    import threading
    import time

    import numpy as np
    import rclpy
    from message_filters import ApproximateTimeSynchronizer, Subscriber
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )
    from sensor_msgs.msg import CameraInfo, Image

    script_dir = os.path.dirname(os.path.abspath(__file__))
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)
    from perception_module import PerceptionModule

    parser = argparse.ArgumentParser(
        description="Standalone DecisionModule runner (raw=optical, state=map)."
    )
    parser.add_argument("--live", default=True)
    parser.add_argument("--query", type=str, default="a black tumbler")
    parser.add_argument("--score-threshold", type=float, default=0.10)
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--camera-ns", type=str, default="gripper_camera")
    parser.add_argument("--camera-name", type=str, default="gripper_camera")
    parser.add_argument("--sync-slop", type=float, default=0.01)
    parser.add_argument("--sync-queue", type=int, default=5)
    parser.add_argument("--executor-threads", type=int, default=2)
    parser.add_argument("--ema-alpha", type=float, default=0.20)
    parser.add_argument("--axis-alpha", type=float, default=0.20)
    parser.add_argument("--quality-thr", type=float, default=0.15)
    parser.add_argument("--dtheta-max", type=float, default=0.35)
    parser.add_argument("--depth-band-m", type=float, default=0.08)
    parser.add_argument("--min-valid-points", type=int, default=80)
    parser.add_argument("--gripper-width", type=float, default=0.080)
    parser.add_argument("--gripper-length", type=float, default=0.04)
    parser.add_argument("--finger-margin", type=float, default=0.0)
    parser.add_argument("--inward-fraction", type=float, default=0.25)
    parser.add_argument("--prefer-wide-approach", action="store_true")
    parser.add_argument("--lambda-degenerate-thr", type=float, default=0.10)
    parser.add_argument("--lambda-recover-thr", type=float, default=0.15)
    parser.add_argument("--extent-degenerate-thr", type=float, default=0.10)
    parser.add_argument("--extent-recover-thr", type=float, default=0.15)
    parser.add_argument("--min-approach-prior-norm", type=float, default=0.05)
    parser.add_argument("--warmup-iters", type=int, default=3)
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--no-sync-timing", action="store_true")
    args = parser.parse_args()

    if not args.live:
        print("decision_module3.py is ready.")
        raise SystemExit(0)

    def image_msg_to_numpy(msg):
        enc = msg.encoding.lower()
        if enc in ("rgb8", "bgr8"):
            dtype = np.uint8
            channels = 3
        elif enc in ("mono8", "8uc1"):
            dtype = np.uint8
            channels = 1
        elif enc in ("mono16", "16uc1"):
            dtype = np.uint16
            channels = 1
        elif enc == "32fc1":
            dtype = np.float32
            channels = 1
        else:
            raise RuntimeError(f"unsupported image encoding: {msg.encoding}")

        itemsize = np.dtype(dtype).itemsize
        row_elems = msg.step // itemsize
        raw = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            arr = raw.reshape(msg.height, row_elems)[:, : msg.width]
            if enc == "32fc1":
                arr = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
                arr = np.clip(arr * 1000.0, 0.0, 65535.0).astype(np.uint16)
        else:
            row_pixels = row_elems // channels
            arr = raw.reshape(msg.height, row_pixels, channels)[:, : msg.width, :]
        return np.ascontiguousarray(arr)

    rclpy.init()
    node = rclpy.create_node("decision_module_standalone")
    callback_group = ReentrantCallbackGroup()
    lock = threading.Lock()
    frame_ready = threading.Event()
    state = {
        "rgb": None,
        "depth": None,
        "K": None,
        "is_bgr": False,
        "stamp": 0.0,
    }

    def info_callback(msg):
        with lock:
            if state["K"] is None:
                state["K"] = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def sync_callback(rgb_msg, depth_msg):
        rgb_np = image_msg_to_numpy(rgb_msg)
        depth_np = image_msg_to_numpy(depth_msg)
        stamp = rgb_msg.header.stamp.sec + 1.0e-9 * rgb_msg.header.stamp.nanosec
        with lock:
            state["rgb"] = rgb_np
            state["depth"] = depth_np
            state["is_bgr"] = rgb_msg.encoding.lower().startswith("bgr")
            state["stamp"] = stamp
            frame_ready.set()

    def wait_frame(timeout):
        if not frame_ready.wait(timeout=timeout):
            return None
        with lock:
            if state["rgb"] is None or state["depth"] is None or state["K"] is None:
                frame_ready.clear()
                return None
            frame_ready.clear()
            return {
                "rgb": state["rgb"],
                "depth": state["depth"],
                "K": state["K"],
                "is_bgr": state["is_bgr"],
                "stamp": state["stamp"],
            }

    prefix = f"/{args.camera_ns}/{args.camera_name}"
    rgb_topic = f"{prefix}/color/image_raw"
    depth_topic = f"{prefix}/aligned_depth_to_color/image_raw"
    info_topic = f"{prefix}/aligned_depth_to_color/camera_info"
    sensor_qos = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )

    info_sub = node.create_subscription(
        CameraInfo, info_topic, info_callback, sensor_qos, callback_group=callback_group
    )
    rgb_sub = Subscriber(
        node, Image, rgb_topic, qos_profile=sensor_qos, callback_group=callback_group
    )
    depth_sub = Subscriber(
        node, Image, depth_topic, qos_profile=sensor_qos, callback_group=callback_group
    )
    sync = ApproximateTimeSynchronizer(
        [rgb_sub, depth_sub], queue_size=args.sync_queue, slop=args.sync_slop
    )
    sync.registerCallback(sync_callback)

    executor = MultiThreadedExecutor(num_threads=max(1, args.executor_threads))
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print("Waiting for synchronized RGB-D frame...")
        first_frame = None
        t_wait0 = time.perf_counter()
        while first_frame is None:
            if time.perf_counter() - t_wait0 > 30.0:
                raise RuntimeError("camera data was not ready within 30 seconds")
            first_frame = wait_frame(timeout=0.5)

        K = first_frame["K"]
        decision = DecisionModule(
            device=args.device,
            fx=float(K[0, 0]),
            fy=float(K[1, 1]),
            cx=float(K[0, 2]),
            cy=float(K[1, 2]),
            ema_alpha=args.ema_alpha,
            axis_alpha=args.axis_alpha,
            quality_thr=args.quality_thr,
            dtheta_max=args.dtheta_max,
            gripper_width_limit=args.gripper_width,
            gripper_length_limit=args.gripper_length,
            finger_margin=args.finger_margin,
            inward_fraction=args.inward_fraction,
            depth_band_m=args.depth_band_m,
            min_valid_points=args.min_valid_points,
            prefer_short_approach=not args.prefer_wide_approach,
            lambda_degenerate_thr=args.lambda_degenerate_thr,
            lambda_recover_thr=args.lambda_recover_thr,
            extent_degenerate_thr=args.extent_degenerate_thr,
            extent_recover_thr=args.extent_recover_thr,
            min_approach_prior_norm=args.min_approach_prior_norm,
        )

        weights = os.path.abspath(os.path.join(script_dir, "..", "weights"))
        perception = PerceptionModule(
            owl_model_name="google/owlvit-base-patch32",
            owl_image_engine=os.path.join(weights, "owlvit", "tensorrt", "32_image_encoder_fp16.engine"),
            owl_text_engine=os.path.join(weights, "owlvit", "tensorrt", "32_text_encoder_fp32.engine"),
            sam_encoder_engine=os.path.join(weights, "nanosam", "tensorrt", "encoder_fp16.engine"),
            sam_decoder_engine=os.path.join(weights, "nanosam", "tensorrt", "decoder_fp16.engine"),
            device=args.device,
            score_threshold=args.score_threshold,
        )
        perception.set_text_query([args.query])
        perception.setup(first_frame["rgb"])

        device = torch.device(args.device)
        h_img, w_img = first_frame["depth"].shape
        v = torch.arange(h_img, device=device, dtype=torch.float32)
        u = torch.arange(w_img, device=device, dtype=torch.float32)
        vv, uu = torch.meshgrid(v, u, indexing="ij")
        full_x_base_buf = (uu - float(K[0, 2])) / float(K[0, 0])
        full_y_base_buf = (vv - float(K[1, 2])) / float(K[1, 1])
        rgb_float_buf = torch.empty((3, h_img, w_img), dtype=torch.float32, device=device)
        depth_float_buf = torch.empty((1, h_img, w_img), dtype=torch.float32, device=device)
        mask_float_buf = torch.empty((h_img, w_img), dtype=torch.float32, device=device)
        status_names = {
            DecisionModule._STATUS_RESET: "reset",
            DecisionModule._STATUS_REJECTED: "rejected",
            DecisionModule._STATUS_HELD: "held",
            DecisionModule._STATUS_FRESH: "fresh",
        }
        reason_names = {
            DecisionModule._REASON_NONE: "none",
            DecisionModule._REASON_INSUFFICIENT_POINTS: "insufficient_points",
            DecisionModule._REASON_APPROACH_PRIOR_DEGENERATE: "approach_prior_degenerate",
            DecisionModule._REASON_INITIAL_PHYSICAL_GATE: "initial_physical_gate",
            DecisionModule._REASON_INITIAL_HORIZONTAL_COMPONENT_MISSING: (
                "initial_horizontal_component_missing"
            ),
            DecisionModule._REASON_TRACKING_PHYSICAL_GATE: "tracking_physical_gate",
        }

        def prepare_roi_ready_tensors(bbox_tensor):
            x0, y0, x1, y1 = bbox_tensor.long().detach().cpu().tolist()
            m = decision.crop_margin
            x0 = max(min(x0 - m, w_img - 1), 0)
            y0 = max(min(y0 - m, h_img - 1), 0)
            x1 = min(max(x1 + m, x0 + 1), w_img)
            y1 = min(max(y1 + m, y0 + 1), h_img)
            return (
                rgb_float_buf[:, y0:y1, x0:x1],
                depth_float_buf[0, y0:y1, x0:x1],
                mask_float_buf[y0:y1, x0:x1],
                full_x_base_buf[y0:y1, x0:x1],
                full_y_base_buf[y0:y1, x0:x1],
            )

        # Standalone ^{map}T_{optical} (orientation-only validation).
        T_map_optical_standalone = torch.tensor(
            [
                [0.0, 0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=torch.float32,
            device=device,
        )

        print("Warmup...")
        for _ in range(max(0, args.warmup_iters)):
            frame = wait_frame(timeout=1.0)
            if frame is None:
                continue
            result = perception.infer_tensor(
                frame["rgb"], depth_data=frame["depth"], is_bgr=frame["is_bgr"]
            )
            result.event.synchronize()
            if bool(result.valid.item()):
                rgb_float_buf.copy_(perception.gpu_raw_image).mul_(1.0 / 255.0)
                depth_float_buf[0].copy_(perception.gpu_depth_image).mul_(0.001)
                mask_float_buf.copy_(result.mask)
                roi_rgb, roi_depth, roi_mask, roi_x_base, roi_y_base = (
                    prepare_roi_ready_tensors(result.bbox)
                )
                _ = decision.run_full_pipeline(
                    roi_rgb,
                    roi_depth,
                    roi_mask,
                    roi_x_base,
                    roi_y_base,
                    T_map_optical=T_map_optical_standalone,
                )
                if device.type == "cuda":
                    torch.cuda.synchronize()
        decision.reset()
        frame_ready.clear()

        print("Live decision loop started (thin boundary + ROI-ready decision core).")
        processed = 0
        valid_det = 0
        fresh_grasp = 0
        held_grasp = 0
        rejected_grasp = 0
        infer_time = 0.0
        boundary_time = 0.0
        decision_core_time = 0.0
        wait_time = 0.0
        latest_grasp = None
        latest_status = "none"
        latest_reason = ""
        last_print = time.perf_counter()
        last_processed = 0
        last_valid_det = 0
        last_fresh_grasp = 0
        last_held_grasp = 0
        last_rejected_grasp = 0
        last_infer_time = 0.0
        last_boundary_time = 0.0
        last_decision_core_time = 0.0
        last_wait_time = 0.0

        while rclpy.ok():
            if args.max_frames > 0 and processed >= args.max_frames:
                break

            wait_t0 = time.perf_counter()
            frame = wait_frame(timeout=0.2)
            wait_t1 = time.perf_counter()
            wait_time += wait_t1 - wait_t0
            if frame is None:
                continue

            t0 = time.perf_counter()
            perception_result = perception.infer_tensor(
                frame["rgb"], depth_data=frame["depth"], is_bgr=frame["is_bgr"]
            )
            perception_result.event.synchronize()
            t1 = time.perf_counter()
            infer_time += t1 - t0
            processed += 1

            if bool(perception_result.valid.item()):
                valid_det += 1
                rgb_float_buf.copy_(perception.gpu_raw_image).mul_(1.0 / 255.0)
                depth_float_buf[0].copy_(perception.gpu_depth_image).mul_(0.001)
                mask_float_buf.copy_(perception_result.mask)
                b0 = time.perf_counter()
                roi_rgb, roi_depth, roi_mask, roi_x_base, roi_y_base = (
                    prepare_roi_ready_tensors(perception_result.bbox)
                )
                b1 = time.perf_counter()
                boundary_time += b1 - b0
                d0 = time.perf_counter()
                grasp = decision.run_full_pipeline(
                    roi_rgb,
                    roi_depth,
                    roi_mask,
                    roi_x_base,
                    roi_y_base,
                    T_map_optical=T_map_optical_standalone,
                )
                if device.type == "cuda" and not args.no_sync_timing:
                    torch.cuda.synchronize()
                d1 = time.perf_counter()
                decision_core_time += d1 - d0
                status_code = int(grasp["status_code"].item())
                reason_code = int(grasp["reason_code"].item())
                if status_code == DecisionModule._STATUS_FRESH:
                    fresh_grasp += 1
                elif status_code == DecisionModule._STATUS_HELD:
                    held_grasp += 1
                elif status_code == DecisionModule._STATUS_REJECTED:
                    rejected_grasp += 1
                latest_grasp = grasp
                latest_status = status_names.get(status_code, f"unknown({status_code})")
                latest_reason = reason_names.get(reason_code, f"unknown({reason_code})")

            now = time.perf_counter()
            if now - last_print >= args.print_every:
                dt = now - last_print
                dp = processed - last_processed
                dd = valid_det - last_valid_det
                df = fresh_grasp - last_fresh_grasp
                dh = held_grasp - last_held_grasp
                dr = rejected_grasp - last_rejected_grasp
                di = infer_time - last_infer_time
                db = boundary_time - last_boundary_time
                ddc = decision_core_time - last_decision_core_time
                dwait = wait_time - last_wait_time
                inf_ms = (di / max(dp, 1)) * 1000.0
                boundary_ms = (db / max(dd, 1)) * 1000.0
                dec_core_ms = (ddc / max(dd, 1)) * 1000.0
                dec_total_ms = boundary_ms + dec_core_ms
                wait_ms = (dwait / max(dp, 1)) * 1000.0

                if latest_grasp is not None:
                    p_map = latest_grasp["p_map_raw"].detach().cpu().tolist()
                    p_target = latest_grasp["T_map_target"][:3, 3].detach().cpu().tolist()
                    center_jump_mm = torch.sqrt(decision._last_center_jump_sq[0]).item() * 1e3
                    extent_jump_mm = torch.sqrt(decision._last_extent_jump_sq[0]).item() * 1e3
                    z_median_m = decision._last_z_median[0].item()
                    latest_summary = (
                        f"{latest_status} p_map=[{p_map[0]:.3f}, {p_map[1]:.3f}, {p_map[2]:.3f}] "
                        f"p_target=[{p_target[0]:.3f}, {p_target[1]:.3f}, {p_target[2]:.3f}] "
                        f"width={latest_grasp['width'].item() * 1e3:.1f}mm "
                        f"depth={latest_grasp['depth'].item() * 1e3:.1f}mm "
                        f"q={latest_grasp['quality_metric'].item():.3f} "
                        f"axisQ={latest_grasp['axis_quality'].item():.3f} "
                        f"er={latest_grasp['extent_ratio'].item():.3f} "
                        f"deg={bool(latest_grasp['is_degenerate'].item())} "
                        f"feasible={bool(latest_grasp['feasible'].item())} "
                        f"safe_margin={latest_grasp['safe_width_margin'].item() * 1e3:.1f}mm "
                        f"meas={bool(latest_grasp['measurement_valid'].item())} "
                        f"cj={center_jump_mm:.1f}mm "
                        f"ej={extent_jump_mm:.1f}mm "
                        f"zmed={z_median_m:.3f}m "
                        f"pts(raw/band)={int(latest_grasp['point_count_raw'].item())}/"
                        f"{int(latest_grasp['point_count_band'].item())} "
                        f"reason={latest_reason or 'none'}"
                    )
                else:
                    latest_summary = f"{latest_status} reason={latest_reason or 'unknown'}"

                print(
                    f"Loop {dp / max(dt, 1.0e-9):.1f} Hz | "
                    f"det {dd}/{dp} | fresh {df}/{max(dd, 1)} | held {dh} | rejected {dr} | "
                    f"infer {inf_ms:.2f} ms | boundary {boundary_ms:.2f} ms | "
                    f"decision_core {dec_core_ms:.2f} ms | decision {dec_total_ms:.2f} ms | "
                    f"wait {wait_ms:.2f} ms | {latest_summary}"
                )
                last_print = now
                last_processed = processed
                last_valid_det = valid_det
                last_fresh_grasp = fresh_grasp
                last_held_grasp = held_grasp
                last_rejected_grasp = rejected_grasp
                last_infer_time = infer_time
                last_boundary_time = boundary_time
                last_decision_core_time = decision_core_time
                last_wait_time = wait_time

        raise SystemExit(0)
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
