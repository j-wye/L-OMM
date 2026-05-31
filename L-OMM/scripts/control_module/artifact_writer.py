#!/usr/bin/env python3
"""CSV/JSON/PNG artifact writer for planner + reference + DLS rollout."""
from __future__ import annotations

import csv
import json
import os
from typing import Any, Dict, Optional

import numpy as np

from episode_result import EpisodeResult
from plan_result import PlanResult


class ArtifactWriter:
    """Side-effecting writer; owns no algorithms."""

    def __init__(self, make_plots: bool = True) -> None:
        self.make_plots = bool(make_plots)

    def write_case(self,
                   out_dir: str,
                   plan: PlanResult,
                   reference,
                   episode: Optional[EpisodeResult],
                   summary: Dict[str, Any],
                   image_root: Optional[str] = None,
                   image_prefix: Optional[str] = None) -> None:
        os.makedirs(out_dir, exist_ok=True)
        self.write_path_csv(os.path.join(out_dir, "path.csv"), plan)
        self.write_reference_csv(os.path.join(out_dir, "reference.csv"), reference)
        if episode is not None:
            self.write_episode_csv(os.path.join(out_dir, "episode.csv"), episode)
        else:
            stale_episode = os.path.join(out_dir, "episode.csv")
            if os.path.exists(stale_episode):
                os.remove(stale_episode)
        self.write_json(os.path.join(out_dir, "summary.json"), summary)
        if self.make_plots:
            img_dir = image_root if image_root is not None else os.path.join(out_dir, "img")
            prefix = image_prefix or os.path.basename(os.path.normpath(out_dir))
            self.write_path_png(os.path.join(img_dir, f"{prefix}_path.png"), plan)
            if episode is not None:
                self.write_episode_png(os.path.join(img_dir, f"{prefix}_episode.png"), episode)
            else:
                stale_png = os.path.join(img_dir, f"{prefix}_episode.png")
                if os.path.exists(stale_png):
                    os.remove(stale_png)

    def write_path_csv(self, path: str, plan: PlanResult) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["k", "ix", "iz", "x", "z", "pitch_ref_rad", "mu_g"])
            for k, (ix, iz) in enumerate(plan.cells):
                w.writerow([
                    k, int(ix), int(iz),
                    float(plan.xz[k, 0]), float(plan.xz[k, 1]),
                    float(plan.pitch[k]),
                    float(plan.map_handle.mu_grid[int(ix), int(iz)]),
                ])

    def write_reference_csv(self, path: str, reference) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        s, xz, pitch = reference.table_arrays()
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["k", "s", "x", "z", "pitch_ref_rad"])
            for k in range(xz.shape[0]):
                w.writerow([k, float(s[k]), float(xz[k, 0]), float(xz[k, 1]), float(pitch[k])])

    def write_episode_csv(self, path: str, ep: EpisodeResult) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        n = ep.q_traj.shape[0]
        # Spec §12.2 — diagnostic columns: sdot, ||r_dot_ref||, dpitch/ds,
        # pitch feedforward, plus the xz components of r_dot_ref so the user
        # can plot ṙ_ref(t) directly without re-deriving it from ṡ.
        sdot_arr = ep.sdot_hist if ep.sdot_hist is not None else np.zeros(n)
        rdot_norm_arr = ep.rdot_norm_hist if ep.rdot_norm_hist is not None else np.zeros(n)
        dp_arr = ep.dpitch_ds_hist if ep.dpitch_ds_hist is not None else np.zeros(n)
        gap_arr = ep.gap_norm_hist if ep.gap_norm_hist is not None else np.zeros(n)
        e_along_arr = ep.e_along_hist if ep.e_along_hist is not None else np.zeros(n)
        rdot_arr = ep.r_dot_ref_hist if ep.r_dot_ref_hist is not None else np.zeros((n, 3))
        with open(path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            header = ["k", "s", "wp", "ee_x", "ee_y", "ee_z", "e_pos_norm", "e_rot_norm",
                      "sigma_min", "lambda", "qdot_norm",
                      "sdot", "rdot_norm", "dpitch_ds", "pitch_feedforward",
                      "rdot_x", "rdot_z", "gap_norm", "e_along"]
            active_labels = ["q2", "q3", "q5"]
            header.extend(active_labels[:ep.q_traj.shape[1]])
            w.writerow(header)
            for k in range(n):
                row = [
                    k,
                    float(ep.s_hist[k]),
                    int(ep.wp_index[k]),
                    float(ep.ee_traj[k, 0]),
                    float(ep.ee_traj[k, 1]),
                    float(ep.ee_traj[k, 2]),
                    float(np.linalg.norm(ep.e_pos_hist[k])),
                    float(np.linalg.norm(ep.e_rot_hist[k])),
                    float(ep.sigma_min_hist[k]),
                    float(ep.lambda_hist[k]),
                    float(np.linalg.norm(ep.qdot_traj[k])),
                    float(sdot_arr[k]) if k < sdot_arr.size else 0.0,
                    float(rdot_norm_arr[k]) if k < rdot_norm_arr.size else 0.0,
                    float(dp_arr[k]) if k < dp_arr.size else 0.0,
                    float(rdot_arr[k, 2]) if k < rdot_arr.shape[0] else 0.0,
                    float(rdot_arr[k, 0]) if k < rdot_arr.shape[0] else 0.0,
                    float(rdot_arr[k, 1]) if k < rdot_arr.shape[0] else 0.0,
                    float(gap_arr[k]) if k < gap_arr.size else 0.0,
                    float(e_along_arr[k]) if k < e_along_arr.size else 0.0,
                ]
                row.extend([float(v) for v in ep.q_traj[k]])
                w.writerow(row)

    def write_path_png(self, path: str, plan: PlanResult) -> Optional[str]:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            from matplotlib.patches import Rectangle
        except Exception:
            return None
        try:
            handle = plan.map_handle
            extent = [
                handle.x0,
                handle.x0 + (handle.shape[0] - 1) * handle.resolution_m,
                handle.z0,
                handle.z0 + (handle.shape[1] - 1) * handle.resolution_m,
            ]
            width = max(extent[1] - extent[0], 1.0e-9)
            height = max(extent[3] - extent[2], 1.0e-9)
            fig_w = 6.0
            fig_h = fig_w * height / width
            fig, ax = plt.subplots(figsize=(fig_w, fig_h), dpi=140)
            img = np.ma.masked_where(~plan.feasible_mask.T, handle.mu_grid.T)
            ax.imshow(img, origin="lower", extent=extent, aspect="equal", cmap="viridis")
            if plan.xz.size:
                ax.plot(plan.xz[:, 0], plan.xz[:, 1], "w-", lw=2.0)
                ax.plot(plan.xz[0, 0], plan.xz[0, 1], "go", ms=5)
                ax.plot(plan.xz[-1, 0], plan.xz[-1, 1], "ro", ms=5)
            for xl, zt, xr, zb in plan.blocked_rects:
                ax.add_patch(Rectangle((xl, zb), xr - xl, zt - zb, fill=False, edgecolor="orange", lw=2.0))
            ax.set_xlabel("x [m]")
            ax.set_ylabel("z [m]")
            ax.set_title("Active-map A* skeleton")
            ax.set_aspect("equal", adjustable="box")
            fig.tight_layout()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            fig.savefig(path)
            plt.close(fig)
            return path
        except Exception:
            return None

    def write_episode_png(self, path: str, ep: EpisodeResult) -> Optional[str]:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:
            return None
        try:
            t = np.arange(ep.q_traj.shape[0], dtype=np.float64)
            # Spec §12.2 — diagnostic plot strengthened: pos err, rot err,
            # ||qdot||, sdot+||r_dot_ref|| overlay so the reviewer can see at a
            # glance where the time envelope and the actual feedforward live.
            fig, axes = plt.subplots(4, 1, figsize=(8.0, 9.0), dpi=140, sharex=True)
            axes[0].plot(t, np.linalg.norm(ep.e_pos_hist, axis=1) * 1000.0)
            axes[0].set_ylabel("pos err [mm]")
            axes[1].plot(t, np.degrees(np.linalg.norm(ep.e_rot_hist, axis=1)))
            axes[1].set_ylabel("rot err [deg]")
            qdot = np.linalg.norm(ep.qdot_traj, axis=1)
            axes[2].plot(t, qdot, label="||qdot||")
            for label, val, tk in [("init", ep.peak_qdot_initial, ep.peak_qdot_initial_tick),
                                    ("mid", ep.peak_qdot_mid, ep.peak_qdot_mid_tick),
                                    ("term", ep.peak_qdot_terminal, ep.peak_qdot_terminal_tick)]:
                if val > 0:
                    axes[2].axvline(tk, color="0.7", linestyle=":", linewidth=0.8)
                    axes[2].text(tk, val, f"{label}={val:.2f}", fontsize=7)
            axes[2].set_ylabel("||qdot|| [rad/s]")
            axes[2].legend(loc="best", fontsize=7)
            sdot = ep.sdot_hist if ep.sdot_hist is not None else np.zeros_like(t)
            rnorm = ep.rdot_norm_hist if ep.rdot_norm_hist is not None else np.zeros_like(t)
            axes[3].plot(t, sdot, label="sdot [m/s]")
            axes[3].plot(t, rnorm, label="||r_dot_ref|| [m/s]", linestyle="--")
            if ep.gap_norm_hist is not None and ep.gap_norm_hist.size == t.size:
                axes[3].plot(t, ep.gap_norm_hist, label="gap [m]", linestyle=":")
            axes[3].set_ylabel("envelope")
            axes[3].set_xlabel("tick")
            axes[3].legend(loc="best", fontsize=7)
            fig.tight_layout()
            os.makedirs(os.path.dirname(path), exist_ok=True)
            fig.savefig(path)
            plt.close(fig)
            return path
        except Exception:
            return None

    def write_json(self, path: str, data: Dict[str, Any]) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self._jsonable(data), f, indent=2, ensure_ascii=False)

    def _jsonable(self, value: Any) -> Any:
        if isinstance(value, dict):
            return {str(k): self._jsonable(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [self._jsonable(v) for v in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if hasattr(value, "name") and hasattr(value, "value"):
            return value.name
        return value
