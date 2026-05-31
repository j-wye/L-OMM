#!/usr/bin/env python3
"""Integrated Camera -> Perception -> Decision pipeline for L-OMM."""

import argparse
import os
import sys
import threading
import time
from typing import Optional, Tuple

import rclpy
import torch
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from decision_module import DecisionModule
from perception_module import PerceptionModule
from preprocessing import CameraPreprocessor, FrameData


class TransformProvider(Node):
    """TF2 provider for numeric ^{map}T_{gripper_camera_color_optical_frame}."""

    def __init__(
        self,
        target_frame: str = "map",
        source_frame: str = "gripper_camera_color_optical_frame",
        default_timeout_sec: float = 0.02,
    ):
        super().__init__("transform_provider")

        self.target_frame = target_frame
        self.source_frame = source_frame
        self.default_timeout_sec = float(default_timeout_sec)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.lookup_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.latest_fallback_count = 0
        self.last_status = "never_called"
        self.last_error = ""
        self.last_used_latest_fallback = False

    @staticmethod
    def _rotation_matrix_from_quaternion(
        x: float, y: float, z: float, w: float,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
        norm = (x * x + y * y + z * z + w * w) ** 0.5
        if norm <= 1.0e-12:
            raise ValueError("zero-norm quaternion in TF transform")
        x /= norm
        y /= norm
        z /= norm
        w /= norm
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return (
            (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
            (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
            (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
        )

    def _transform_to_tensor(self, transform, device: torch.device) -> torch.Tensor:
        q = transform.transform.rotation
        t = transform.transform.translation
        R = self._rotation_matrix_from_quaternion(q.x, q.y, q.z, q.w)
        T = torch.empty((4, 4), dtype=torch.float32, device=device)
        T[0, 0] = R[0][0]
        T[0, 1] = R[0][1]
        T[0, 2] = R[0][2]
        T[0, 3] = float(t.x)
        T[1, 0] = R[1][0]
        T[1, 1] = R[1][1]
        T[1, 2] = R[1][2]
        T[1, 3] = float(t.y)
        T[2, 0] = R[2][0]
        T[2, 1] = R[2][1]
        T[2, 2] = R[2][2]
        T[2, 3] = float(t.z)
        T[3, 0] = 0.0
        T[3, 1] = 0.0
        T[3, 2] = 0.0
        T[3, 3] = 1.0
        return T

    def lookup_map_optical(
        self,
        stamp_sec: float,
        device: torch.device,
        timeout_sec: Optional[float] = None,
        allow_latest_fallback: bool = False,
    ) -> Optional[torch.Tensor]:
        """Return ^{map}T_{optical}, or None when TF is unavailable."""
        self.lookup_count += 1
        self.last_used_latest_fallback = False
        timeout = Duration(
            seconds=self.default_timeout_sec if timeout_sec is None else float(timeout_sec)
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                Time(seconds=float(stamp_sec)),
                timeout,
            )
            T_map_optical = self._transform_to_tensor(transform, torch.device(device))
            self.success_count += 1
            self.last_status = "ok"
            self.last_error = ""
            return T_map_optical
        except Exception as exc:
            primary_error = str(exc)

        if allow_latest_fallback:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    self.source_frame,
                    Time(),
                    timeout,
                )
                T_map_optical = self._transform_to_tensor(transform, torch.device(device))
                self.success_count += 1
                self.latest_fallback_count += 1
                self.last_used_latest_fallback = True
                self.last_status = "latest_fallback"
                self.last_error = primary_error
                return T_map_optical
            except Exception as exc:
                self.last_error = f"{primary_error}; latest fallback failed: {exc}"
        else:
            self.last_error = primary_error

        self.failure_count += 1
        self.last_status = "failed"
        return None


class StaticTransformProvider:
    """Fixed ^{map}T_{optical} provider for robotless standalone tests."""

    def __init__(self, device: torch.device):
        self.device = torch.device(device)
        self.lookup_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.last_status = "standalone"
        self.last_error = ""
        self.T_map_optical = torch.tensor(
            [
                [0.0, 0.0, 1.0, 0.0],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=torch.float32,
            device=self.device,
        )

    def lookup_map_optical(
        self,
        stamp_sec: float,
        device: torch.device,
        timeout_sec: Optional[float] = None,
        allow_latest_fallback: bool = False,
    ) -> torch.Tensor:
        self.lookup_count += 1
        self.success_count += 1
        self.last_status = "standalone"
        self.last_error = ""
        if torch.device(device) == self.device:
            return self.T_map_optical
        return self.T_map_optical.to(device=torch.device(device), dtype=torch.float32)


class PerceptionDecisionPipeline:
    """PerceptionModule + DecisionModule3 pipeline with GPU output retention."""

    def __init__(
        self,
        *,
        query: str = "a black tumbler",
        device: str = "cuda",
        score_threshold: float = 0.1,
        depth_band_m: float = 0.08,
        transform_provider=None,
        weights_dir: Optional[str] = None,
        sync_timing: bool = True,
    ):
        self.query = query
        self.device = torch.device(device)
        self.score_threshold = float(score_threshold)
        self.depth_band_m = float(depth_band_m)
        self.transform_provider = transform_provider
        self.sync_timing = bool(sync_timing)
        self._DecisionModule = DecisionModule
        self._PerceptionModule = PerceptionModule
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.weights_dir = (
            os.path.abspath(os.path.join(script_dir, "..", "weights"))
            if weights_dir is None
            else os.path.abspath(weights_dir)
        )

        self.perception = None
        self.decision = None
        self.ready = False
        self.image_shape: Optional[Tuple[int, int]] = None
        self.rgb_float_buf = None
        self.depth_float_buf = None
        self.mask_float_buf = None
        self.full_x_base_buf = None
        self.full_y_base_buf = None
        self.latest_grasp = None
        self.latest_T_map_target = None
        self.latest_control_ready = False
        self.latest_status_code = None
        self.latest_reason_code = None
        self.latest_status = "not_ready"
        self.latest_reason = "not_ready"
        self.last_infer_sec = 0.0
        self.last_boundary_sec = 0.0
        self.last_decision_sec = 0.0

        self.status_names = {
            DecisionModule._STATUS_RESET: "reset",
            DecisionModule._STATUS_REJECTED: "rejected",
            DecisionModule._STATUS_HELD: "held",
            DecisionModule._STATUS_FRESH: "fresh",
        }
        self.reason_names = {
            DecisionModule._REASON_NONE: "none",
            DecisionModule._REASON_INSUFFICIENT_POINTS: "insufficient_points",
            DecisionModule._REASON_APPROACH_PRIOR_DEGENERATE: "approach_prior_degenerate",
            DecisionModule._REASON_INITIAL_PHYSICAL_GATE: "initial_physical_gate",
            DecisionModule._REASON_INITIAL_HORIZONTAL_COMPONENT_MISSING: (
                "initial_horizontal_component_missing"
            ),
            DecisionModule._REASON_TRACKING_PHYSICAL_GATE: "tracking_physical_gate",
        }

    def set_transform_provider(self, transform_provider) -> None:
        self.transform_provider = transform_provider

    def setup(self, first_frame: FrameData) -> None:
        K = first_frame.intrinsics
        h_img, w_img = first_frame.depth.shape
        self.image_shape = (h_img, w_img)

        self.perception = self._PerceptionModule(
            owl_model_name="google/owlvit-base-patch32",
            owl_image_engine=os.path.join(
                self.weights_dir, "owlvit", "tensorrt", "32_image_encoder_fp16.engine"
            ),
            owl_text_engine=os.path.join(
                self.weights_dir, "owlvit", "tensorrt", "32_text_encoder_fp32.engine"
            ),
            sam_encoder_engine=os.path.join(
                self.weights_dir, "nanosam", "tensorrt", "encoder_fp16.engine"
            ),
            sam_decoder_engine=os.path.join(
                self.weights_dir, "nanosam", "tensorrt", "decoder_fp16.engine"
            ),
            device=str(self.device),
            score_threshold=self.score_threshold,
        )
        self.perception.set_text_query([self.query])
        self.perception.setup(first_frame.rgb)

        self.decision = self._DecisionModule(
            device=str(self.device),
            fx=float(K[0, 0]),
            fy=float(K[1, 1]),
            cx=float(K[0, 2]),
            cy=float(K[1, 2]),
            depth_band_m=self.depth_band_m,
        )

        v = torch.arange(h_img, device=self.device, dtype=torch.float32)
        u = torch.arange(w_img, device=self.device, dtype=torch.float32)
        vv, uu = torch.meshgrid(v, u, indexing="ij")
        self.full_x_base_buf = (uu - float(K[0, 2])) / float(K[0, 0])
        self.full_y_base_buf = (vv - float(K[1, 2])) / float(K[1, 1])
        self.rgb_float_buf = torch.empty(
            (3, h_img, w_img), dtype=torch.float32, device=self.device
        )
        self.depth_float_buf = torch.empty(
            (1, h_img, w_img), dtype=torch.float32, device=self.device
        )
        self.mask_float_buf = torch.empty((h_img, w_img), dtype=torch.float32, device=self.device)
        if self.transform_provider is None:
            self.transform_provider = StaticTransformProvider(self.device)
        self.ready = True
        self.latest_status = "ready"
        self.latest_reason = "none"

    def reset_decision(self) -> None:
        if self.decision is not None:
            self.decision.reset()
        self.latest_grasp = None
        self.latest_T_map_target = None
        self.latest_control_ready = False
        self.latest_status = "reset"
        self.latest_reason = "none"

    def warmup(self, frame: FrameData, n: int = 10) -> None:
        if not self.ready:
            self.setup(frame)
        for _ in range(max(0, int(n))):
            result = self.perception.infer_tensor(
                frame.rgb, depth_data=frame.depth, is_bgr=frame.is_bgr
            )
            result.event.synchronize()
        self.reset_decision()

    def _prepare_roi_ready_tensors(self, bbox_tensor):
        h_img, w_img = self.image_shape
        x0, y0, x1, y1 = bbox_tensor.long().detach().cpu().tolist()
        m = self.decision.crop_margin
        x0 = max(min(x0 - m, w_img - 1), 0)
        y0 = max(min(y0 - m, h_img - 1), 0)
        x1 = min(max(x1 + m, x0 + 1), w_img)
        y1 = min(max(y1 + m, y0 + 1), h_img)
        return (
            self.rgb_float_buf[:, y0:y1, x0:x1],
            self.depth_float_buf[0, y0:y1, x0:x1],
            self.mask_float_buf[y0:y1, x0:x1],
            self.full_x_base_buf[y0:y1, x0:x1],
            self.full_y_base_buf[y0:y1, x0:x1],
        )

    def _set_latest_grasp(self, grasp: dict) -> bool:
        output_valid = bool(grasp["output_valid"].item())
        stale = bool(grasp["stale"].item())
        feasible = bool(grasp["feasible"].item())
        self.latest_grasp = grasp
        self.latest_T_map_target = grasp["T_map_target"]
        self.latest_control_ready = output_valid and (not stale) and feasible
        status_code = int(grasp["status_code"].item())
        reason_code = int(grasp["reason_code"].item())
        self.latest_status_code = status_code
        self.latest_reason_code = reason_code
        self.latest_status = self.status_names.get(status_code, f"unknown({status_code})")
        self.latest_reason = self.reason_names.get(reason_code, f"unknown({reason_code})")
        return self.latest_control_ready

    @torch.inference_mode()
    def step(
        self,
        frame: FrameData,
        *,
        timeout_sec: Optional[float] = None,
        allow_latest_fallback: bool = False,
    ) -> dict:
        if not self.ready:
            self.setup(frame)

        infer_t0 = time.perf_counter()
        perception_result = self.perception.infer_tensor(
            frame.rgb, depth_data=frame.depth, is_bgr=frame.is_bgr
        )
        perception_result.event.synchronize()
        if self.device.type == "cuda" and self.sync_timing:
            torch.cuda.synchronize()
        infer_t1 = time.perf_counter()
        self.last_infer_sec = infer_t1 - infer_t0

        if not bool(perception_result.valid.item()):
            self.latest_control_ready = False
            self.latest_status = "perception_skip"
            self.latest_reason = "invalid_detection"
            self.last_boundary_sec = 0.0
            self.last_decision_sec = 0.0
            return {
                "control_ready": False,
                "grasp": self.latest_grasp,
                "T_map_target": self.latest_T_map_target,
                "perception_result": perception_result,
                "status": self.latest_status,
                "reason": self.latest_reason,
                "xform_valid": False,
                "infer_sec": self.last_infer_sec,
                "boundary_sec": self.last_boundary_sec,
                "decision_sec": self.last_decision_sec,
            }

        T_map_optical = self.transform_provider.lookup_map_optical(
            frame.timestamp,
            device=self.device,
            timeout_sec=timeout_sec,
            allow_latest_fallback=allow_latest_fallback,
        )
        if T_map_optical is None:
            self.latest_control_ready = False
            self.latest_status = "xform_skip"
            self.latest_reason = self.transform_provider.last_error or self.transform_provider.last_status
            self.last_boundary_sec = 0.0
            self.last_decision_sec = 0.0
            return {
                "control_ready": False,
                "grasp": self.latest_grasp,
                "T_map_target": self.latest_T_map_target,
                "perception_result": perception_result,
                "status": self.latest_status,
                "reason": self.latest_reason,
                "xform_valid": False,
                "infer_sec": self.last_infer_sec,
                "boundary_sec": self.last_boundary_sec,
                "decision_sec": self.last_decision_sec,
            }

        boundary_t0 = time.perf_counter()
        self.rgb_float_buf.copy_(self.perception.gpu_raw_image).mul_(1.0 / 255.0)
        self.depth_float_buf[0].copy_(self.perception.gpu_depth_image).mul_(0.001)
        self.mask_float_buf.copy_(perception_result.mask)
        roi_rgb, roi_depth, roi_mask, roi_x_base, roi_y_base = (
            self._prepare_roi_ready_tensors(perception_result.bbox)
        )
        boundary_t1 = time.perf_counter()
        self.last_boundary_sec = boundary_t1 - boundary_t0

        decision_t0 = time.perf_counter()
        grasp = self.decision.run_full_pipeline(
            roi_rgb,
            roi_depth,
            roi_mask,
            roi_x_base,
            roi_y_base,
            T_map_optical=T_map_optical,
        )
        if self.device.type == "cuda" and self.sync_timing:
            torch.cuda.synchronize()
        decision_t1 = time.perf_counter()
        self.last_decision_sec = decision_t1 - decision_t0
        control_ready = self._set_latest_grasp(grasp)
        return {
            "control_ready": control_ready,
            "grasp": grasp,
            "T_map_target": grasp["T_map_target"],
            "perception_result": perception_result,
            "status": self.latest_status,
            "reason": self.latest_reason,
            "xform_valid": True,
            "infer_sec": self.last_infer_sec,
            "boundary_sec": self.last_boundary_sec,
            "decision_sec": self.last_decision_sec,
        }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="L-OMM RGB-D ingress + PerceptionModule + DecisionModule3"
    )
    parser.add_argument("--query", type=str, default="a black tumbler")
    parser.add_argument("--camera-ns", type=str, default="gripper_camera")
    parser.add_argument("--camera-name", type=str, default="gripper_camera")
    parser.add_argument("--target-frame", type=str, default="map")
    parser.add_argument(
        "--source-frame", type=str, default="gripper_camera_color_optical_frame"
    )
    parser.add_argument(
        "--transform-mode",
        type=str,
        choices=("tf2", "standalone"),
        default="standalone",
        help="standalone uses fixed map<-optical; tf2 uses live TF lookup.",
    )
    parser.add_argument("--tf-timeout", type=float, default=0.02)
    parser.add_argument("--allow-latest-tf", action="store_true")
    parser.add_argument("--sync-slop", type=float, default=0.01)
    parser.add_argument("--sync-queue", type=int, default=5)
    parser.add_argument("--executor-threads", type=int, default=3)
    parser.add_argument("--msg-conversion", type=str, choices=("direct", "cv_bridge"), default="direct")
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--score-threshold", type=float, default=0.1)
    parser.add_argument("--depth-band-m", type=float, default=0.08)
    parser.add_argument("--warmup-iters", type=int, default=10)
    parser.add_argument("--print-every", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--no-sync-timing", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    preprocessor = None
    transform_provider = None
    executor = None

    try:
        preprocessor = CameraPreprocessor(
            camera_namespace=args.camera_ns,
            camera_name=args.camera_name,
            sync_slop=args.sync_slop,
            sync_queue=args.sync_queue,
            msg_conversion=args.msg_conversion,
        )
        if args.transform_mode == "tf2":
            transform_provider = TransformProvider(
                target_frame=args.target_frame,
                source_frame=args.source_frame,
                default_timeout_sec=args.tf_timeout,
            )

        executor = MultiThreadedExecutor(num_threads=max(args.executor_threads, 2))
        executor.add_node(preprocessor)
        if transform_provider is not None:
            executor.add_node(transform_provider)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        print("\nL-OMM live pipeline: RGB-D ingress -> perception -> decision_module3")
        print("Waiting for camera data...")
        if not preprocessor.wait_for_ready(timeout=30.0):
            print("ERROR: Camera data not received within 30s.")
            sys.exit(1)

        first_frame = None
        while first_frame is None:
            first_frame = preprocessor.wait_for_frame(timeout=0.5)

        K = first_frame.intrinsics
        h_img, w_img = first_frame.depth.shape
        print(
            f"Camera ready: {w_img}x{h_img}, "
            f"fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}, "
            f"cx={K[0, 2]:.1f}, cy={K[1, 2]:.1f}"
        )
        if args.transform_mode == "tf2":
            print(f"Transform mode: tf2 ({args.target_frame} <- {args.source_frame})")
        else:
            print("Transform mode: standalone (fixed map<-optical matrix)")

        pipeline = PerceptionDecisionPipeline(
            query=args.query,
            device=args.device,
            score_threshold=args.score_threshold,
            depth_band_m=args.depth_band_m,
            transform_provider=transform_provider,
            sync_timing=not args.no_sync_timing,
        )

        preprocessor.suspend_callbacks()
        pipeline.setup(first_frame)
        print(f"Warmup ({max(0, args.warmup_iters)} iterations)...")
        pipeline.warmup(first_frame, args.warmup_iters)
        preprocessor.resume_callbacks()
        preprocessor.clear_pending_frame()
        preprocessor.reset_runtime_counters()

        print("Live loop started.")
        processed = 0
        valid_det = 0
        xform_ok = 0
        xform_fail = 0
        skipped_invalid = 0
        skipped_xform = 0
        fresh_grasp = 0
        held_grasp = 0
        rejected_grasp = 0
        control_ready_count = 0
        timeout_count = 0
        infer_time = 0.0
        boundary_time = 0.0
        decision_time = 0.0
        wait_time = 0.0
        buffer_age_accum_ms = 0.0
        latest_grasp = None
        latest_status = "none"
        latest_reason = "none"

        last_print = time.perf_counter()
        last_processed = 0
        last_valid_det = 0
        last_xform_ok = 0
        last_xform_fail = 0
        last_skipped_invalid = 0
        last_skipped_xform = 0
        last_fresh_grasp = 0
        last_held_grasp = 0
        last_rejected_grasp = 0
        last_control_ready_count = 0
        last_timeout_count = 0
        last_infer_time = 0.0
        last_boundary_time = 0.0
        last_decision_time = 0.0
        last_wait_time = 0.0
        last_buffer_age_accum_ms = 0.0
        last_camera_count = preprocessor.frame_count
        last_overwrite_count = preprocessor.overwrite_count

        while rclpy.ok():
            if args.max_frames > 0 and processed >= args.max_frames:
                break

            wait_t0 = time.perf_counter()
            frame = preprocessor.wait_for_frame(timeout=0.2)
            wait_t1 = time.perf_counter()
            wait_time += wait_t1 - wait_t0
            if frame is None:
                timeout_count += 1
                continue

            buffer_age_accum_ms += max(
                (time.time() - frame.received_timestamp) * 1000.0, 0.0
            )
            result = pipeline.step(
                frame,
                timeout_sec=args.tf_timeout,
                allow_latest_fallback=args.allow_latest_tf,
            )
            processed += 1
            infer_time += result["infer_sec"]
            boundary_time += result["boundary_sec"]
            decision_time += result["decision_sec"]
            latest_status = result["status"]
            latest_reason = str(result["reason"]).strip().replace("\n", " ")

            perception_result = result["perception_result"]
            if bool(perception_result.valid.item()):
                valid_det += 1
            else:
                skipped_invalid += 1

            if result["xform_valid"]:
                xform_ok += 1
            elif bool(perception_result.valid.item()):
                xform_fail += 1
                skipped_xform += 1

            grasp = result["grasp"]
            if result["xform_valid"] and grasp is not None:
                status_code = int(grasp["status_code"].item())
                if status_code == pipeline._DecisionModule._STATUS_FRESH:
                    fresh_grasp += 1
                elif status_code == pipeline._DecisionModule._STATUS_HELD:
                    held_grasp += 1
                elif status_code == pipeline._DecisionModule._STATUS_REJECTED:
                    rejected_grasp += 1
                latest_grasp = grasp

            if result["control_ready"]:
                control_ready_count += 1

            now = time.perf_counter()
            if now - last_print < args.print_every:
                continue

            dt = now - last_print
            dp = processed - last_processed
            dd = valid_det - last_valid_det
            dxok = xform_ok - last_xform_ok
            dxfail = xform_fail - last_xform_fail
            dsi = skipped_invalid - last_skipped_invalid
            dsx = skipped_xform - last_skipped_xform
            df = fresh_grasp - last_fresh_grasp
            dh = held_grasp - last_held_grasp
            dr = rejected_grasp - last_rejected_grasp
            dcr = control_ready_count - last_control_ready_count
            dto = timeout_count - last_timeout_count
            di = infer_time - last_infer_time
            db = boundary_time - last_boundary_time
            ddc = decision_time - last_decision_time
            dw = wait_time - last_wait_time
            dba = buffer_age_accum_ms - last_buffer_age_accum_ms
            camera_count = preprocessor.frame_count
            overwrite_count = preprocessor.overwrite_count
            dcamera = camera_count - last_camera_count
            doverwrite = overwrite_count - last_overwrite_count

            latest_summary = f"{latest_status} reason={latest_reason}"
            if latest_grasp is not None:
                p_map = latest_grasp["p_map_raw"].detach().cpu().tolist()
                p_target = latest_grasp["T_map_target"][:3, 3].detach().cpu().tolist()
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
                    f"pts(raw/band)={int(latest_grasp['point_count_raw'].item())}/"
                    f"{int(latest_grasp['point_count_band'].item())} "
                    f"reason={latest_reason}"
                )

            loop_hz = dp / dt if dt > 1.0e-9 else 0.0
            camera_fps = dcamera / dt if dt > 1.0e-9 else 0.0
            infer_ms = (di / max(dp, 1)) * 1000.0
            boundary_ms = (db / max(dxok, 1)) * 1000.0
            decision_ms = (ddc / max(dxok, 1)) * 1000.0
            wait_ms = (dw / max(dp + dto, 1)) * 1000.0
            buffer_age_ms = dba / max(dp, 1)

            print(
                f"Loop {loop_hz:.1f} Hz | Camera {camera_fps:.1f} FPS | "
                f"det {dd}/{dp} | xform {dxok}/{dd} fail {dxfail} | "
                f"skip invalid/xform {dsi}/{dsx} | "
                f"fresh/held/rejected {df}/{dh}/{dr} | control_ready {dcr} | "
                f"infer {infer_ms:.2f} ms | boundary {boundary_ms:.2f} ms | "
                f"decision_core {decision_ms:.2f} ms | wait {wait_ms:.2f} ms | "
                f"buffer_age {buffer_age_ms:.2f} ms | timeout {dto} | "
                f"overwrite {doverwrite} | {latest_summary}"
            )

            last_print = now
            last_processed = processed
            last_valid_det = valid_det
            last_xform_ok = xform_ok
            last_xform_fail = xform_fail
            last_skipped_invalid = skipped_invalid
            last_skipped_xform = skipped_xform
            last_fresh_grasp = fresh_grasp
            last_held_grasp = held_grasp
            last_rejected_grasp = rejected_grasp
            last_control_ready_count = control_ready_count
            last_timeout_count = timeout_count
            last_infer_time = infer_time
            last_boundary_time = boundary_time
            last_decision_time = decision_time
            last_wait_time = wait_time
            last_buffer_age_accum_ms = buffer_age_accum_ms
            last_camera_count = camera_count
            last_overwrite_count = overwrite_count

    except KeyboardInterrupt:
        print("\nInterrupted by user (Ctrl+C)")
    finally:
        if preprocessor is not None:
            preprocessor.destroy_node()
        if transform_provider is not None:
            transform_provider.destroy_node()
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        rclpy.shutdown()
        print("Pipeline stopped.")
