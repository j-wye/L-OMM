"""
Perception Module v3: Optimized Open-Vocabulary Detection + Segmentation Pipeline

Design Principles (from gemini_analysis.md):
  P0: Restore v1 CPU-permute transfer path (remove GPU transpose)
  P1: Add BGR→RGB conversion for correct input semantics
  P2: Zero-allocation bridge from v2
  P3: PerceptionResult + score gating from v2
  P4: Exclusive per-stage CUDA Event profiling (new)
  P5: Resolution guard + setup guard from v2
  P6: API contract documentation (single-flight, dtype, layout)

API Contract:
  - Input: uint8 HWC numpy array (BGR or RGB, specified by is_bgr flag)
  - is_bgr=True  (default): OpenCV/CvBridge bgr8 convention → internal BGR→RGB
  - is_bgr=False: RealSense rgb8 or CvBridge passthrough → no channel swap
  - Single-flight only: caller MUST synchronize the returned event before
    calling infer_tensor() again. Overlapping calls cause data corruption.
  - Resolution: must match the sample_image given to setup(). Mismatch raises.

Target: Jetson AGX Orin 64GB @ MAXN mode
Author: L-OMM Perception Pipeline
"""

import torch
import torch.nn.functional as F
import numpy as np
import tensorrt as trt
from torch2trt import TRTModule
import cv2
import os
import sys
import time
from transformers import OwlViTProcessor
from typing import List, Tuple, Optional, NamedTuple


class PerceptionResult(NamedTuple):
    bbox: torch.Tensor
    mask: torch.Tensor
    score: torch.Tensor
    valid: torch.Tensor
    event: torch.cuda.Event

class PerceptionModule:
    """4-Stream CUDA Graph Perception Pipeline.

    Architecture:
        Stream 0 (copy):     CPU→GPU async transfer (BGR→RGB + HWC→CHW on CPU)
        Stream 1 (owl):      OWL-ViT preprocessing → TRT inference → postprocessing
        Stream 2 (sam_enc):  NanoSAM preprocessing → TRT encoder
        Stream 3 (sam_dec):  Bridge → TRT decoder → mask upscale

    Critical Path:
        T_critical = T_copy + max(T_owl, T_sam_enc) + T_sam_dec
    """

    def __init__(self,
                 owl_model_name: str,
                 owl_image_engine: str,
                 owl_text_engine: str,
                 sam_encoder_engine: str,
                 sam_decoder_engine: str,
                 device: str = "cuda",
                 score_threshold: float = 0.1):

        self.device = torch.device(device)
        self.score_threshold = score_threshold
        print(f"[PerceptionModule v3] Initializing on {self.device}...")

        # HuggingFace processor for tokenizer & normalization constants
        self.processor = OwlViTProcessor.from_pretrained(owl_model_name)
        self.owl_image_size = self.processor.image_processor.size['height']

        # TensorRT engines
        print("   - Loading TensorRT Engines...")
        self.owl_image_engine = self._load_engine(
            owl_image_engine, ["image"],
            ["image_embeds", "image_class_embeds", "logit_shift", "logit_scale", "pred_boxes"])
        self.owl_text_engine = self._load_engine(
            owl_text_engine, ["input_ids", "attention_mask"], ["text_embeds"])
        self.sam_encoder_engine = self._load_engine(
            sam_encoder_engine, ["image"], ["image_embeddings"])
        self.sam_decoder_engine = self._load_engine(
            sam_decoder_engine,
            ["image_embeddings", "point_coords", "point_labels", "mask_input", "has_mask_input"],
            ["iou_predictions", "low_res_masks"])

        # 4-Stream DAG
        self.stream_copy = torch.cuda.Stream()
        self.stream_owl = torch.cuda.Stream()
        self.stream_sam_enc = torch.cuda.Stream()
        self.stream_sam_dec = torch.cuda.Stream()

        # Inter-stream synchronization events
        self.event_copy_done = torch.cuda.Event()
        self.event_owl_done = torch.cuda.Event()
        self.event_sam_enc_done = torch.cuda.Event()
        self.event_final_done = torch.cuda.Event()

        # Text embedding cache
        self.cached_text_query: Optional[List[str]] = None
        self.text_embeds: Optional[torch.Tensor] = None

        # SAM normalization constants (ImageNet, raw pixel scale [0,255])
        self.sam_target_size = 1024
        self.sam_mean = torch.tensor([123.675, 116.28, 103.53], device=self.device).view(3, 1, 1)
        self.sam_std = torch.tensor([58.395, 57.12, 57.375], device=self.device).view(3, 1, 1)

        # OWL normalization constants (ImageNet, [0,1] scale)
        self.owl_mean = torch.tensor(
            self.processor.image_processor.image_mean, device=self.device).view(1, 3, 1, 1)
        self.owl_std = torch.tensor(
            self.processor.image_processor.image_std, device=self.device).view(1, 3, 1, 1)

        # CUDA Graph handles (populated in setup)
        self.graph_owl: Optional[torch.cuda.CUDAGraph] = None
        self.graph_sam_enc: Optional[torch.cuda.CUDAGraph] = None
        self.graph_sam_dec: Optional[torch.cuda.CUDAGraph] = None

        # Static I/O buffers (populated in setup)
        self.static_inputs: dict = {}
        self.static_outputs: dict = {}

        # Host↔Device transfer buffers (populated in setup)
        # v3: CHW layout on pinned memory (v1 approach, no GPU transpose)
        self.pinned_cpu_buffer: Optional[torch.Tensor] = None
        self.gpu_raw_image: Optional[torch.Tensor] = None

        # Depth pinned path (optional, used when caller supplies depth_data).
        # Piggy-backs on stream_copy, fully hidden behind OWL critical path.
        self.pinned_depth_cpu: Optional[torch.Tensor] = None
        self.gpu_depth_image: Optional[torch.Tensor] = None
        self._pinned_depth_np: Optional[np.ndarray] = None

        # Bridge is fused into OWL graph (no per-frame buffer needed)

        # Resolution tracking for mismatch guard
        self.orig_shape: Optional[Tuple[int, int]] = None

        print("[PerceptionModule v3] Initialized.")

    # =========================================================================
    # Engine Loading
    # =========================================================================

    @staticmethod
    def _load_engine(path: str, input_names: List[str], output_names: List[str]) -> TRTModule:
        if not os.path.exists(path):
            raise FileNotFoundError(f"Engine not found: {path}")
        logger = trt.Logger(trt.Logger.WARNING)
        with open(path, "rb") as f, trt.Runtime(logger) as runtime:
            engine = runtime.deserialize_cuda_engine(f.read())
        return TRTModule(engine, input_names=input_names, output_names=output_names)

    # =========================================================================
    # JIT-compiled Static Functions (Graph-capturable)
    # =========================================================================

    @staticmethod
    @torch.jit.script
    def jit_preprocess_owl(raw_img: torch.Tensor, mean: torch.Tensor, std: torch.Tensor,
                           new_h: int, new_w: int,
                           pad_dims: Tuple[int, int, int, int]) -> torch.Tensor:
        """OWL-ViT preprocessing: resize → letterbox pad → normalize.
        Input: (C, H, W) uint8 RGB   Output: (1, C, H', W') float32
        Uses FP16 for resize+pad+normalize to halve memory bandwidth,
        then converts to FP32 for TRT engine compatibility."""
        pad_left, pad_right, pad_top, pad_bottom = pad_dims
        resized = F.interpolate(raw_img.unsqueeze(0).half(), size=(new_h, new_w),
                                mode='bilinear', align_corners=False)
        padded = F.pad(resized, (pad_left, pad_right, pad_top, pad_bottom), "constant", 0.0)
        norm = (padded / 255.0 - mean.half()) / std.half()
        return norm.float()

    @staticmethod
    @torch.jit.script
    def jit_postprocess_owl(cls_embeds: torch.Tensor, shift: torch.Tensor, scale: torch.Tensor,
                            boxes: torch.Tensor, text_embeds: torch.Tensor,
                            post_params: Tuple[float, int, int, int, int],
                            image_size: int) -> Tuple[torch.Tensor, torch.Tensor]:
        """OWL-ViT postprocessing: cosine similarity → score → best box in original coords."""
        post_scale, post_pad_x, post_pad_y, orig_h, orig_w = post_params

        # L2 normalize class embeddings
        cls_embeds = cls_embeds / (torch.linalg.norm(cls_embeds, dim=-1, keepdim=True) + 1e-6)

        # Cosine similarity → learned affine → sigmoid
        logits = torch.einsum("...pd,...qd->...pq", cls_embeds, text_embeds)
        logits = logits * scale + shift
        scores = torch.sigmoid(logits).squeeze()

        # Select best detection
        best_score, best_idx = torch.max(scores, 0)
        idx_tensor = best_idx.view(1, 1, 1).expand(1, 1, 4)
        best_box_rel = torch.gather(boxes, 1, idx_tensor).squeeze()

        # Restore to original pixel coordinates
        box_scaled = best_box_rel * image_size
        x0 = (box_scaled[0] - post_pad_x) * post_scale
        y0 = (box_scaled[1] - post_pad_y) * post_scale
        x1 = (box_scaled[2] - post_pad_x) * post_scale
        y1 = (box_scaled[3] - post_pad_y) * post_scale
        final_box = torch.stack([x0, y0, x1, y1])
        final_box[0::2].clamp_(0, orig_w)
        final_box[1::2].clamp_(0, orig_h)

        return final_box, best_score

    @staticmethod
    @torch.jit.script
    def jit_preprocess_sam(img_chw: torch.Tensor, target_size: int,
                           mean: torch.Tensor, std: torch.Tensor) -> Tuple[torch.Tensor, float, Tuple[int, int]]:
        """SAM preprocessing: resize (longest edge) → center-pad → normalize.
        Input: (C, H, W) float RGB   Output: (C, 1024, 1024) float32"""
        h, w = img_chw.shape[1], img_chw.shape[2]
        scale = float(target_size) / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)

        resized = F.interpolate(img_chw.unsqueeze(0), size=(new_h, new_w),
                                mode='bilinear', align_corners=False).squeeze(0)
        padded = torch.zeros((3, target_size, target_size), dtype=resized.dtype, device=resized.device)
        dh, dw = (target_size - new_h) // 2, (target_size - new_w) // 2
        padded[:, dh:dh+new_h, dw:dw+new_w] = resized
        norm = (padded - mean) / std

        return norm, scale, (dh, dw)

    # Bridge is fused into OWL Graph — no per-frame method call needed.
    # See setup() Graph 1 capture for the fused bridge logic.

    # =========================================================================
    # Text Query Management
    # =========================================================================

    @torch.no_grad()
    def set_text_query(self, text_query: List[str]):
        """Encode and cache text query. Only recomputes if query changes."""
        if text_query == self.cached_text_query:
            return
        print(f"   - [Update] Text Query: {text_query}")
        text_input = self.processor(text=text_query, return_tensors="pt")
        input_ids = text_input['input_ids'].to(self.device).unsqueeze(0)
        attention_mask = text_input['attention_mask'].to(self.device).unsqueeze(0)

        text_embeds_output = self.owl_text_engine(input_ids, attention_mask)
        text_embeds = text_embeds_output[0] if isinstance(text_embeds_output, (tuple, list)) else text_embeds_output
        self.text_embeds = text_embeds / (torch.linalg.norm(text_embeds, dim=-1, keepdim=True) + 1e-6)
        self.cached_text_query = text_query

        # Hot-swap text embeddings into captured graph's static buffer
        if self.graph_owl is not None:
            self.static_inputs['owl_text_embeds'].copy_(self.text_embeds)

    # =========================================================================
    # Setup: Buffer Allocation + CUDA Graph Capture
    # =========================================================================

    @torch.no_grad()
    def setup(self, sample_image: np.ndarray):
        """Capture CUDA Graphs for deterministic-latency inference.

        Args:
            sample_image: Representative BGR HWC uint8 image for graph capture.
                          All subsequent infer_tensor() calls must use the same resolution.
        """
        if self.text_embeds is None:
            raise RuntimeError("set_text_query() must be called before setup().")

        print("\n--- Setting up Perception Pipeline v3 (Graph Capture) ---")
        h, w, c = sample_image.shape
        self.orig_shape = (h, w)

        # =================================================================
        # v3 Transfer Buffers: CHW pinned + CHW GPU (v1 approach, no GPU transpose)
        # BGR→RGB is done at CPU copy time via numpy negative-stride view
        # =================================================================
        self.pinned_cpu_buffer = torch.empty((c, h, w), dtype=torch.uint8).pin_memory()
        self._pinned_np = self.pinned_cpu_buffer.numpy()  # cached numpy view for zero-overhead copyto
        self.gpu_raw_image = torch.empty((c, h, w), dtype=torch.uint8, device=self.device)

        # Depth transfer buffers: int16 (= uint16 bit-reinterpretation for depth ≤ 32.767m).
        # Decoupled from RGB so callers that don't need depth pay zero cost.
        # On consumers: .float() * depth_scale restores true meters (negative values
        # only possible if raw depth > 32767mm, filtered downstream by depth > 0).
        self.pinned_depth_cpu = torch.empty((h, w), dtype=torch.int16).pin_memory()
        self._pinned_depth_np = self.pinned_depth_cpu.numpy().view(np.uint16)  # zero-copy dtype view
        self.gpu_depth_image = torch.empty((h, w), dtype=torch.int16, device=self.device)

        # =================================================================
        # OWL Preprocessing Parameters (computed once, baked into graph)
        # =================================================================
        scale_owl = self.owl_image_size / max(h, w)
        new_h_owl, new_w_owl = int(h * scale_owl), int(w * scale_owl)
        pad_h_owl, pad_w_owl = self.owl_image_size - new_h_owl, self.owl_image_size - new_w_owl
        pad_l, pad_r = pad_w_owl // 2, pad_w_owl - (pad_w_owl // 2)
        pad_t, pad_b = pad_h_owl // 2, pad_h_owl - (pad_h_owl // 2)

        self.owl_params_pre = {'new_h': new_h_owl, 'new_w': new_w_owl,
                               'pad_dims': (pad_l, pad_r, pad_t, pad_b)}
        self.owl_params_post = {'post_params': (1/scale_owl, pad_l, pad_t, h, w),
                                'image_size': self.owl_image_size}

        # =================================================================
        # SAM Static Buffers
        # =================================================================
        self.sam_img_embeds = torch.empty((1, 256, 64, 64), dtype=torch.float32, device=self.device)
        self.sam_points = torch.empty((1, 3, 2), dtype=torch.float32, device=self.device)
        self.sam_labels = torch.tensor([[2, 3, 1]], dtype=torch.float32, device=self.device)
        self.sam_mask_input = torch.zeros((1, 1, 256, 256), dtype=torch.float32, device=self.device)
        self.sam_has_mask = torch.zeros((1), dtype=torch.float32, device=self.device)

        # =================================================================
        # Static I/O Buffers
        # =================================================================
        self.static_inputs['sam_raw_image'] = self.gpu_raw_image
        self.static_outputs['sam_mask'] = torch.empty((h, w), dtype=torch.bool, device=self.device)
        self.static_outputs['sam_iou'] = torch.empty((), dtype=torch.float32, device=self.device)
        self.static_outputs['owl_box'] = torch.zeros(4, dtype=torch.float32, device=self.device)
        self.static_outputs['owl_score'] = torch.zeros((), dtype=torch.float32, device=self.device)
        self.static_outputs['valid'] = torch.zeros((), dtype=torch.bool, device=self.device)
        self.static_inputs['owl_text_embeds'] = self.text_embeds.clone()

        # =================================================================
        # Pre-compute SAM spatial constants (needed by bridge fusion in OWL graph)
        # Must run BEFORE OWL graph capture so sam_scale/pad are available.
        # =================================================================
        _, self.sam_scale, self.sam_pad = self.jit_preprocess_sam(
            self.gpu_raw_image.float(), self.sam_target_size, self.sam_mean, self.sam_std)
        torch.cuda.synchronize()
        sam_s = float(self.sam_scale)
        sam_ph = float(self.sam_pad[0])
        sam_pw = float(self.sam_pad[1])

        # =================================================================
        # Graph 1: OWL-ViT + Bridge Fusion
        # Bridge writes directly to sam_points inside the graph, eliminating
        # per-frame kernel launches and scheduling gap.
        # =================================================================
        print("   - Capturing Graph 1: OwlViT + Bridge...")
        def run_owl():
            norm = self.jit_preprocess_owl(self.gpu_raw_image, self.owl_mean, self.owl_std,
                                           **self.owl_params_pre)
            _, cls, shift, scale, boxes = self.owl_image_engine(norm)
            return self.jit_postprocess_owl(cls, shift, scale, boxes,
                                            self.static_inputs['owl_text_embeds'],
                                            **self.owl_params_post)

        for _ in range(3):
            run_owl()
        torch.cuda.synchronize()

        self.graph_owl = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.graph_owl, stream=self.stream_owl):
            box, score = run_owl()
            self.static_outputs['owl_box'].copy_(box)
            self.static_outputs['owl_score'].copy_(score)
            self.static_outputs['valid'].copy_(score > self.score_threshold)

            # ── Bridge Fusion ──────────────────────────────────────────
            # OWL bbox → SAM prompt points, written directly to sam_points.
            # During replay, box[0..3] contain new TRT outputs, and these
            # operations re-execute on the same memory addresses.
            # Constants (sam_s, sam_ph, sam_pw) are baked at capture time.
            # ───────────────────────────────────────────────────────────
            x1, y1, x2, y2 = box[0], box[1], box[2], box[3]
            self.sam_points[0, 0, 0] = x1 * sam_s + sam_pw  # top-left x
            self.sam_points[0, 0, 1] = y1 * sam_s + sam_ph  # top-left y
            self.sam_points[0, 1, 0] = x2 * sam_s + sam_pw  # bottom-right x
            self.sam_points[0, 1, 1] = y2 * sam_s + sam_ph  # bottom-right y
            self.sam_points[0, 2, 0] = (x1 + x2) * 0.5 * sam_s + sam_pw  # center x
            self.sam_points[0, 2, 1] = (y1 + y2) * 0.5 * sam_s + sam_ph  # center y

        # =================================================================
        # Graph 2: NanoSAM Encoder
        # =================================================================
        print("   - Capturing Graph 2: NanoSAM Encoder...")
        self.sam_input_processed = torch.empty((1, 3, 1024, 1024), dtype=torch.float32, device=self.device)

        for _ in range(3):
            p, _, _ = self.jit_preprocess_sam(
                self.gpu_raw_image.float(), self.sam_target_size, self.sam_mean, self.sam_std)
            self.sam_encoder_engine(p.unsqueeze(0))
        torch.cuda.synchronize()

        self.graph_sam_enc = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.graph_sam_enc, stream=self.stream_sam_enc):
            p, _, _ = self.jit_preprocess_sam(self.gpu_raw_image.float(), self.sam_target_size,
                                              self.sam_mean, self.sam_std)
            self.sam_input_processed.copy_(p.unsqueeze(0))
            embeds, = self.sam_encoder_engine(self.sam_input_processed)
            self.sam_img_embeds.copy_(embeds)

        # =================================================================
        # Graph 3: NanoSAM Decoder (decode → best mask → upscale)
        # =================================================================
        print("   - Capturing Graph 3: NanoSAM Decoder...")
        for _ in range(3):
            self.sam_decoder_engine(self.sam_img_embeds, self.sam_points, self.sam_labels,
                                   self.sam_mask_input, self.sam_has_mask)
        torch.cuda.synchronize()

        # Pre-compute mask crop parameters (fixed for given resolution)
        pad_h, pad_w = self.sam_pad
        pad_h_256 = int(pad_h * 0.25)
        pad_w_256 = int(pad_w * 0.25)
        unpadded_h = int(h * self.sam_scale * 0.25)
        unpadded_w = int(w * self.sam_scale * 0.25)

        self.graph_sam_dec = torch.cuda.CUDAGraph()
        with torch.cuda.graph(self.graph_sam_dec, stream=self.stream_sam_dec):
            iou, low_masks = self.sam_decoder_engine(self.sam_img_embeds, self.sam_points,
                                                     self.sam_labels, self.sam_mask_input,
                                                     self.sam_has_mask)
            best_idx = torch.argmax(iou, dim=1).view(-1, 1, 1, 1)
            best_mask = torch.gather(low_masks, 1, best_idx.expand(-1, -1, 256, 256))
            mask_cropped = best_mask[..., pad_h_256 : pad_h_256 + unpadded_h,
                                         pad_w_256 : pad_w_256 + unpadded_w]
            final_mask = F.interpolate(mask_cropped, size=(h, w), mode='bilinear', align_corners=False)
            self.static_outputs['sam_mask'].copy_((final_mask > 0.0).squeeze())
            self.static_outputs['sam_iou'].copy_(torch.max(iou))

        print("[PerceptionModule v3] All Graphs Captured.")

    # =========================================================================
    # Main Inference
    # =========================================================================

    def infer_tensor(
        self,
        input_data,
        depth_data: Optional[np.ndarray] = None,
        is_bgr: bool = True,
    ) -> PerceptionResult:
        """Run full perception pipeline (async, returns CUDA Event).

        IMPORTANT: This is a single-flight API. The caller MUST synchronize the
        returned event before calling infer_tensor() again. Overlapping calls
        corrupt shared buffers.

        Args:
            input_data: HWC uint8 numpy array (BGR or RGB, see is_bgr).
            depth_data: Optional HW uint16 numpy array (RealSense aligned depth, mm).
                        When provided, a zero-copy pinned→GPU DMA is issued on the
                        same stream_copy used for RGB. The resulting GPU tensor is
                        exposed as self.gpu_depth_image (int16, bit-identical to
                        uint16 for values ≤ 32767). Downstream consumers can read
                        it after event_copy_done without any additional H2D cost.
                        Not used by OWL/SAM — this is a piggy-back for Decision.
            is_bgr: True if BGR (OpenCV/cv2.imread). False if RGB (RealSense rgb8).

        Returns:
            PerceptionResult with bbox, mask, score, valid flag, and sync event.
        """
        if self.graph_owl is None:
            raise RuntimeError("setup() must be called before infer_tensor().")

        # =================================================================
        # Stage 0: CPU→GPU Async Transfer
        # =================================================================
        if isinstance(input_data, np.ndarray):
            h, w = input_data.shape[:2]
            if (h, w) != self.orig_shape:
                raise RuntimeError(
                    f"Resolution mismatch: expected {self.orig_shape}, got ({h}, {w}). "
                    f"Call setup() again with the new resolution.")

            if is_bgr:
                # BGR→RGB + HWC→CHW: single-pass strided copy
                np.copyto(self._pinned_np, input_data[:, :, ::-1].transpose(2, 0, 1))
            else:
                # RGB + HWC→CHW: single-pass strided copy (no channel swap)
                np.copyto(self._pinned_np, input_data.transpose(2, 0, 1))

            # Optional depth piggy-back (uint16 HW → pinned int16 HW, same layout)
            if depth_data is not None:
                if depth_data.shape != self.orig_shape:
                    raise RuntimeError(
                        f"Depth resolution mismatch: expected {self.orig_shape}, "
                        f"got {depth_data.shape}.")
                np.copyto(self._pinned_depth_np, depth_data)

            with torch.cuda.stream(self.stream_copy):
                self.gpu_raw_image.copy_(self.pinned_cpu_buffer, non_blocking=True)
                if depth_data is not None:
                    # Same stream as RGB: DMA engine serializes back-to-back,
                    # but both copies finish before event_copy_done is recorded,
                    # so downstream (OWL, SAM, Decision) all see consistent data.
                    self.gpu_depth_image.copy_(self.pinned_depth_cpu, non_blocking=True)
                self.stream_copy.record_event(self.event_copy_done)

        elif isinstance(input_data, torch.Tensor):
            # Tensor input: expect CHW RGB uint8 on CUDA
            with torch.cuda.stream(self.stream_copy):
                if input_data.device != self.device:
                    input_data = input_data.to(self.device, non_blocking=True)
                if input_data.dim() == 3 and input_data.shape[2] == 3:
                    self.gpu_raw_image.copy_(input_data.permute(2, 0, 1))
                else:
                    self.gpu_raw_image.copy_(input_data, non_blocking=True)
                self.stream_copy.record_event(self.event_copy_done)

        # =================================================================
        # Stage 1: OWL-ViT + Bridge (Stream 1, waits on copy)
        # No context manager needed: graph.replay() uses captured stream.
        # =================================================================
        self.stream_owl.wait_event(self.event_copy_done)
        self.graph_owl.replay()
        self.stream_owl.record_event(self.event_owl_done)

        # =================================================================
        # Stage 2: SAM Encoder (Stream 2, waits on copy, parallel with OWL)
        # =================================================================
        self.stream_sam_enc.wait_event(self.event_copy_done)
        self.graph_sam_enc.replay()
        self.stream_sam_enc.record_event(self.event_sam_enc_done)

        # =================================================================
        # Stage 3: SAM Decoder (Stream 3, waits on OWL + SAM Enc)
        # Bridge is fused into OWL graph — sam_points already updated.
        # =================================================================
        self.stream_sam_dec.wait_event(self.event_owl_done)
        self.stream_sam_dec.wait_event(self.event_sam_enc_done)
        self.graph_sam_dec.replay()
        self.stream_sam_dec.record_event(self.event_final_done)

        return PerceptionResult(
            bbox=self.static_outputs['owl_box'],
            mask=self.static_outputs['sam_mask'],
            score=self.static_outputs['owl_score'],
            valid=self.static_outputs['valid'],
            event=self.event_final_done
        )

    # =========================================================================
    # Per-Stage Latency Profiling (Exclusive Stage Timing)
    # =========================================================================

    def profile_latency(self, input_data: np.ndarray, is_bgr: bool = True,
                        n_warmup: int = 20, n_measure: int = 100) -> dict:
        """Measure EXCLUSIVE per-stage latency using CUDA Events.

        Unlike v2's cumulative timing, this measures each stage's own duration:
          copy:    copy_start → copy_end
          owl:     owl_start  → owl_end
          sam_enc: se_start   → se_end
          sam_dec: sd_start   → sd_end
          total:   frame_start → frame_end
        """
        for _ in range(n_warmup):
            result = self.infer_tensor(input_data, is_bgr=is_bgr)
            result.event.synchronize()

        # Per-stage timing events (enable_timing=True for elapsed_time)
        e_frame_start = torch.cuda.Event(enable_timing=True)
        e_copy_start = torch.cuda.Event(enable_timing=True)
        e_copy_end = torch.cuda.Event(enable_timing=True)
        e_owl_start = torch.cuda.Event(enable_timing=True)
        e_owl_end = torch.cuda.Event(enable_timing=True)
        e_se_start = torch.cuda.Event(enable_timing=True)
        e_se_end = torch.cuda.Event(enable_timing=True)
        e_sd_start = torch.cuda.Event(enable_timing=True)
        e_sd_end = torch.cuda.Event(enable_timing=True)

        timings = {
            'cpu_copy': [], 'copy': [], 'owl': [], 'sam_enc': [], 'sam_dec': [],
            'total': [], 'critical_path': []
        }

        for _ in range(n_measure):
            torch.cuda.synchronize()
            e_frame_start.record()

            # --- Manually inline the pipeline with per-stage events ---

            # Stage 0: Copy (CPU np.copyto + GPU DMA)
            t_cpu0 = time.perf_counter()
            if is_bgr:
                np.copyto(self._pinned_np, input_data[:, :, ::-1].transpose(2, 0, 1))
            else:
                np.copyto(self._pinned_np, input_data.transpose(2, 0, 1))
            t_cpu1 = time.perf_counter()
            timings['cpu_copy'].append((t_cpu1 - t_cpu0) * 1000.0)

            with torch.cuda.stream(self.stream_copy):
                e_copy_start.record(self.stream_copy)
                self.gpu_raw_image.copy_(self.pinned_cpu_buffer, non_blocking=True)
                e_copy_end.record(self.stream_copy)
                self.stream_copy.record_event(self.event_copy_done)

            # Stage 1: OWL (context manager required for correct event timing)
            with torch.cuda.stream(self.stream_owl):
                self.stream_owl.wait_event(self.event_copy_done)
                e_owl_start.record(self.stream_owl)
                self.graph_owl.replay()
                e_owl_end.record(self.stream_owl)
                self.stream_owl.record_event(self.event_owl_done)

            # Stage 2: SAM Encoder
            with torch.cuda.stream(self.stream_sam_enc):
                self.stream_sam_enc.wait_event(self.event_copy_done)
                e_se_start.record(self.stream_sam_enc)
                self.graph_sam_enc.replay()
                e_se_end.record(self.stream_sam_enc)
                self.stream_sam_enc.record_event(self.event_sam_enc_done)

            # Stage 3: SAM Decoder (bridge fused into OWL graph)
            with torch.cuda.stream(self.stream_sam_dec):
                self.stream_sam_dec.wait_event(self.event_owl_done)
                self.stream_sam_dec.wait_event(self.event_sam_enc_done)
                e_sd_start.record(self.stream_sam_dec)
                self.graph_sam_dec.replay()
                e_sd_end.record(self.stream_sam_dec)
                self.stream_sam_dec.record_event(self.event_final_done)

            torch.cuda.synchronize()

            # Collect exclusive stage timings
            timings['copy'].append(e_copy_start.elapsed_time(e_copy_end))
            timings['owl'].append(e_owl_start.elapsed_time(e_owl_end))
            timings['sam_enc'].append(e_se_start.elapsed_time(e_se_end))
            timings['sam_dec'].append(e_sd_start.elapsed_time(e_sd_end))
            timings['total'].append(e_frame_start.elapsed_time(e_sd_end))
            # Critical path: copy + max(owl, sam_enc) + sam_dec
            timings['critical_path'].append(
                e_copy_start.elapsed_time(e_copy_end) +
                max(e_owl_start.elapsed_time(e_owl_end),
                    e_se_start.elapsed_time(e_se_end)) +
                e_sd_start.elapsed_time(e_sd_end)
            )

        stats = {}
        for key, vals in timings.items():
            arr = np.array(vals)
            stats[key] = {
                'mean_ms': float(np.mean(arr)),
                'std_ms': float(np.std(arr)),
                'p50_ms': float(np.median(arr)),
                'p99_ms': float(np.percentile(arr, 99))
            }

        return stats


# =============================================================================
# Standalone Test & Benchmark
# =============================================================================
if __name__ == '__main__':
    print("\n" + "=" * 60)
    print("Perception Module v3: Accuracy & Performance Verification")
    print("=" * 60)

    data_dir = "assets/test_data"
    if not os.path.exists(data_dir):
        sys.exit("No test data.")
    rgb_files = [f for f in os.listdir(data_dir) if f.endswith("_rgb.png")]
    latest_file = sorted(rgb_files)[-1]
    print(f"Dataset: {latest_file}")

    img_cv = cv2.imread(os.path.join(data_dir, latest_file))
    query = "a cup which on the bin"

    perception = PerceptionModule(
        owl_model_name="google/owlvit-base-patch32",
        owl_image_engine="weights/owlvit/tensorrt/32_image_encoder_fp16.engine",
        owl_text_engine="weights/owlvit/tensorrt/32_text_encoder_fp32.engine",
        sam_encoder_engine="weights/nanosam/tensorrt/encoder_fp16.engine",
        sam_decoder_engine="weights/nanosam/tensorrt/decoder_fp16.engine",
        score_threshold=0.1
    )

    perception.set_text_query([query])
    perception.setup(img_cv)

    # Warmup
    print("Warming up...")
    for _ in range(20):
        result = perception.infer_tensor(img_cv)
        result.event.synchronize()

    # Throughput Benchmark
    print(f"Measuring Performance (100 runs)...")
    torch.cuda.synchronize()
    start_time = time.perf_counter()
    for _ in range(100):
        result = perception.infer_tensor(img_cv)
    torch.cuda.synchronize()
    total_time = time.perf_counter() - start_time
    fps = 100 / total_time
    print(f"Throughput: {fps:.2f} FPS ({total_time/100*1000:.2f} ms/frame)")

    # Per-stage Exclusive Profiling
    print("\nPer-stage EXCLUSIVE Latency Profiling...")
    stats = perception.profile_latency(img_cv)
    for stage, s in stats.items():
        print(f"  {stage:>15s}: {s['mean_ms']:.2f} +/- {s['std_ms']:.2f} ms "
              f"(p50={s['p50_ms']:.2f}, p99={s['p99_ms']:.2f})")

    # Accuracy Check
    result.event.synchronize()
    score = result.score.item()
    valid = result.valid.item()
    box = result.bbox.cpu().numpy().astype(int)

    print(f"\nScore: {score:.4f}, Valid: {valid}")
    print(f"Box: {box}")

    # Visualization
    vis_img = img_cv.copy()
    mask = result.mask.cpu().numpy().astype(bool)
    if mask.any():
        vis_img[mask] = (vis_img[mask] * 0.5 + np.array([0, 0, 255]) * 0.5).astype(np.uint8)
    cv2.rectangle(vis_img, (box[0], box[1]), (box[2], box[3]), (0, 255, 0), 2)
    cv2.putText(vis_img, f"{score:.2f} | {fps:.1f}FPS | valid={valid}",
                (box[0], box[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imwrite("assets/test_data/perception_module3.jpg", vis_img)
    print("Visualization Saved.")
