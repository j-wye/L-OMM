#!/usr/bin/env python3
"""ROS2 RGB-D camera ingress for L-OMM."""

import sys
import threading
import time
from collections import deque
from typing import Deque, Dict, NamedTuple, Optional, Tuple

import message_filters
import numpy as np
from cv_bridge import CvBridge
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image


# =============================================================================
# Constants & Utilities
# =============================================================================

# ROS header stamp와 local wall-clock의 차이가 이 값 이하면,
# IngressAge/SensorAge를 절대 latency로 해석 가능
STAMP_DOMAIN_OK_THRESHOLD_S = 5.0


def stamp_to_ns(msg_stamp) -> int:
    """ROS stamp → nanosecond integer (dict key용)."""
    return int(msg_stamp.sec) * 1_000_000_000 + int(msg_stamp.nanosec)


def stamp_to_sec(msg_stamp) -> float:
    """ROS stamp → float seconds."""
    return msg_stamp.sec + msg_stamp.nanosec * 1e-9


# =============================================================================
# Data Structures
# =============================================================================

class FrameData(NamedTuple):
    """동기화된 RGB-D 프레임의 불변 컨테이너.
    
    Attributes:
        rgb:                HWC uint8 numpy array (BGR or RGB)
        depth:              HW uint16 numpy array (mm, aligned to color)
        intrinsics:         3x3 float64 camera matrix K
        distortion:         1D float64 distortion coefficients
        distortion_model:   ROS distortion model name
        is_bgr:             True if rgb is BGR channel order
        timestamp:          ROS header stamp (sensor time, seconds)
        received_timestamp: Local wall-clock when frame was stored
    """
    rgb: np.ndarray
    depth: np.ndarray
    intrinsics: np.ndarray
    distortion: np.ndarray
    distortion_model: str
    is_bgr: bool
    timestamp: float
    received_timestamp: float


# =============================================================================
# CameraPreprocessor: ROS2 Camera Frontend
# =============================================================================

class CameraPreprocessor(Node):
    """ROS2 RealSense RGB-D 토픽을 수신하여 최신 프레임을 제공하는 노드.
    
    Design:
        - Latest-frame overwrite: 새 프레임이 도착하면 이전 프레임을 덮어씀
        - Thread-safe: _lock으로 보호, single consumer 패턴
        - Event-based wait: polling 대신 threading.Event로 idle 최소화
        - Direct numpy conversion: known encoding에 대해 cv_bridge 우회
    """

    def __init__(
        self,
        camera_namespace: str = "gripper_camera",
        camera_name: str = "gripper_camera",
        sync_slop: float = 0.01,
        sync_queue: int = 5,
        msg_conversion: str = "direct",
    ):
        """CameraPreprocessor 초기화.
        
        Args:
            camera_namespace: RealSense 카메라 namespace
            camera_name:      RealSense 카메라 이름
            sync_slop:        ApproximateTimeSynchronizer slop (seconds)
            sync_queue:       ApproximateTimeSynchronizer queue size
            msg_conversion:   "direct" (np.frombuffer) or "cv_bridge" (fallback)
        """
        super().__init__("camera_preprocessor")

        self._bridge = CvBridge()
        self._callback_group = ReentrantCallbackGroup()
        self._msg_conversion = msg_conversion
        self._lock = threading.Lock()
        self._frame_ready = threading.Event()
        self._callbacks_enabled = True
        self._sync_processing_enabled = True

        # --- Frame Storage (thread-safe via _lock) ---
        self._latest_rgb: Optional[np.ndarray] = None
        self._latest_depth: Optional[np.ndarray] = None
        self._intrinsics: Optional[np.ndarray] = None
        self._distortion: Optional[np.ndarray] = None
        self._distortion_model: str = ""
        self._is_bgr: bool = False
        self._encoding_detected: bool = False
        self._new_frame_available: bool = False

        # --- Counters (thread-safe via _lock) ---
        self._frame_count: int = 0
        self._rgb_raw_count: int = 0
        self._depth_raw_count: int = 0
        self._overwrite_count: int = 0

        # --- Timestamps ---
        self._timestamp: float = 0.0
        self._received_timestamp: float = 0.0

        # --- Sync dt (RGB-Depth timestamp residual) ---
        self._sync_dt_sum: float = 0.0
        self._sync_dt_max: float = 0.0

        # --- Callback CPU cost ---
        self._callback_proc_time_accum: float = 0.0
        self._callback_proc_max: float = 0.0

        # --- Transport age: sensor stamp → raw callback arrival ---
        self._rgb_transport_age_sum: float = 0.0
        self._rgb_transport_age_max: float = 0.0
        self._depth_transport_age_sum: float = 0.0
        self._depth_transport_age_max: float = 0.0

        # --- Sync wait: raw arrival → sync callback dispatch ---
        self._sync_wait_age_sum: float = 0.0
        self._sync_wait_age_max: float = 0.0
        self._sync_wait_count: int = 0

        # --- Stamp domain validation ---
        self._stamp_domain_offset_abs_sum: float = 0.0
        self._stamp_domain_offset_abs_max: float = 0.0
        self._stamp_domain_samples: int = 0

        # --- Raw arrival timestamp cache (for sync wait calculation) ---
        self._raw_rgb_arrival_by_stamp_ns: Dict[int, float] = {}
        self._raw_depth_arrival_by_stamp_ns: Dict[int, float] = {}
        self._raw_rgb_stamp_order: Deque[int] = deque()
        self._raw_depth_stamp_order: Deque[int] = deque()
        self._arrival_cache_limit: int = max(sync_queue * 8, 64)

        # --- Topic Construction ---
        prefix = f"/{camera_namespace}/{camera_name}"
        rgb_topic = f"{prefix}/color/image_raw"
        depth_topic = f"{prefix}/aligned_depth_to_color/image_raw"
        info_topic = f"{prefix}/aligned_depth_to_color/camera_info"

        # --- QoS: Sensor Data Profile ---
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- CameraInfo (첫 메시지만 처리) ---
        self._info_sub = self.create_subscription(
            CameraInfo,
            info_topic,
            self._info_callback,
            sensor_qos,
            callback_group=self._callback_group,
        )

        # --- Synchronized RGB + Depth ---
        self._rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic,
            qos_profile=sensor_qos,
            callback_group=self._callback_group,
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, depth_topic,
            qos_profile=sensor_qos,
            callback_group=self._callback_group,
        )
        self._rgb_sub.registerCallback(self._rgb_raw_callback)
        self._depth_sub.registerCallback(self._depth_raw_callback)

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._rgb_sub, self._depth_sub],
            queue_size=sync_queue,
            slop=sync_slop,
        )
        self._sync.registerCallback(self._sync_callback)

        self.get_logger().info(
            f"CameraPreprocessor initialized\n"
            f"  RGB:   {rgb_topic}\n"
            f"  Depth: {depth_topic}\n"
            f"  Info:  {info_topic}\n"
            f"  Sync slop: {sync_slop}s, queue: {sync_queue}\n"
            f"  Message conversion: {msg_conversion}"
        )

    # =========================================================================
    # Internal: Message Conversion
    # =========================================================================

    def _trim_arrival_cache(self, cache: Dict[int, float], order: Deque[int]):
        """arrival timestamp cache의 크기를 제한."""
        while len(order) > self._arrival_cache_limit:
            old_key = order.popleft()
            cache.pop(old_key, None)

    def _direct_imgmsg_to_numpy(self, msg: Image) -> Optional[np.ndarray]:
        """cv_bridge를 우회하여 ROS Image → numpy view 직접 생성.
        
        지원 encoding: rgb8, bgr8, mono8, mono16, 16UC1
        지원하지 않는 encoding이면 None을 반환 (cv_bridge fallback).
        
        Returns:
            numpy view (msg.data 위의 zero-copy view) or None
        """
        if self._msg_conversion != "direct":
            return None

        encoding_map = {
            "rgb8": (np.uint8, 3),
            "bgr8": (np.uint8, 3),
            "mono8": (np.uint8, 1),
            "mono16": (np.uint16, 1),
            "16UC1": (np.uint16, 1),
        }
        encoding_spec = encoding_map.get(msg.encoding)
        if encoding_spec is None:
            return None

        dtype, channels = encoding_spec
        itemsize = np.dtype(dtype).itemsize
        step = int(msg.step)
        row_bytes = int(msg.width) * channels * itemsize
        total_bytes = step * int(msg.height)

        # Safety: buffer가 충분한지, stride가 유효한지 확인
        if step < row_bytes or len(msg.data) < total_bytes:
            return None

        # Endianness: big-endian 메시지 + little-endian host → fallback
        if msg.is_bigendian and itemsize > 1 and sys.byteorder == "little":
            return None

        if channels == 1:
            return np.ndarray(
                shape=(msg.height, msg.width),
                dtype=dtype,
                buffer=msg.data,
                strides=(step, itemsize),
            )
        return np.ndarray(
            shape=(msg.height, msg.width, channels),
            dtype=dtype,
            buffer=msg.data,
            strides=(step, channels * itemsize, itemsize),
        )

    def _imgmsg_to_numpy(self, msg: Image) -> np.ndarray:
        """ROS Image → numpy array 변환 (direct → cv_bridge fallback)."""
        direct_result = self._direct_imgmsg_to_numpy(msg)
        if direct_result is not None:
            return direct_result
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # =========================================================================
    # Internal: Callbacks
    # =========================================================================

    def _update_stamp_domain_stats(self, arrival_wall: float, stamp_sec: float):
        """wall-clock과 ROS stamp의 domain 차이를 누적 (lock 내부에서 호출)."""
        offset_abs = abs(arrival_wall - stamp_sec)
        self._stamp_domain_offset_abs_sum += offset_abs
        self._stamp_domain_offset_abs_max = max(self._stamp_domain_offset_abs_max, offset_abs)
        self._stamp_domain_samples += 1

    def _rgb_raw_callback(self, msg: Image):
        """raw RGB 메시지 도착 시: 카운터 증가 + transport age 기록."""
        if not self._callbacks_enabled:
            return
        arrival_wall = time.time()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        stamp_sec = stamp_to_sec(msg.header.stamp)
        transport_age = max(arrival_wall - stamp_sec, 0.0)
        with self._lock:
            self._rgb_raw_count += 1
            self._rgb_transport_age_sum += transport_age
            self._rgb_transport_age_max = max(self._rgb_transport_age_max, transport_age)
            self._raw_rgb_arrival_by_stamp_ns[stamp_ns] = arrival_wall
            self._raw_rgb_stamp_order.append(stamp_ns)
            self._trim_arrival_cache(self._raw_rgb_arrival_by_stamp_ns, self._raw_rgb_stamp_order)
            self._update_stamp_domain_stats(arrival_wall, stamp_sec)

    def _depth_raw_callback(self, msg: Image):
        """raw Depth 메시지 도착 시: 카운터 증가 + transport age 기록."""
        if not self._callbacks_enabled:
            return
        arrival_wall = time.time()
        stamp_ns = stamp_to_ns(msg.header.stamp)
        stamp_sec = stamp_to_sec(msg.header.stamp)
        transport_age = max(arrival_wall - stamp_sec, 0.0)
        with self._lock:
            self._depth_raw_count += 1
            self._depth_transport_age_sum += transport_age
            self._depth_transport_age_max = max(self._depth_transport_age_max, transport_age)
            self._raw_depth_arrival_by_stamp_ns[stamp_ns] = arrival_wall
            self._raw_depth_stamp_order.append(stamp_ns)
            self._trim_arrival_cache(self._raw_depth_arrival_by_stamp_ns, self._raw_depth_stamp_order)
            self._update_stamp_domain_stats(arrival_wall, stamp_sec)

    def _info_callback(self, msg: CameraInfo):
        """CameraInfo 수신: intrinsics + distortion 저장 (첫 1회만)."""
        if self._intrinsics is not None:
            return

        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self._intrinsics = K
        self._distortion = np.array(msg.d, dtype=np.float64)
        self._distortion_model = msg.distortion_model

        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]
        self.get_logger().info(
            "Camera intrinsics received:\n"
            f"  fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}\n"
            f"  distortion_model={self._distortion_model}, "
            f"distortion_len={self._distortion.size}"
        )

    def _sync_callback(self, rgb_msg: Image, depth_msg: Image):
        """동기화된 RGB-Depth pair 수신: numpy 변환 + latest-frame 저장."""
        if not self._callbacks_enabled or not self._sync_processing_enabled:
            return
        cb_t0 = time.perf_counter()
        cb_start_wall = time.time()

        # 첫 프레임에서 encoding 자동 감지
        if not self._encoding_detected:
            self._is_bgr = rgb_msg.encoding in ("bgr8", "bgr16")
            enc_str = "BGR" if self._is_bgr else "RGB"
            self.get_logger().info(
                f"Color encoding detected: {rgb_msg.encoding} -> {enc_str}\n"
                f"  Depth encoding: {depth_msg.encoding}\n"
                f"  Resolution: {rgb_msg.width}x{rgb_msg.height}"
            )
            self._encoding_detected = True

        # ROS Image → numpy (direct path 우선, cv_bridge fallback)
        rgb = self._imgmsg_to_numpy(rgb_msg)
        depth = self._imgmsg_to_numpy(depth_msg)

        # PerceptionModule은 C-contiguous numpy buffer를 요구
        if not rgb.flags.c_contiguous:
            rgb = np.ascontiguousarray(rgb)
        if not depth.flags.c_contiguous:
            depth = np.ascontiguousarray(depth)

        # Timestamp 추출
        rgb_stamp = stamp_to_sec(rgb_msg.header.stamp)
        depth_stamp = stamp_to_sec(depth_msg.header.stamp)
        rgb_stamp_ns = stamp_to_ns(rgb_msg.header.stamp)
        depth_stamp_ns = stamp_to_ns(depth_msg.header.stamp)
        stamp = rgb_stamp
        sync_dt = abs(rgb_stamp - depth_stamp)
        received_timestamp = time.time()
        callback_proc_time = time.perf_counter() - cb_t0

        # Thread-safe 저장 (latest-frame overwrite)
        with self._lock:
            # Sync wait 계산: raw arrival → sync callback 시작
            rgb_arrival_wall = self._raw_rgb_arrival_by_stamp_ns.pop(rgb_stamp_ns, None)
            depth_arrival_wall = self._raw_depth_arrival_by_stamp_ns.pop(depth_stamp_ns, None)
            if rgb_arrival_wall is not None and depth_arrival_wall is not None:
                sync_wait_age = max(cb_start_wall - max(rgb_arrival_wall, depth_arrival_wall), 0.0)
                self._sync_wait_age_sum += sync_wait_age
                self._sync_wait_age_max = max(self._sync_wait_age_max, sync_wait_age)
                self._sync_wait_count += 1

            # Overwrite 감지
            if self._new_frame_available:
                self._overwrite_count += 1

            # 프레임 저장
            self._latest_rgb = rgb
            self._latest_depth = depth
            self._new_frame_available = True
            self._frame_count += 1
            self._timestamp = stamp
            self._received_timestamp = received_timestamp
            self._sync_dt_sum += sync_dt
            self._sync_dt_max = max(self._sync_dt_max, sync_dt)
            self._callback_proc_time_accum += callback_proc_time
            self._callback_proc_max = max(self._callback_proc_max, callback_proc_time)
            self._frame_ready.set()

    # =========================================================================
    # Public API: Frame Access
    # =========================================================================

    def get_frame(self) -> Optional[FrameData]:
        """최신 동기화 프레임 반환 (non-blocking).
        
        마지막 호출 이후 새 프레임이 없으면 None 반환.
        Thread-safe: single consumer 패턴.
        """
        with self._lock:
            if not self._new_frame_available:
                return None
            self._new_frame_available = False
            self._frame_ready.clear()
            return FrameData(
                rgb=self._latest_rgb,
                depth=self._latest_depth,
                intrinsics=self._intrinsics,
                distortion=self._distortion,
                distortion_model=self._distortion_model,
                is_bgr=self._is_bgr,
                timestamp=self._timestamp,
                received_timestamp=self._received_timestamp,
            )

    def wait_for_frame(self, timeout: float = 0.1) -> Optional[FrameData]:
        """새 프레임 도착까지 event-based 대기 후 반환."""
        if not self._frame_ready.wait(timeout=timeout):
            return None
        return self.get_frame()

    # =========================================================================
    # Public API: Lifecycle Control
    # =========================================================================

    def pause_sync_processing(self):
        """모델 초기화와 warmup 중 callback의 무거운 경로를 일시 중단."""
        with self._lock:
            self._sync_processing_enabled = False

    def resume_sync_processing(self):
        """일시 중단했던 sync processing 재개."""
        with self._lock:
            self._sync_processing_enabled = True

    def suspend_callbacks(self):
        """모든 camera callback을 일시 정지 (모델 초기화/벤치마크 시 간섭 방지)."""
        with self._lock:
            self._callbacks_enabled = False
            self._sync_processing_enabled = False

    def resume_callbacks(self):
        """정지했던 camera callback을 재개."""
        with self._lock:
            self._callbacks_enabled = True
            self._sync_processing_enabled = True

    def clear_pending_frame(self):
        """pending frame handoff 상태를 비움."""
        with self._lock:
            self._new_frame_available = False
            self._frame_ready.clear()

    def reset_runtime_counters(self):
        """warmup 이후 live session용 누적 지표를 초기화."""
        with self._lock:
            self._new_frame_available = False
            self._frame_ready.clear()
            self._frame_count = 0
            self._rgb_raw_count = 0
            self._depth_raw_count = 0
            self._overwrite_count = 0
            self._sync_dt_sum = 0.0
            self._sync_dt_max = 0.0
            self._callback_proc_time_accum = 0.0
            self._callback_proc_max = 0.0
            self._rgb_transport_age_sum = 0.0
            self._rgb_transport_age_max = 0.0
            self._depth_transport_age_sum = 0.0
            self._depth_transport_age_max = 0.0
            self._sync_wait_age_sum = 0.0
            self._sync_wait_age_max = 0.0
            self._sync_wait_count = 0
            self._raw_rgb_arrival_by_stamp_ns.clear()
            self._raw_depth_arrival_by_stamp_ns.clear()
            self._raw_rgb_stamp_order.clear()
            self._raw_depth_stamp_order.clear()

    def wait_for_ready(self, timeout: float = 30.0) -> bool:
        """Intrinsics와 첫 프레임 수신까지 블로킹 대기.
        
        Args:
            timeout: 최대 대기 시간 (초)
        Returns:
            True if ready, False if timeout
        """
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if self.is_ready:
                return True
            time.sleep(0.05)
        return False

    # =========================================================================
    # Public API: Properties (Observability)
    # =========================================================================

    @property
    def is_ready(self) -> bool:
        """Intrinsics + 첫 프레임이 수신되어 데이터 제공 가능한 상태인지."""
        with self._lock:
            return (
                self._intrinsics is not None
                and self._distortion is not None
                and self._latest_rgb is not None
                and self._latest_depth is not None
            )

    @property
    def frame_count(self) -> int:
        """총 수신 동기화 프레임 수."""
        with self._lock:
            return self._frame_count

    @property
    def overwrite_count(self) -> int:
        """소비 전에 최신 프레임으로 덮어쓴 횟수."""
        with self._lock:
            return self._overwrite_count

    @property
    def rgb_raw_count(self) -> int:
        """수신한 raw RGB 메시지 누적 개수."""
        with self._lock:
            return self._rgb_raw_count

    @property
    def depth_raw_count(self) -> int:
        """수신한 raw Depth 메시지 누적 개수."""
        with self._lock:
            return self._depth_raw_count

    @property
    def sync_dt_mean_ms(self) -> float:
        """RGB-Depth timestamp residual 평균 (ms)."""
        with self._lock:
            if self._frame_count == 0:
                return 0.0
            return (self._sync_dt_sum / self._frame_count) * 1000.0

    @property
    def sync_dt_max_ms(self) -> float:
        """RGB-Depth timestamp residual 최대값 (ms)."""
        with self._lock:
            return self._sync_dt_max * 1000.0

    @property
    def callback_proc_time_accum(self) -> float:
        """Sync callback 누적 CPU 처리 시간 (seconds)."""
        with self._lock:
            return self._callback_proc_time_accum

    @property
    def callback_proc_max_ms(self) -> float:
        """Sync callback 단일 호출 최대 CPU 비용 (ms)."""
        with self._lock:
            return self._callback_proc_max * 1000.0

    @property
    def rgb_transport_age_mean_ms(self) -> float:
        """RGB transport age 평균 (sensor stamp → raw arrival, ms)."""
        with self._lock:
            if self._rgb_raw_count == 0:
                return 0.0
            return (self._rgb_transport_age_sum / self._rgb_raw_count) * 1000.0

    @property
    def rgb_transport_age_max_ms(self) -> float:
        """RGB transport age 최대값 (ms)."""
        with self._lock:
            return self._rgb_transport_age_max * 1000.0

    @property
    def depth_transport_age_mean_ms(self) -> float:
        """Depth transport age 평균 (sensor stamp → raw arrival, ms)."""
        with self._lock:
            if self._depth_raw_count == 0:
                return 0.0
            return (self._depth_transport_age_sum / self._depth_raw_count) * 1000.0

    @property
    def depth_transport_age_max_ms(self) -> float:
        """Depth transport age 최대값 (ms)."""
        with self._lock:
            return self._depth_transport_age_max * 1000.0

    @property
    def sync_wait_age_mean_ms(self) -> float:
        """ATS sync wait 평균 (raw arrival → sync callback, ms)."""
        with self._lock:
            if self._sync_wait_count == 0:
                return 0.0
            return (self._sync_wait_age_sum / self._sync_wait_count) * 1000.0

    @property
    def sync_wait_age_max_ms(self) -> float:
        """ATS sync wait 최대값 (ms)."""
        with self._lock:
            return self._sync_wait_age_max * 1000.0

    @property
    def stamp_domain_offset_mean_ms(self) -> float:
        """ROS stamp과 local wall-clock의 절대 차이 평균 (ms)."""
        with self._lock:
            if self._stamp_domain_samples == 0:
                return 0.0
            return (self._stamp_domain_offset_abs_sum / self._stamp_domain_samples) * 1000.0

    @property
    def stamp_domain_offset_max_ms(self) -> float:
        """ROS stamp과 local wall-clock의 절대 차이 최대값 (ms)."""
        with self._lock:
            return self._stamp_domain_offset_abs_max * 1000.0

    @property
    def stamp_domain_status(self) -> str:
        """stamp domain 판정: OK / APPROX / UNKNOWN."""
        with self._lock:
            if self._stamp_domain_samples == 0:
                return "UNKNOWN"
            return (
                "OK"
                if self._stamp_domain_offset_abs_max <= STAMP_DOMAIN_OK_THRESHOLD_S
                else "APPROX"
            )

    @property
    def image_shape(self) -> Optional[Tuple[int, int]]:
        """현재 프레임 해상도 (H, W). 프레임 미수신 시 None."""
        with self._lock:
            if self._latest_rgb is not None:
                return self._latest_rgb.shape[:2]
            return None


