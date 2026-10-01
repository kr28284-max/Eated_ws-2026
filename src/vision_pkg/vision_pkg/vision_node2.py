"""Always-visible, high-frame-rate frontend for on-demand RGB-D vision."""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridgeError
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from vision_pkg.vision_node import FrameObservation
from vision_pkg.vision_node import VisionNode
from vision_pkg.vision_utils import Detection


@dataclass(frozen=True)
class OverlaySnapshot:
    """Latest lightweight inference result drawn over live RGB frames."""

    target_class: str
    detection: Optional[Detection]
    depth_m: Optional[float]
    base_point: Optional[tuple[float, float, float]]
    error: str
    stable_time: float
    final_point: Optional[tuple[float, float, float]]
    updated_at: float


class VisionNode2(VisionNode):
    """Keep the camera preview live while running YOLO only on request."""

    def __init__(self) -> None:
        self._overlay_lock = threading.Lock()
        self._overlay_snapshot: Optional[OverlaySnapshot] = None
        self._display_lock = threading.Lock()
        self._latest_display_image: Optional[np.ndarray] = None
        self._last_display_publish_at = 0.0
        self._display_frame_count = 0
        self._display_fps = 0.0
        self._fps_started_at = time.monotonic()
        self._window_initialized = False
        self._window_available = True
        self._window_warning_logged = False
        self._inference_warmup_lock = threading.Lock()
        self._inference_warmed_up = False
        self._gesture_lock = threading.Lock()
        self._control_state = ''
        self._open_palm_detected = False
        self._open_palm_updated_at = 0.0
        self._release_remaining_seconds: Optional[float] = None
        self._countdown_updated_at = 0.0

        super().__init__()

        # Image streams should not wait for reliable delivery. Replacing the
        # original publisher with sensor-data QoS keeps slow viewers from
        # applying backpressure to the camera callback.
        self.destroy_publisher(self.debug_image_publisher)
        self.debug_image_publisher = self.create_publisher(
            Image,
            '/vision/debug_image',
            qos_profile_sensor_data,
        )
        self.live_rgb_subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.live_rgb_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.control_state_subscription = self.create_subscription(
            String,
            '/control/state',
            self.control_state_callback,
            state_qos,
            callback_group=self.callback_group,
        )
        self.palm_subscription = self.create_subscription(
            Bool,
            '/vision/open_palm',
            self.open_palm_callback,
            10,
            callback_group=self.callback_group,
        )
        self.countdown_subscription = self.create_subscription(
            Float32,
            '/control/release_countdown_sec',
            self.release_countdown_callback,
            10,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'VisionNode2 live preview enabled: '
            f'publish_rate={self.debug_display_rate_hz:.1f} Hz, '
            f'window={self.show_debug_window}. YOLO remains on-demand.'
        )

    def _declare_parameters(self) -> None:
        super()._declare_parameters()
        # This is a ceiling, not a forced frame rate. Keeping it above the
        # D435i's usual 30 FPS prevents timestamp jitter from dropping every
        # other frame.
        self.declare_parameter('debug_display_rate_hz', 60.0)
        self.declare_parameter('show_debug_window', True)
        self.declare_parameter(
            'debug_window_title',
            'EATED Vision - live / on-demand YOLO',
        )
        self.declare_parameter('overlay_max_age', 1.0)

    def _read_parameters(self) -> None:
        super()._read_parameters()
        self.debug_display_rate_hz = float(
            self.get_parameter('debug_display_rate_hz').value
        )
        self.show_debug_window = bool(
            self.get_parameter('show_debug_window').value
        )
        self.debug_window_title = str(
            self.get_parameter('debug_window_title').value
        )
        self.overlay_max_age = float(
            self.get_parameter('overlay_max_age').value
        )

    def _validate_parameters(self) -> None:
        super()._validate_parameters()
        if self.debug_display_rate_hz <= 0.0:
            raise ValueError('debug_display_rate_hz must be positive')
        if self.overlay_max_age <= 0.0:
            raise ValueError('overlay_max_age must be positive')
        if not self.debug_window_title.strip():
            raise ValueError('debug_window_title must not be empty')

    def _set_recognition_active(self, target_class: str) -> None:
        with self._overlay_lock:
            self._overlay_snapshot = None
        super()._set_recognition_active(target_class)

    def reset_recognition_state(self) -> None:
        super().reset_recognition_state()
        with self._overlay_lock:
            self._overlay_snapshot = None

    def _run_recognition(self, goal_handle, target_class: str):
        """Warm up CUDA once before the recognition timeout starts."""
        self._ensure_inference_warmed_up()
        return super()._run_recognition(goal_handle, target_class)

    def _ensure_inference_warmed_up(self) -> None:
        """Initialize the selected inference backend once, on first command."""
        if self._inference_warmed_up:
            return

        with self._inference_warmup_lock:
            if self._inference_warmed_up:
                return

            device = 0 if torch.cuda.is_available() else 'cpu'
            device_name = (
                torch.cuda.get_device_name(0)
                if device == 0
                else 'CPU'
            )
            self.get_logger().info(
                'Preparing YOLO inference backend before starting the '
                f'recognition timeout: device={device_name}'
            )
            started_at = time.monotonic()
            warmup_image = np.zeros((640, 640, 3), dtype=np.uint8)
            self.model.predict(
                source=warmup_image,
                conf=self.confidence_threshold,
                device=device,
                verbose=False,
            )
            if device == 0:
                torch.cuda.synchronize()
            self._inference_warmed_up = True
            elapsed = time.monotonic() - started_at
            self.get_logger().info(
                f'YOLO inference backend ready in {elapsed:.2f}s; '
                'recognition timeout starts now'
            )

    def publish_debug_image(
        self,
        observation: FrameObservation,
        stable_time: float,
        final_point: Optional[tuple[float, float, float]] = None,
    ) -> None:
        """Cache inference metadata; live RGB callback owns frame output."""
        with self._state_lock:
            target_class = self.target_class
        snapshot = OverlaySnapshot(
            target_class=target_class,
            detection=observation.detection,
            depth_m=observation.depth_m,
            base_point=observation.base_point,
            error=observation.error,
            stable_time=stable_time,
            final_point=final_point,
            updated_at=time.monotonic(),
        )
        with self._overlay_lock:
            self._overlay_snapshot = snapshot

    def live_rgb_callback(self, rgb_msg: Image) -> None:
        """Publish and stage the newest RGB image at the display frame rate."""
        now = time.monotonic()
        minimum_interval = 1.0 / self.debug_display_rate_hz
        if now - self._last_display_publish_at < minimum_interval:
            return
        self._last_display_publish_at = now

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8',
            )
        except CvBridgeError as exc:
            self._log_frame_error(f'Live RGB conversion failed: {exc}')
            return

        image = np.asarray(rgb_image).copy()
        self._draw_live_overlay(image, now)
        self._draw_hand_overlay(image, now)
        self._update_display_fps(now)

        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8',
            )
        except CvBridgeError as exc:
            self._log_frame_error(f'Debug image conversion failed: {exc}')
            return
        debug_message.header = rgb_msg.header
        self.debug_image_publisher.publish(debug_message)

        with self._display_lock:
            self._latest_display_image = image

    def control_state_callback(self, msg: String) -> None:
        """Follow the controller's current delivery phase."""
        with self._gesture_lock:
            if msg.data != self._control_state:
                self._open_palm_detected = False
                self._release_remaining_seconds = None
            self._control_state = msg.data

    def open_palm_callback(self, msg: Bool) -> None:
        """Cache the latest MediaPipe hand decision."""
        with self._gesture_lock:
            self._open_palm_detected = bool(msg.data)
            self._open_palm_updated_at = time.monotonic()

    def release_countdown_callback(self, msg: Float32) -> None:
        """Cache the controller's actual time remaining before release."""
        with self._gesture_lock:
            self._release_remaining_seconds = max(0.0, float(msg.data))
            self._countdown_updated_at = time.monotonic()

    def _draw_hand_overlay(self, image: np.ndarray, now: float) -> None:
        with self._gesture_lock:
            state = self._control_state
            is_open = (
                self._open_palm_detected
                and now - self._open_palm_updated_at <= 0.5
            )
            remaining = self._release_remaining_seconds
            countdown_fresh = now - self._countdown_updated_at <= 0.5

        if state == 'DRIVE_TO_PLACE':
            if is_open:
                time_label = (
                    f'Release in {remaining:.1f} s'
                    if remaining is not None and countdown_fresh
                    else 'Measuring hold time...'
                )
                lines = ['OPEN PALM DETECTED', time_label]
                color = (0, 255, 0)
            else:
                lines = ['Show an open palm to release',
                         'Hold for 2.0 s']
                color = (0, 200, 255)
        elif state == 'PLACING':
            lines = ['OPEN PALM CONFIRMED', 'Opening gripper...']
            color = (0, 255, 0)
        elif state == 'RETURN_HOME':
            lines = ['Returning HOME']
            color = (0, 255, 0)
        else:
            return

        panel_top = max(0, image.shape[0] - 86)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (0, panel_top),
            (min(image.shape[1], 560), image.shape[0]),
            (0, 0, 0),
            -1,
        )
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0.0, image)
        for index, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (12, panel_top + 34 + index * 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                color,
                2,
                cv2.LINE_AA,
            )

    def _draw_live_overlay(self, image: np.ndarray, now: float) -> None:
        with self._state_lock:
            recognition_active = self.recognition_active
            target_class = self.target_class
        with self._overlay_lock:
            snapshot = self._overlay_snapshot

        if not recognition_active:
            lines = [
                'IDLE - waiting for /find_object command',
                f'preview: {self._display_fps:.1f} FPS | YOLO: OFF',
            ]
            self._draw_status_panel(image, lines, (0, 255, 0))
            return

        valid_snapshot = (
            snapshot is not None
            and snapshot.target_class == target_class
            and now - snapshot.updated_at <= self.overlay_max_age
        )
        if not valid_snapshot:
            lines = [
                f'DETECTING: {target_class}',
                f'preview: {self._display_fps:.1f} FPS | YOLO: ON',
                'waiting for inference result...',
            ]
            self._draw_status_panel(image, lines, (0, 200, 255))
            return

        self._draw_snapshot(image, snapshot)

    def _draw_snapshot(
        self,
        image: np.ndarray,
        snapshot: OverlaySnapshot,
    ) -> None:
        detection = snapshot.detection
        status_lines = [
            f'DETECTING: {snapshot.target_class}',
            f'preview: {self._display_fps:.1f} FPS | YOLO: ON',
        ]
        self._draw_status_panel(image, status_lines, (0, 200, 255))

        if detection is None:
            self._draw_text_lines(
                image,
                [snapshot.error or 'target not detected'],
                10,
                78,
                color=(0, 200, 255),
            )
            return

        x1 = int(round(detection.x1))
        y1 = int(round(detection.y1))
        x2 = int(round(detection.x2))
        y2 = int(round(detection.y2))
        u, v = detection.center
        cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.circle(image, (u, v), 4, (0, 0, 255), -1)
        labels = [
            f'{detection.class_name} {detection.confidence:.2f}',
            self._depth_label(snapshot.depth_m),
            self._base_point_label(snapshot.base_point),
            (
                f'accumulated: {snapshot.stable_time:.1f} / '
                f'{self.stable_detection_duration:.1f} s'
            ),
        ]
        self._draw_text_lines(image, labels, x1, max(95, y1 - 70))

        if snapshot.error:
            self._draw_text_lines(
                image,
                [snapshot.error],
                10,
                image.shape[0] - 20,
                color=(0, 0, 255),
            )
        if snapshot.final_point is not None:
            point = snapshot.final_point
            self._draw_text_lines(
                image,
                [
                    'FINAL base_link median XYZ:',
                    f'{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f} m',
                ],
                10,
                95,
                color=(0, 255, 0),
            )

    @staticmethod
    def _draw_status_panel(
        image: np.ndarray,
        lines: list[str],
        color: tuple[int, int, int],
    ) -> None:
        panel_height = 12 + 25 * len(lines)
        overlay = image.copy()
        cv2.rectangle(
            overlay,
            (0, 0),
            (min(image.shape[1], 520), panel_height),
            (0, 0, 0),
            -1,
        )
        cv2.addWeighted(overlay, 0.55, image, 0.45, 0.0, image)
        VisionNode._draw_text_lines(image, lines, 10, 24, color=color)

    def _update_display_fps(self, now: float) -> None:
        self._display_frame_count += 1
        elapsed = now - self._fps_started_at
        if elapsed >= 1.0:
            self._display_fps = self._display_frame_count / elapsed
            self._display_frame_count = 0
            self._fps_started_at = now

    def update_debug_window(self) -> None:
        """Refresh the OpenCV window from the process main thread."""
        if not self.show_debug_window or not self._window_available:
            return
        if not (
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
        ):
            if not self._window_warning_logged:
                self.get_logger().warn(
                    'No desktop display detected; OpenCV window disabled. '
                    'The /vision/debug_image topic remains available.'
                )
                self._window_warning_logged = True
            self._window_available = False
            return

        with self._display_lock:
            if self._latest_display_image is None:
                image = np.zeros((360, 640, 3), dtype=np.uint8)
                self._draw_text_lines(
                    image,
                    ['Waiting for RealSense RGB stream...'],
                    30,
                    180,
                    color=(0, 200, 255),
                )
            else:
                image = self._latest_display_image.copy()

        try:
            if not self._window_initialized:
                cv2.namedWindow(self.debug_window_title, cv2.WINDOW_NORMAL)
                self._window_initialized = True
            cv2.imshow(self.debug_window_title, image)
            cv2.waitKey(1)
        except cv2.error as exc:
            if not self._window_warning_logged:
                self.get_logger().warn(
                    f'OpenCV window disabled: {exc}. '
                    'The /vision/debug_image topic remains available.'
                )
                self._window_warning_logged = True
            self._window_available = False

    def destroy_node(self) -> None:
        if self._window_initialized:
            try:
                cv2.destroyWindow(self.debug_window_title)
                cv2.waitKey(1)
            except cv2.error:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode2()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            node.update_debug_window()
            time.sleep(1.0 / 60.0 if node.show_debug_window else 0.1)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
