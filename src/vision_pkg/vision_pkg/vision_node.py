"""On-demand YOLO RGB-D localization exposed as a ROS 2 action."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from eated_interfaces.action import FindObject
from geometry_msgs.msg import PointStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO

from vision_pkg.vision_utils import Detection
from vision_pkg.vision_utils import deproject_pixel
from vision_pkg.vision_utils import get_median_depth
from vision_pkg.vision_utils import median_localization
from vision_pkg.vision_utils import select_target_detection


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole parameters copied from a color CameraInfo message."""

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class FrameObservation:
    """Detection and localization result for one synchronized frame pair."""

    rgb_image: np.ndarray
    rgb_message: Image
    detection: Optional[Detection] = None
    depth_m: Optional[float] = None
    camera_point: Optional[tuple[float, float, float]] = None
    base_point: Optional[tuple[float, float, float]] = None
    error: str = ''


class VisionNode(Node):
    """Localize a requested YOLO class in base_link on demand."""

    def __init__(self) -> None:
        super().__init__('vision_node')

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.bridge = CvBridge()
        self.model = self.load_model(self.model_path)
        self.class_names_by_id = self._normalize_model_names(
            self.model.names
        )
        self.available_classes = set(self.class_names_by_id.values())

        self.callback_group = ReentrantCallbackGroup()
        self._frame_condition = threading.Condition()
        self._latest_pair: Optional[tuple[Image, Image]] = None
        self._latest_frame_sequence = 0
        self._latest_frame_received_at = 0.0
        self._intrinsics_lock = threading.Lock()
        self._camera_intrinsics: Optional[CameraIntrinsics] = None
        self._state_lock = threading.Lock()
        self._goal_reserved = False
        self.recognition_active = False
        self.target_class = ''
        self._warned_frame_mismatch = False
        self._last_detail_log_time = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.rgb_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data,
            callback_group=self.callback_group,
        )
        self.image_synchronizer = (
            message_filters.ApproximateTimeSynchronizer(
                [self.rgb_subscriber, self.depth_subscriber],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop,
            )
        )
        self.image_synchronizer.registerCallback(self.image_pair_callback)

        self.debug_image_publisher = self.create_publisher(
            Image,
            '/vision/debug_image',
            10,
        )
        self.action_server = ActionServer(
            self,
            FindObject,
            '/find_object',
            execute_callback=self.execute_find_object,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Vision action server ready: '
            f'model={self.model_path}, '
            f'classes={sorted(self.available_classes)}'
        )
        self.get_logger().info(
            f'RGB={self.rgb_topic}, depth={self.depth_topic}, '
            f'CameraInfo={self.camera_info_topic}'
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter(
            'rgb_topic',
            '/realsense/d435i/color/image_raw',
        )
        self.declare_parameter(
            'depth_topic',
            '/realsense/d435i/aligned_depth_to_color/image_raw',
        )
        self.declare_parameter(
            'camera_info_topic',
            '/realsense/d435i/color/camera_info',
        )
        self.declare_parameter(
            'camera_frame',
            'd435i_color_optical_frame',
        )
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('stable_detection_duration', 3.0)
        self.declare_parameter('recognition_timeout', 10.0)
        self.declare_parameter('depth_window_size', 7)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('max_frame_age', 0.5)
        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('min_valid_samples', 10)
        self.declare_parameter('max_inference_rate_hz', 10.0)
        self.declare_parameter('sync_queue_size', 5)
        self.declare_parameter('sync_slop', 0.05)

    def _read_parameters(self) -> None:
        self.model_path = str(self.get_parameter('model_path').value)
        self.confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.camera_frame = str(self.get_parameter('camera_frame').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.stable_detection_duration = float(
            self.get_parameter('stable_detection_duration').value
        )
        self.recognition_timeout = float(
            self.get_parameter('recognition_timeout').value
        )
        self.depth_window_size = int(
            self.get_parameter('depth_window_size').value
        )
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.max_frame_age = float(
            self.get_parameter('max_frame_age').value
        )
        self.tf_timeout = float(self.get_parameter('tf_timeout').value)
        self.min_valid_samples = int(
            self.get_parameter('min_valid_samples').value
        )
        self.max_inference_rate_hz = float(
            self.get_parameter('max_inference_rate_hz').value
        )
        self.sync_queue_size = int(
            self.get_parameter('sync_queue_size').value
        )
        self.sync_slop = float(self.get_parameter('sync_slop').value)

    def _validate_parameters(self) -> None:
        if not self.model_path.strip():
            raise ValueError('model_path parameter must not be empty')
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError('confidence_threshold must be between 0 and 1')
        if self.stable_detection_duration <= 0.0:
            raise ValueError('stable_detection_duration must be positive')
        if self.recognition_timeout <= self.stable_detection_duration:
            raise ValueError(
                'recognition_timeout must exceed stable_detection_duration'
            )
        if self.depth_window_size < 1 or self.depth_window_size % 2 == 0:
            raise ValueError(
                'depth_window_size must be a positive odd integer'
            )
        if self.depth_scale <= 0.0:
            raise ValueError('depth_scale must be positive')
        if self.max_frame_age <= 0.0:
            raise ValueError('max_frame_age must be positive')
        if self.tf_timeout < 0.0:
            raise ValueError('tf_timeout must not be negative')
        if self.min_valid_samples < 1:
            raise ValueError('min_valid_samples must be at least one')
        if self.max_inference_rate_hz <= 0.0:
            raise ValueError('max_inference_rate_hz must be positive')
        if self.sync_queue_size < 1:
            raise ValueError('sync_queue_size must be at least one')
        if self.sync_slop < 0.0:
            raise ValueError('sync_slop must not be negative')

    def load_model(self, model_path: str) -> YOLO:
        """Load the YOLO model once during node initialization."""
        resolved_path = Path(model_path).expanduser().resolve()
        if not resolved_path.is_file():
            raise FileNotFoundError(f'YOLO model not found: {resolved_path}')
        self.model_path = str(resolved_path)
        self.get_logger().info(f'Loading YOLO model: {resolved_path}')
        return YOLO(str(resolved_path))

    @staticmethod
    def _normalize_model_names(names) -> dict[int, str]:
        if isinstance(names, dict):
            items = names.items()
        else:
            items = enumerate(names)
        return {
            int(class_id): str(class_name).strip().lower()
            for class_id, class_name in items
        }

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Cache color-camera pinhole intrinsics."""
        if len(msg.k) != 9:
            self.get_logger().error(
                'CameraInfo K matrix does not have 9 values'
            )
            return
        intrinsics = CameraIntrinsics(
            fx=float(msg.k[0]),
            fy=float(msg.k[4]),
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
        )
        if intrinsics.fx <= 0.0 or intrinsics.fy <= 0.0:
            self.get_logger().error(
                'CameraInfo contains invalid focal lengths'
            )
            return
        with self._intrinsics_lock:
            self._camera_intrinsics = intrinsics

    def image_pair_callback(self, rgb_msg: Image, depth_msg: Image) -> None:
        """Keep only the newest synchronized RGB and aligned-depth pair."""
        with self._frame_condition:
            self._latest_pair = (rgb_msg, depth_msg)
            self._latest_frame_sequence += 1
            self._latest_frame_received_at = time.monotonic()
            self._frame_condition.notify_all()

    def validate_target_class(self, target_class: str) -> bool:
        """Return whether the normalized target exists in the loaded model."""
        return target_class.strip().lower() in self.available_classes

    def goal_callback(self, goal_request: FindObject.Goal) -> GoalResponse:
        """Allow at most one active or pending recognition request."""
        with self._state_lock:
            if self._goal_reserved:
                self.get_logger().warn(
                    'Rejecting /find_object goal because recognition is busy'
                )
                return GoalResponse.REJECT
            self._goal_reserved = True
        self.get_logger().info(
            f'Target requested: {goal_request.target_class}'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        """Accept cancellation; the execute loop performs state cleanup."""
        self.get_logger().info('Cancel requested for /find_object')
        return CancelResponse.ACCEPT

    def reset_recognition_state(self) -> None:
        """Disable inference and release the single-goal reservation."""
        with self._state_lock:
            self.recognition_active = False
            self.target_class = ''
            self._goal_reserved = False

    def _set_recognition_active(self, target_class: str) -> None:
        with self._state_lock:
            self.recognition_active = True
            self.target_class = target_class

    def execute_find_object(self, goal_handle):
        """Find, stabilize, and localize the requested target class."""
        target_class = goal_handle.request.target_class.strip().lower()
        result = FindObject.Result()

        try:
            if not self.validate_target_class(target_class):
                result.message = f'Unknown target class: {target_class}'
                goal_handle.abort()
                self.get_logger().warn(result.message)
                return result

            self._set_recognition_active(target_class)
            return self._run_recognition(goal_handle, target_class)
        except Exception as exc:
            result.message = (
                'Vision recognition failed: '
                f'{type(exc).__name__}: {exc}'
            )
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result
        finally:
            self.reset_recognition_state()

    def _run_recognition(self, goal_handle, target_class: str):
        started_at = time.monotonic()
        stable_started_at: Optional[float] = None
        samples: list[tuple[float, float, float, float]] = []
        last_processed_sequence = -1
        next_inference_at = started_at
        last_feedback_at = 0.0
        inference_interval = 1.0 / self.max_inference_rate_hz

        while rclpy.ok():
            now = time.monotonic()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._failure_result('Goal canceled.')

            if now - started_at >= self.recognition_timeout:
                goal_handle.abort()
                return self._failure_result(
                    'Target was not stably detected before timeout.'
                )

            if now < next_inference_at:
                time.sleep(min(0.02, next_inference_at - now))
                continue

            frame_data = self._wait_for_new_frame(
                last_processed_sequence,
                timeout_sec=0.05,
            )
            if frame_data is None:
                if self._camera_stream_is_stale(now):
                    stable_started_at = None
                    samples.clear()
                    self._log_frame_error(
                        'no synchronized RGB/depth frame'
                    )
                if now - last_feedback_at >= 0.5:
                    self._publish_feedback(goal_handle, False, 0.0, 0.0)
                    last_feedback_at = now
                continue

            sequence, rgb_msg, depth_msg = frame_data
            last_processed_sequence = sequence
            next_inference_at = time.monotonic() + inference_interval

            frame_error = self._validate_frame_age(rgb_msg)
            if frame_error:
                stable_started_at = None
                samples.clear()
                self._publish_feedback(goal_handle, False, 0.0, 0.0)
                self._log_frame_error(frame_error)
                continue

            observation = self._process_frame(
                rgb_msg,
                depth_msg,
                target_class,
            )
            observation_time = time.monotonic()

            if observation.detection is None:
                stable_started_at = None
                samples.clear()
                self._publish_feedback(goal_handle, False, 0.0, 0.0)
                self.publish_debug_image(observation, 0.0)
                self._log_observation(observation, 0.0, len(samples))
                continue

            if stable_started_at is None:
                stable_started_at = observation_time
                samples.clear()
            stable_time = observation_time - stable_started_at

            if observation.base_point is not None:
                samples.append(
                    (
                        observation.base_point[0],
                        observation.base_point[1],
                        observation.base_point[2],
                        observation.detection.confidence,
                    )
                )

            self._publish_feedback(
                goal_handle,
                True,
                stable_time,
                observation.detection.confidence,
            )
            self.publish_debug_image(observation, stable_time)
            self._log_observation(observation, stable_time, len(samples))

            if (
                stable_time >= self.stable_detection_duration
                and len(samples) >= self.min_valid_samples
            ):
                final_values = median_localization(samples)
                self.publish_debug_image(
                    observation,
                    stable_time,
                    final_point=final_values[:3],
                )
                self._log_final_point(final_values, len(samples))
                goal_handle.succeed()
                return self._success_result(final_values)

        goal_handle.abort()
        return self._failure_result('ROS shutdown during recognition.')

    def _wait_for_new_frame(
        self,
        last_sequence: int,
        timeout_sec: float,
    ) -> Optional[tuple[int, Image, Image]]:
        with self._frame_condition:
            if self._latest_frame_sequence == last_sequence:
                self._frame_condition.wait(timeout=timeout_sec)
            if (
                self._latest_pair is None
                or self._latest_frame_sequence == last_sequence
            ):
                return None
            rgb_msg, depth_msg = self._latest_pair
            return self._latest_frame_sequence, rgb_msg, depth_msg

    def _camera_stream_is_stale(self, now: float) -> bool:
        with self._frame_condition:
            received_at = self._latest_frame_received_at
        return received_at == 0.0 or now - received_at > self.max_frame_age

    def _validate_frame_age(self, rgb_msg: Image) -> str:
        frame_time = Time.from_msg(rgb_msg.header.stamp)
        if frame_time.nanoseconds == 0:
            return 'RGB frame has no timestamp'
        age = (self.get_clock().now() - frame_time).nanoseconds / 1e9
        if age > self.max_frame_age:
            return f'synchronized frame is stale: age={age:.3f}s'
        return ''

    def _process_frame(
        self,
        rgb_msg: Image,
        depth_msg: Image,
        target_class: str,
    ) -> FrameObservation:
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8',
            )
        except CvBridgeError as exc:
            raise RuntimeError(f'RGB conversion failed: {exc}') from exc

        observation = FrameObservation(
            rgb_image=np.asarray(rgb_image).copy(),
            rgb_message=rgb_msg,
        )
        detection = self.find_target_detection(rgb_image, target_class)
        observation.detection = detection
        if detection is None:
            observation.error = 'target not detected'
            return observation

        u, v = detection.center
        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough',
            )
            observation.depth_m = get_median_depth(
                np.asarray(depth_image),
                u,
                v,
                depth_msg.encoding,
                self.depth_scale,
                self.depth_window_size,
            )
        except (CvBridgeError, ValueError) as exc:
            observation.error = f'invalid depth: {exc}'
            return observation

        if observation.depth_m is None:
            observation.error = 'invalid depth: no valid value in window'
            return observation

        with self._intrinsics_lock:
            intrinsics = self._camera_intrinsics
        if intrinsics is None:
            observation.error = 'CameraInfo unavailable'
            return observation

        observation.camera_point = deproject_pixel(
            u,
            v,
            observation.depth_m,
            intrinsics.fx,
            intrinsics.fy,
            intrinsics.cx,
            intrinsics.cy,
        )

        actual_frame = rgb_msg.header.frame_id
        if actual_frame and actual_frame != self.camera_frame:
            if not self._warned_frame_mismatch:
                self.get_logger().warn(
                    'RGB frame_id differs from camera_frame parameter: '
                    f'{actual_frame!r} != {self.camera_frame!r}'
                )
                self._warned_frame_mismatch = True

        try:
            observation.base_point = self.transform_point_to_base(
                observation.camera_point,
                rgb_msg,
            )
        except TransformException as exc:
            observation.error = f'TF unavailable: {exc}'
        return observation

    def find_target_detection(
        self,
        rgb_image: np.ndarray,
        target_class: str,
    ) -> Optional[Detection]:
        """Run YOLO once and select the strongest requested-class box."""
        prediction = self.model.predict(
            source=rgb_image,
            conf=self.confidence_threshold,
            verbose=False,
        )[0]
        detections = []
        if prediction.boxes is not None:
            for box in prediction.boxes:
                class_id = int(box.cls[0].item())
                class_name = self.class_names_by_id.get(
                    class_id,
                    str(class_id),
                )
                coordinates = box.xyxy[0].detach().cpu().tolist()
                detections.append(
                    Detection(
                        class_name=class_name,
                        confidence=float(box.conf[0].item()),
                        x1=float(coordinates[0]),
                        y1=float(coordinates[1]),
                        x2=float(coordinates[2]),
                        y2=float(coordinates[3]),
                    )
                )
        return select_target_detection(
            detections,
            target_class,
            self.confidence_threshold,
        )

    def transform_point_to_base(
        self,
        camera_point: tuple[float, float, float],
        rgb_msg: Image,
    ) -> tuple[float, float, float]:
        """Transform an optical point at image time into target_frame."""
        point = PointStamped()
        point.header.stamp = rgb_msg.header.stamp
        point.header.frame_id = self.camera_frame
        point.point.x = camera_point[0]
        point.point.y = camera_point[1]
        point.point.z = camera_point[2]

        transform = self.tf_buffer.lookup_transform(
            self.target_frame,
            self.camera_frame,
            Time.from_msg(rgb_msg.header.stamp),
            timeout=Duration(seconds=self.tf_timeout),
        )
        transformed = do_transform_point(point, transform)
        return (
            float(transformed.point.x),
            float(transformed.point.y),
            float(transformed.point.z),
        )

    def _publish_feedback(
        self,
        goal_handle,
        detected: bool,
        stable_time: float,
        confidence: float,
    ) -> None:
        feedback = FindObject.Feedback()
        feedback.detected = detected
        feedback.stable_time = float(stable_time)
        feedback.confidence = float(confidence)
        goal_handle.publish_feedback(feedback)

    def publish_debug_image(
        self,
        observation: FrameObservation,
        stable_time: float,
        final_point: Optional[tuple[float, float, float]] = None,
    ) -> None:
        """Publish the active recognition overlay as a ROS image."""
        image = observation.rgb_image.copy()
        detection = observation.detection

        if detection is not None:
            x1 = int(round(detection.x1))
            y1 = int(round(detection.y1))
            x2 = int(round(detection.x2))
            y2 = int(round(detection.y2))
            u, v = detection.center
            cv2.rectangle(image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(image, (u, v), 4, (0, 0, 255), -1)

            labels = [
                f'{detection.class_name} {detection.confidence:.2f}',
                self._depth_label(observation.depth_m),
                self._base_point_label(observation.base_point),
                (
                    f'stable: {stable_time:.1f} / '
                    f'{self.stable_detection_duration:.1f} s'
                ),
            ]
            self._draw_text_lines(image, labels, x1, max(20, y1 - 70))
        else:
            self._draw_text_lines(
                image,
                [f'searching: {self.target_class}', observation.error],
                10,
                25,
            )

        if observation.error and detection is not None:
            self._draw_text_lines(
                image,
                [observation.error],
                10,
                image.shape[0] - 20,
                color=(0, 0, 255),
            )
        if final_point is not None:
            self._draw_text_lines(
                image,
                [
                    'FINAL base_link median XYZ:',
                    (
                        f'{final_point[0]:.3f}, {final_point[1]:.3f}, '
                        f'{final_point[2]:.3f} m'
                    ),
                ],
                10,
                25,
                color=(0, 255, 0),
            )

        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                image,
                encoding='bgr8',
            )
        except CvBridgeError as exc:
            self.get_logger().error(f'Debug image conversion failed: {exc}')
            return
        debug_message.header = observation.rgb_message.header
        self.debug_image_publisher.publish(debug_message)

    @staticmethod
    def _draw_text_lines(
        image: np.ndarray,
        lines: list[str],
        x: int,
        y: int,
        color: tuple[int, int, int] = (0, 255, 0),
    ) -> None:
        for index, line in enumerate(line for line in lines if line):
            position = (max(0, x), max(18, y + index * 20))
            cv2.putText(
                image,
                line,
                position,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                2,
                cv2.LINE_AA,
            )

    @staticmethod
    def _depth_label(depth_m: Optional[float]) -> str:
        if depth_m is None:
            return 'depth: invalid'
        return f'depth: {depth_m:.3f} m'

    @staticmethod
    def _base_point_label(
        point: Optional[tuple[float, float, float]],
    ) -> str:
        if point is None:
            return 'base XYZ: unavailable'
        return f'base XYZ: {point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f} m'

    def _log_frame_error(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_detail_log_time >= 1.0:
            self.get_logger().warn(message)
            self._last_detail_log_time = now

    def _log_observation(
        self,
        observation: FrameObservation,
        stable_time: float,
        sample_count: int,
    ) -> None:
        now = time.monotonic()
        if now - self._last_detail_log_time < 1.0:
            return
        detection = observation.detection
        if detection is None:
            self.get_logger().info(
                f'target not detected; stable=0.0/'
                f'{self.stable_detection_duration:.1f}s'
            )
        else:
            u, v = detection.center
            self.get_logger().info(
                f'Detection class={detection.class_name} '
                f'confidence={detection.confidence:.3f}; '
                f'center=({u}, {v}); '
                f'depth={self._format_depth(observation.depth_m)}; '
                f'camera={self._format_point(observation.camera_point)}; '
                f'base={self._format_point(observation.base_point)}; '
                f'stable={stable_time:.2f}/'
                f'{self.stable_detection_duration:.2f}s; '
                f'samples={sample_count}; error={observation.error or "none"}'
            )
        self._last_detail_log_time = now

    def _log_final_point(
        self,
        values: tuple[float, float, float, float],
        sample_count: int,
    ) -> None:
        self.get_logger().info(
            'Final median base point: '
            f'X={values[0]:.3f}, Y={values[1]:.3f}, '
            f'Z={values[2]:.3f}, confidence={values[3]:.3f}, '
            f'samples={sample_count}'
        )

    @staticmethod
    def _format_depth(depth_m: Optional[float]) -> str:
        if depth_m is None:
            return 'invalid'
        return f'{depth_m:.3f}m'

    @staticmethod
    def _format_point(
        point: Optional[tuple[float, float, float]],
    ) -> str:
        if point is None:
            return 'unavailable'
        return f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})'

    def _success_result(
        self,
        values: tuple[float, float, float, float],
    ) -> FindObject.Result:
        result = FindObject.Result()
        result.success = True
        result.x = values[0]
        result.y = values[1]
        result.z = values[2]
        result.confidence = values[3]
        result.frame_id = self.target_frame
        result.message = 'Target successfully localized.'
        return result

    @staticmethod
    def _failure_result(message: str) -> FindObject.Result:
        result = FindObject.Result()
        result.success = False
        result.message = message
        return result

    def destroy_node(self) -> None:
        if hasattr(self, 'action_server'):
            self.action_server.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
