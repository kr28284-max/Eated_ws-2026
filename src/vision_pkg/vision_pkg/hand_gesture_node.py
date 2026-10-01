"""Publish open-palm observations from the D435i RGB stream."""

import time

import mediapipe as mp
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from vision_pkg.hand_gesture import is_open_palm


class HandGestureNode(Node):
    """Run MediaPipe only while the controller is carrying an object."""

    def __init__(self) -> None:
        super().__init__('hand_gesture_node')
        self.declare_parameter(
            'rgb_topic', '/realsense/d435i/color/image_raw'
        )
        self.declare_parameter('max_detection_rate_hz', 10.0)
        self.rgb_topic = str(self.get_parameter('rgb_topic').value)
        rate = float(self.get_parameter('max_detection_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('max_detection_rate_hz must be positive')
        self.inference_interval = 1.0 / rate
        self.last_inference_at = 0.0
        self.last_warning_at = 0.0
        self.active = False
        self.last_open = False
        self.bridge = CvBridge()
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=0,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7,
        )
        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_subscription = self.create_subscription(
            String,
            '/control/state',
            self.state_callback,
            state_qos,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.rgb_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.palm_publisher = self.create_publisher(
            Bool, '/vision/open_palm', 10
        )
        self.get_logger().info(
            f'MediaPipe hand detection ready: RGB={self.rgb_topic}'
        )

    def state_callback(self, msg: String) -> None:
        """Enable hand inference only during the delivery drive."""
        active = msg.data == 'DRIVE_TO_PLACE'
        if active != self.active:
            self.active = active
            self.last_open = False
            self.palm_publisher.publish(Bool(data=False))
            self.get_logger().info(
                'Open-palm detection enabled' if active
                else 'Open-palm detection disabled'
            )

    def image_callback(self, msg: Image) -> None:
        """Classify a color frame and publish a positive or negative."""
        if not self.active:
            return
        now = time.monotonic()
        if now - self.last_inference_at < self.inference_interval:
            return
        self.last_inference_at = now
        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            result = self.hands.process(rgb)
            is_open = bool(
                result.multi_hand_landmarks
                and any(
                    is_open_palm(hand.landmark)
                    for hand in result.multi_hand_landmarks
                )
            )
        except (CvBridgeError, RuntimeError, ValueError) as exc:
            is_open = False
            if now - self.last_warning_at >= 2.0:
                self.get_logger().warn(f'Hand detection failed: {exc}')
                self.last_warning_at = now
        self.palm_publisher.publish(Bool(data=is_open))
        if is_open != self.last_open:
            self.get_logger().info(
                'Open palm detected' if is_open else 'Open palm lost'
            )
            self.last_open = is_open

    def destroy_node(self) -> bool:
        """Release the MediaPipe graph before shutting down ROS."""
        self.hands.close()
        return super().destroy_node()


def main(args=None) -> None:
    """Run the hand gesture detector."""
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
