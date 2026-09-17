"""Joystick control for TurtleBot base driving only."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class TotalControl(Node):
    """Drive the TurtleBot base and reserve task-transition buttons."""

    def __init__(self) -> None:
        super().__init__('total_control_node')

        self.declare_parameter('recognition_button_index', 0)
        self.declare_parameter('place_button_index', 1)
        self.declare_parameter('home_button_index', 2)
        self.declare_parameter('emergency_button_index', 3)
        self.recognition_button_index = int(
            self.get_parameter('recognition_button_index').value
        )
        self.place_button_index = int(
            self.get_parameter('place_button_index').value
        )
        self.home_recovery_button_index = int(
            self.get_parameter('home_button_index').value
        )
        self.emergency_stop_button_index = int(
            self.get_parameter('emergency_button_index').value
        )
        self._validate_button_parameters()

        self.wheel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
        )
        authority_qos = QoSProfile(depth=1)
        authority_qos.reliability = ReliabilityPolicy.RELIABLE
        authority_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.authority_subscription = self.create_subscription(
            Bool,
            '/control/manual_drive_enabled',
            self.authority_callback,
            authority_qos,
        )

        self.start_time = self.get_clock().now()
        self.last_joy_time = self.start_time
        self.latest_axes: list[float] = []
        self.latest_buttons: list[int] = []

        self.previous_recognition_pressed = False
        self.previous_place_pressed = False
        self.previous_home_recovery_pressed = False
        self.previous_emergency_stop_pressed = False
        self._button_count_warning_logged = False
        self.manual_drive_enabled = True
        self.neutral_required = True

        self.max_linear_speed = 0.20
        self.max_angular_speed = 0.60
        self.max_linear_accel = 0.60
        self.max_angular_accel = 1.80

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.command_linear = 0.0
        self.command_angular = 0.0

        self.control_period = 0.02
        self.joy_timeout_sec = 0.30
        self.control_timer = self.create_timer(
            self.control_period,
            self.control_loop,
        )

        self.get_logger().info(
            'TurtleBot base joystick control ready: '
            f'recognition_button_index={self.recognition_button_index} '
            '(reserved A), '
            f'place_button_index={self.place_button_index} (reserved B)'
        )
        self.get_logger().info(
            'Safety buttons: '
            f'X={self.home_recovery_button_index} (HOME recovery), '
            f'Y={self.emergency_stop_button_index} (emergency stop)'
        )
        self.get_logger().info(
            'Manipulator and gripper control are disabled in this node.'
        )

    def _validate_button_parameters(self) -> None:
        button_indices = {
            self.recognition_button_index,
            self.place_button_index,
            self.home_recovery_button_index,
            self.emergency_stop_button_index,
        }
        if min(button_indices) < 0:
            raise ValueError('button indices must not be negative')
        if len(button_indices) != 4:
            raise ValueError(
                'recognition, place, HOME recovery, and emergency-stop '
                'button indices must be different'
            )

    def authority_callback(self, msg: Bool) -> None:
        """Apply the Control Node's ownership decision to base driving."""
        enabled = bool(msg.data)
        if enabled == self.manual_drive_enabled:
            return

        self.manual_drive_enabled = enabled
        self.stop_base()
        self.neutral_required = True
        owner = 'joystick' if enabled else 'control node'
        self.get_logger().info(f'Base control authority: {owner}')

    def joy_callback(self, msg: Joy) -> None:
        """Store drive input and detect reserved button rising edges."""
        now = self.get_clock().now()
        self.last_joy_time = now
        self.latest_axes = list(msg.axes)
        self.latest_buttons = list(msg.buttons)

        recognition_pressed = self.get_button(
            self.recognition_button_index
        )
        place_pressed = self.get_button(self.place_button_index)
        home_recovery_pressed = self.get_button(
            self.home_recovery_button_index
        )
        emergency_stop_pressed = self.get_button(
            self.emergency_stop_button_index
        )
        self._warn_if_button_indices_unavailable()

        startup_elapsed = (now - self.start_time).nanoseconds / 1e9
        if startup_elapsed >= 1.0:
            if (
                recognition_pressed
                and not self.previous_recognition_pressed
            ):
                self._gate_for_task_button('Recognition/A')
                self.get_logger().info(
                    'Recognition button pressed: '
                    f'index={self.recognition_button_index}'
                )
            if place_pressed and not self.previous_place_pressed:
                self._gate_for_task_button('Place/B')
                self.get_logger().info(
                    f'Place button pressed: index={self.place_button_index}'
                )
            if (
                emergency_stop_pressed
                and not self.previous_emergency_stop_pressed
            ):
                self.stop_base()
                self.manual_drive_enabled = True
                self.neutral_required = True
                self.get_logger().warn(
                    'Emergency stop/Y pressed: base stopped; release the '
                    'sticks before manual driving resumes'
                )
            if (
                home_recovery_pressed
                and not self.previous_home_recovery_pressed
            ):
                self._gate_for_task_button('HOME recovery/X')

            linear_input = self.apply_deadzone(self.get_axis(1))
            angular_input = self.apply_deadzone(self.get_axis(0))
            if self.neutral_required:
                if linear_input == 0.0 and angular_input == 0.0:
                    self.neutral_required = False
                    self.get_logger().info(
                        'Joystick centered; manual velocity input is armed'
                    )
                else:
                    linear_input = 0.0
                    angular_input = 0.0

            if self.manual_drive_enabled and not self.neutral_required:
                self.target_linear = (
                    linear_input ** 3
                ) * self.max_linear_speed
                self.target_angular = (
                    angular_input ** 3
                ) * self.max_angular_speed
            else:
                self.target_linear = 0.0
                self.target_angular = 0.0

        self.previous_recognition_pressed = recognition_pressed
        self.previous_place_pressed = place_pressed
        self.previous_home_recovery_pressed = home_recovery_pressed
        self.previous_emergency_stop_pressed = emergency_stop_pressed

    def _gate_for_task_button(self, label: str) -> None:
        """Stop immediately while the Control Node evaluates a task button."""
        self.manual_drive_enabled = False
        self.neutral_required = True
        self.stop_base()
        self.get_logger().info(
            f'{label} pressed: waiting for Control Node authority'
        )

    def control_loop(self) -> None:
        """Publish acceleration-limited base velocity at a fixed rate."""
        now = self.get_clock().now()
        if (now - self.start_time).nanoseconds < 1_000_000_000:
            return

        if (
            now - self.last_joy_time
        ).nanoseconds > int(self.joy_timeout_sec * 1e9):
            self.target_linear = 0.0
            self.target_angular = 0.0

        if not self.manual_drive_enabled or self.neutral_required:
            self.stop_base()
            return

        max_linear_delta = self.max_linear_accel * self.control_period
        max_angular_delta = self.max_angular_accel * self.control_period
        self.command_linear = self.slew(
            self.command_linear,
            self.target_linear,
            max_linear_delta,
        )
        self.command_angular = self.slew(
            self.command_angular,
            self.target_angular,
            max_angular_delta,
        )
        self.publish_base_command(
            self.command_linear,
            self.command_angular,
        )

    def _warn_if_button_indices_unavailable(self) -> None:
        required_count = max(
            self.recognition_button_index,
            self.place_button_index,
            self.home_recovery_button_index,
            self.emergency_stop_button_index,
        ) + 1
        if len(self.latest_buttons) >= required_count:
            self._button_count_warning_logged = False
            return
        if not self._button_count_warning_logged:
            self.get_logger().warn(
                'Joy message does not contain the configured task buttons: '
                f'button_count={len(self.latest_buttons)}, '
                f'required_count={required_count}'
            )
            self._button_count_warning_logged = True

    def get_axis(self, index: int, default: float = 0.0) -> float:
        """Return one joystick axis or a safe default."""
        if 0 <= index < len(self.latest_axes):
            return float(self.latest_axes[index])
        return default

    def get_button(self, index: int) -> bool:
        """Return whether one joystick button is currently pressed."""
        return (
            0 <= index < len(self.latest_buttons)
            and self.latest_buttons[index] == 1
        )

    @staticmethod
    def apply_deadzone(value: float, deadzone: float = 0.08) -> float:
        """Suppress small centered-stick noise."""
        if abs(value) < deadzone:
            return 0.0
        return value

    @staticmethod
    def slew(current: float, target: float, max_delta: float) -> float:
        """Move a command toward a target without exceeding max_delta."""
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target

    def publish_base_command(self, linear: float, angular: float) -> None:
        """Publish a TurtleBot velocity command."""
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.wheel_publisher.publish(command)

    def stop_base(self) -> None:
        """Publish an immediate zero-velocity safety command."""
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.publish_base_command(0.0, 0.0)

    def destroy_node(self) -> None:
        """Stop the base before destroying the ROS node."""
        if rclpy.ok():
            self.stop_base()
        super().destroy_node()


def main(args=None) -> None:
    """Run the TurtleBot base joystick node."""
    rclpy.init(args=args)
    node = TotalControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
