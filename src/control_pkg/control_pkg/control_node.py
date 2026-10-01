"""Coordinate voice, joystick, vision, base, arm, and gripper control."""

import math
import threading
import time
from typing import Callable, Optional, Sequence, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from eated_interfaces.action import FindObject
from eated_interfaces.msg import RobotCommand, RobotCommandStatus
from geometry_msgs.msg import Pose, Twist
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, Joy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from control_pkg.command_protocol import DEFAULT_SUPPORTED_TARGETS
from control_pkg.command_protocol import STOP_ACTION
from control_pkg.command_protocol import validate_voice_command
from control_pkg.open_palm_gate import OpenPalmGate
from control_pkg.task_state import TaskState, manual_drive_allowed


class OperationCancelled(RuntimeError):
    """Raised inside a worker after an emergency stop invalidates it."""


class ControlNode(Node):
    """Run the top-level fetch state machine and enforce control ownership."""

    JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']

    def __init__(self) -> None:
        super().__init__('control_node')
        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._goal_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._operation_id = 0
        self._active_goals = []
        self._worker: Optional[threading.Thread] = None
        self._previous_buttons = []
        self._current_joint_positions: Optional[list[float]] = None
        self._emergency_active = False

        self._declare_parameters()
        self._read_parameters()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self._palm_gate = OpenPalmGate(
            self.open_palm_hold_sec,
            self.open_palm_max_gap_sec,
        )

        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.command_subscription = self.create_subscription(
            RobotCommand,
            '/voice/robot_command',
            self.command_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10,
            callback_group=self.callback_group,
        )
        self.palm_subscription = self.create_subscription(
            Bool,
            '/vision/open_palm',
            self.palm_callback,
            10,
            callback_group=self.callback_group,
        )
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.status_publisher = self.create_publisher(
            RobotCommandStatus,
            '/control/command_status',
            10,
        )
        self.release_countdown_publisher = self.create_publisher(
            Float32,
            '/control/release_countdown_sec',
            10,
        )
        self.target_publisher = self.create_publisher(
            String,
            '/control/target_class',
            transient_qos,
        )
        self.state_publisher = self.create_publisher(
            String,
            '/control/state',
            transient_qos,
        )
        self.manual_drive_publisher = self.create_publisher(
            Bool,
            '/control/manual_drive_enabled',
            transient_qos,
        )
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_hold_publisher = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10,
        )

        self.vision_client = ActionClient(
            self,
            FindObject,
            '/find_object',
            callback_group=self.callback_group,
        )
        self.arm_joint_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group,
        )
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group,
        )
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group,
        )
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path',
            callback_group=self.callback_group,
        )
        self.execute_trajectory_client = ActionClient(
            self,
            ExecuteTrajectory,
            '/execute_trajectory',
            callback_group=self.callback_group,
        )

        self.state = TaskState.RETURN_HOME
        self.current_target_class = ''
        self.last_requested_target_class = ''
        self.last_object_point: Optional[Tuple[float, float, float]] = None
        self._manual_drive_enabled = False
        self._publish_state()
        self._publish_target()
        self._set_manual_drive(False, force=True)
        self.safety_timer = self.create_timer(
            0.05,
            self._base_safety_loop,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Control state machine ready: '
            f'targets={list(self.supported_targets)}, '
            f'buttons=A{self.recognition_button_index}/'
            f'X{self.home_button_index}/'
            f'Y{self.emergency_button_index}'
        )
        if not self.pick_enabled:
            self.get_logger().warn(
                'pick_enabled=false: Vision is active, but Cartesian pick '
                'motion is blocked until measured offsets are configured.'
            )

        if self.home_on_startup:
            self._start_operation(self._startup_home_sequence)
        else:
            self._set_state(TaskState.IDLE)
            self._set_manual_drive(True, force=True)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            'supported_targets',
            list(DEFAULT_SUPPORTED_TARGETS),
        )
        self.declare_parameter('recognition_button_index', 0)
        self.declare_parameter('home_button_index', 2)
        self.declare_parameter('emergency_button_index', 3)
        self.declare_parameter('home_on_startup', True)
        self.declare_parameter('home_pose_deg', [0.0, -101.0, 53.0, 70.0])
        self.declare_parameter(
            'recognition_pose_deg',
            [0.0, -30.0, -41.0, 115.0],
        )
        self.declare_parameter(
            'cargo_pose_deg',
            [-73.0, -30.0, 38.0, -10.0],
        )
        self.declare_parameter('joint_motion_duration_sec', 4.0)
        self.declare_parameter('recognition_settle_sec', 0.5)
        self.declare_parameter('release_settle_sec', 0.5)
        self.declare_parameter('open_palm_hold_sec', 2.0)
        self.declare_parameter('open_palm_max_gap_sec', 0.4)
        self.declare_parameter('action_server_timeout_sec', 10.0)
        self.declare_parameter('action_result_timeout_sec', 20.0)
        self.declare_parameter('vision_result_timeout_sec', 60.0)
        self.declare_parameter('gripper_open_position', 0.019)
        self.declare_parameter('gripper_closed_position', -0.010)
        self.declare_parameter('gripper_max_effort', 0.2)
        self.declare_parameter('grasp_position_threshold', -0.008)
        self.declare_parameter('pick_enabled', True)
        self.declare_parameter('pick_calibration_xyz_m', [0.02, 0.0, 0.0])
        self.declare_parameter('pregrasp_offset_xyz_m', [0.0, 0.0, 0.05])
        self.declare_parameter('retreat_offset_xyz_m', [0.0, 0.0, 0.15])
        self.declare_parameter('move_group_name', 'arm')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_link', 'end_effector_link')
        self.declare_parameter(
            'pick_orientation_xyzw',
            [0.0, 0.7071068, 0.0, 0.7071068],
        )
        self.declare_parameter('use_orientation_constraint', False)
        self.declare_parameter('position_tolerance_m', 0.01)
        self.declare_parameter('orientation_tolerance_deg', 50.0)
        self.declare_parameter('moveit_planning_time_sec', 3.0)
        self.declare_parameter('moveit_velocity_scale', 0.10)
        self.declare_parameter('moveit_acceleration_scale', 0.10)
        self.declare_parameter('retreat_cartesian_step_m', 0.005)
        self.declare_parameter('retreat_joint_jump_rad', 0.35)

    def _read_parameters(self) -> None:
        def value(name: str):
            return self.get_parameter(name).value

        self.supported_targets = tuple(value('supported_targets'))
        self.recognition_button_index = int(value('recognition_button_index'))
        self.home_button_index = int(value('home_button_index'))
        self.emergency_button_index = int(value('emergency_button_index'))
        button_indices = {
            self.recognition_button_index,
            self.home_button_index,
            self.emergency_button_index,
        }
        if min(button_indices) < 0 or len(button_indices) != 3:
            raise ValueError('A, X, and Y button indices must be unique')
        self.home_on_startup = bool(value('home_on_startup'))
        self.home_pose = self._pose_parameter(value('home_pose_deg'))
        self.recognition_pose = self._pose_parameter(
            value('recognition_pose_deg')
        )
        self.cargo_pose = self._pose_parameter(value('cargo_pose_deg'))
        self.joint_motion_duration = float(value('joint_motion_duration_sec'))
        self.recognition_settle = float(value('recognition_settle_sec'))
        self.release_settle = float(value('release_settle_sec'))
        self.open_palm_hold_sec = float(value('open_palm_hold_sec'))
        self.open_palm_max_gap_sec = float(value('open_palm_max_gap_sec'))
        self.server_timeout = float(value('action_server_timeout_sec'))
        self.result_timeout = float(value('action_result_timeout_sec'))
        self.vision_result_timeout = float(value('vision_result_timeout_sec'))
        self.gripper_open = float(value('gripper_open_position'))
        self.gripper_closed = float(value('gripper_closed_position'))
        self.gripper_effort = float(value('gripper_max_effort'))
        self.grasp_position_threshold = float(
            value('grasp_position_threshold')
        )
        if not (
            self.gripper_closed
            < self.grasp_position_threshold
            < self.gripper_open
        ):
            raise ValueError(
                'grasp_position_threshold must be between the closed and '
                'open gripper positions'
            )
        self.pick_enabled = bool(value('pick_enabled'))
        self.pick_calibration = self._xyz_parameter(
            value('pick_calibration_xyz_m')
        )
        self.pregrasp_offset = self._xyz_parameter(
            value('pregrasp_offset_xyz_m')
        )
        self.retreat_offset = self._xyz_parameter(
            value('retreat_offset_xyz_m')
        )
        if self.pick_enabled:
            if self._vector_norm(self.pregrasp_offset) < 1e-6:
                raise ValueError(
                    'pick_enabled requires a measured pregrasp offset'
                )
            if self._vector_norm(self.retreat_offset) < 1e-6:
                raise ValueError(
                    'pick_enabled requires a measured retreat offset'
                )
            if (
                abs(self.retreat_offset[0]) > 1e-6
                or abs(self.retreat_offset[1]) > 1e-6
            ):
                raise ValueError(
                    'retreat_offset_xyz_m must have zero X and Y '
                    'for a vertical retreat'
                )
            if self.retreat_offset[2] <= 0:
                raise ValueError('retreat_offset_xyz_m Z must be positive')
        self.move_group_name = str(value('move_group_name'))
        self.base_frame = str(value('base_frame'))
        self.end_effector_link = str(value('end_effector_link'))
        self.pick_orientation = self._quaternion_parameter(
            value('pick_orientation_xyzw')
        )
        self.use_orientation_constraint = bool(
            value('use_orientation_constraint')
        )
        self.position_tolerance = float(value('position_tolerance_m'))
        self.orientation_tolerance = math.radians(
            float(value('orientation_tolerance_deg'))
        )
        self.moveit_planning_time = float(value('moveit_planning_time_sec'))
        self.moveit_velocity_scale = float(value('moveit_velocity_scale'))
        self.moveit_acceleration_scale = float(
            value('moveit_acceleration_scale')
        )
        self.retreat_cartesian_step = float(value('retreat_cartesian_step_m'))
        self.retreat_joint_jump = float(value('retreat_joint_jump_rad'))
        if self.retreat_cartesian_step <= 0 or self.retreat_joint_jump <= 0:
            raise ValueError(
                'Cartesian retreat step and joint jump must be positive'
            )

    @staticmethod
    def _pose_parameter(values: Sequence[float]) -> list[float]:
        if len(values) != 4:
            raise ValueError('joint pose parameters must contain four values')
        return [math.radians(float(value)) for value in values]

    @staticmethod
    def _xyz_parameter(values: Sequence[float]) -> Tuple[float, float, float]:
        if len(values) != 3:
            raise ValueError('XYZ parameters must contain three values')
        return tuple(float(value) for value in values)

    @staticmethod
    def _quaternion_parameter(
        values: Sequence[float],
    ) -> Tuple[float, float, float, float]:
        if len(values) != 4:
            raise ValueError('quaternion parameter must contain four values')
        quaternion = tuple(float(value) for value in values)
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1e-6:
            raise ValueError('pick orientation quaternion cannot be zero')
        return tuple(value / norm for value in quaternion)

    @staticmethod
    def _vector_norm(values: Sequence[float]) -> float:
        return math.sqrt(sum(value * value for value in values))

    def command_callback(self, msg: RobotCommand) -> None:
        """Handle fetch and stop commands from the ROS voice publisher."""
        decision = validate_voice_command(
            msg.action,
            msg.target_class,
            self.supported_targets,
        )
        accepted = decision.accepted
        message = decision.message
        if accepted and decision.action == STOP_ACTION:
            self._handle_emergency_stop(source='voice')
            self._publish_command_status(
                True, decision.action, decision.target_class, message
            )
            return
        with self._lock:
            if accepted and self.state != TaskState.IDLE:
                accepted = False
                message = f'controller is busy in state {self.state.value}'

            if accepted:
                self.last_requested_target_class = decision.target_class
                self._begin_recognition(decision.target_class)
                message = 'recognition and Pick started'

        self._publish_command_status(
            accepted,
            decision.action,
            decision.target_class,
            message,
        )
        if accepted:
            self.get_logger().info(
                f'Fetch target accepted: {decision.target_class}; '
                'starting recognition and Pick now'
            )
        else:
            self.get_logger().warn(f'Voice command rejected: {message}')

    def joy_callback(self, msg: Joy) -> None:
        """Dispatch rising edges for A/X/Y according to the task state."""
        buttons = list(msg.buttons)
        if not self._previous_buttons:
            self._previous_buttons = [0] * len(buttons)

        emergency = self._rising_edge(buttons, self.emergency_button_index)
        home = self._rising_edge(buttons, self.home_button_index)
        recognition = self._rising_edge(
            buttons,
            self.recognition_button_index,
        )
        self._previous_buttons = buttons

        if emergency:
            self._handle_emergency_stop()
            return
        if home:
            self._handle_home_recovery()
            return
        if recognition:
            self._handle_recognition_request()

    def _rising_edge(self, buttons: Sequence[int], index: int) -> bool:
        current = 0 <= index < len(buttons) and buttons[index] == 1
        previous = (
            0 <= index < len(self._previous_buttons)
            and self._previous_buttons[index] == 1
        )
        return current and not previous

    def _begin_recognition(self, target_class: str) -> None:
        """Claim the base and start the recognition/Pick worker."""
        self.current_target_class = target_class
        self.last_object_point = None
        self._publish_target()
        self._set_manual_drive(False, force=True)
        self._publish_zero_velocity()
        self._set_state(TaskState.MOVE_TO_RECOGNITION)
        self._start_operation(self._recognition_pick_sequence)

    def _handle_recognition_request(self) -> None:
        with self._lock:
            target = self.last_requested_target_class
            valid = self.state == TaskState.IDLE and bool(target)
            if valid:
                self._begin_recognition(target)
        if valid:
            self.get_logger().info(
                f'A pressed: repeating recognition and Pick for {target}'
            )
        else:
            self.get_logger().warn(
                f'A ignored in state {self.state.value}: '
                'no idle target is available'
            )
            self._restore_state_authority()

    def palm_callback(self, msg: Bool) -> None:
        """Release after the configured hold of fresh open-palm signals."""
        with self._lock:
            if self.state != TaskState.DRIVE_TO_PLACE:
                self._palm_gate.reset()
                return
            now = time.monotonic()
            triggered = self._palm_gate.observe(bool(msg.data), now)
            remaining = self._palm_gate.remaining_seconds(now)
            self.release_countdown_publisher.publish(
                Float32(data=float(remaining))
            )
            if triggered:
                self._set_manual_drive(False, force=True)
                self._publish_zero_velocity()
                self._set_state(TaskState.PLACING)
                self._start_operation(self._release_sequence)
        if triggered:
            self.get_logger().info(
                f'Open palm held for {self.open_palm_hold_sec:.1f} seconds: '
                'releasing at CARGO'
            )

    def _handle_emergency_stop(self, source: str = 'Y') -> None:
        """Cancel active actions, hold the arm, and restore manual drive."""
        with self._lock:
            self._operation_id += 1
            self._stop_event.set()
            self._emergency_active = True
            self._set_manual_drive(False, force=True)
            self._set_state(TaskState.ERROR)
        self._cancel_active_goals()
        self._publish_zero_velocity()
        self._publish_arm_hold()
        self._set_manual_drive(True, force=True)
        self.get_logger().error(
            f'EMERGENCY STOP ({source}): actions cancelled and joystick base '
            'control restored. Press X to return the arm HOME.'
        )

    def _handle_home_recovery(self) -> None:
        with self._lock:
            valid = self.state == TaskState.ERROR
            if valid:
                self._stop_event.clear()
                self._set_manual_drive(False, force=True)
                self._publish_zero_velocity()
                self._set_state(TaskState.RETURN_HOME)
        if valid:
            self.get_logger().info(
                'X pressed in ERROR: returning arm HOME'
            )
            self._start_operation(self._home_recovery_sequence)
        else:
            self.get_logger().warn(
                'X ignored: HOME recovery is only available in ERROR'
            )
            self._restore_state_authority()

    def joint_state_callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in self.JOINT_NAMES):
            current = [float(positions[name]) for name in self.JOINT_NAMES]
            with self._lock:
                self._current_joint_positions = current

    def _startup_home_sequence(self, operation_id: int) -> None:
        try:
            self.get_logger().info('Startup: moving manipulator to HOME')
            self._send_joint_pose(self.home_pose, operation_id, 'HOME')
            self._ensure_active(operation_id)
            self._set_state(TaskState.IDLE, operation_id)
            self._set_manual_drive(True, force=True)
            self.get_logger().info('Startup complete: joystick drive enabled')
        except OperationCancelled:
            return
        except Exception as exc:
            self._fail_operation(
                operation_id,
                f'startup HOME failed: {exc}',
                auto_home=False,
            )

    def _recognition_pick_sequence(self, operation_id: int) -> None:
        try:
            self._send_joint_pose(
                self.recognition_pose,
                operation_id,
                'RECOGNITION',
            )
            self._sleep_cancelable(self.recognition_settle, operation_id)
            self._set_state(TaskState.VISION_SEARCH, operation_id)
            result = self._find_object(operation_id)
            point = (float(result.x), float(result.y), float(result.z))
            with self._lock:
                self.last_object_point = point
            self.get_logger().info(
                'Vision target received in base_link: '
                f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}) m'
            )

            if not self.pick_enabled:
                self.get_logger().warn(
                    'Pick motion skipped safely because pick_enabled=false. '
                    'Returning HOME; set measured calibration/approach/'
                    'retreat offsets before enabling Pick.'
                )
                self._set_state(TaskState.RETURN_HOME, operation_id)
                self._send_joint_pose(self.home_pose, operation_id, 'HOME')
                self._finish_task(operation_id)
                return

            self._set_state(TaskState.PICKING, operation_id)
            pick_point = self._add_xyz(point, self.pick_calibration)
            pregrasp = self._add_xyz(pick_point, self.pregrasp_offset)
            self._send_gripper(self.gripper_open, operation_id, 'OPEN')
            self._send_cartesian_pose(pregrasp, operation_id, 'PREGRASP')
            self.get_logger().info(
                f'Descending {self.pregrasp_offset[2] * 100.0:.1f} cm '
                'from PREGRASP to PICK'
            )
            self._send_cartesian_pose(pick_point, operation_id, 'PICK')
            self._send_gripper(
                self.gripper_closed,
                operation_id,
                'CLOSE',
                accept_stall=True,
            )
            self.get_logger().info(
                f'Planning up to {self.retreat_offset[2] * 100.0:.1f} cm '
                'of vertical lift from PICK'
            )
            self._send_vertical_retreat(operation_id)
            self._set_state(TaskState.MOVE_TO_CARGO, operation_id)
            self._send_joint_pose(self.cargo_pose, operation_id, 'CARGO')
            self._set_state(TaskState.DRIVE_TO_PLACE, operation_id)
            self._set_manual_drive(True, force=True)
            self.get_logger().info(
                'Object secured: drive to the delivery area and show an '
                f'open palm to the camera for {self.open_palm_hold_sec:.1f} '
                'seconds'
            )
        except OperationCancelled:
            return
        except Exception as exc:
            self._fail_operation(
                operation_id,
                f'recognition/pick failed: {exc}',
            )

    def _release_sequence(self, operation_id: int) -> None:
        try:
            self._send_gripper(self.gripper_open, operation_id, 'OPEN')
            self._sleep_cancelable(self.release_settle, operation_id)
            self._set_state(TaskState.RETURN_HOME, operation_id)
            self._send_joint_pose(self.home_pose, operation_id, 'HOME')
            self._finish_task(operation_id)
        except OperationCancelled:
            return
        except Exception as exc:
            self._fail_operation(operation_id, f'release failed: {exc}')

    def _home_recovery_sequence(self, operation_id: int) -> None:
        try:
            self._send_joint_pose(self.home_pose, operation_id, 'HOME')
            with self._lock:
                self._emergency_active = False
            self._finish_task(operation_id)
            self.get_logger().info(
                'HOME recovery complete: joystick drive enabled'
            )
        except OperationCancelled:
            return
        except Exception as exc:
            self._fail_operation(
                operation_id,
                f'HOME recovery failed: {exc}',
                auto_home=False,
            )

    def _finish_task(self, operation_id: int) -> None:
        self._ensure_active(operation_id)
        with self._lock:
            self.current_target_class = ''
            self.last_object_point = None
            self._publish_target()
        self._set_state(TaskState.IDLE, operation_id)
        self._set_manual_drive(True, force=True)

    def _find_object(self, operation_id: int):
        self._wait_for_server(self.vision_client, 'Vision', operation_id)
        goal = FindObject.Goal()
        goal.target_class = self.current_target_class
        goal_handle = self._send_goal(
            self.vision_client,
            goal,
            operation_id,
            'Vision',
        )
        response = self._wait_result(
            goal_handle,
            operation_id,
            'Vision',
            self.vision_result_timeout,
        )
        result = response.result
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or not result.success
        ):
            raise RuntimeError(result.message or 'Vision action failed')
        if result.frame_id != self.base_frame:
            raise RuntimeError(
                f'Vision frame is {result.frame_id!r}, expected '
                f'{self.base_frame!r}'
            )
        return result

    def _send_joint_pose(
        self,
        positions: Sequence[float],
        operation_id: int,
        label: str,
    ) -> None:
        self._wait_for_server(
            self.arm_joint_client,
            'arm trajectory',
            operation_id,
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        self._set_duration(point.time_from_start, self.joint_motion_duration)
        goal.trajectory.points.append(point)
        goal_handle = self._send_goal(
            self.arm_joint_client,
            goal,
            operation_id,
            f'arm {label}',
        )
        response = self._wait_result(
            goal_handle,
            operation_id,
            f'arm {label}',
            self.joint_motion_duration + 8.0,
        )
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or response.result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f'{label} trajectory failed: '
                f'{response.result.error_string or response.result.error_code}'
            )
        self.get_logger().info(f'Arm reached {label}')

    def _send_gripper(
        self,
        position: float,
        operation_id: int,
        label: str,
        accept_stall: bool = False,
    ) -> None:
        self._wait_for_server(
            self.gripper_client,
            'gripper',
            operation_id,
        )
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.gripper_effort
        goal_handle = self._send_goal(
            self.gripper_client,
            goal,
            operation_id,
            f'gripper {label}',
        )
        response = self._wait_result(
            goal_handle,
            operation_id,
            f'gripper {label}',
        )
        result = response.result
        if accept_stall:
            valid_grasp = (
                result.stalled
                and result.position >= self.grasp_position_threshold
            )
            if valid_grasp:
                self.get_logger().info(
                    f'Gripper {label} grasp confirmed: '
                    f'position={result.position:.4f} >= '
                    f'{self.grasp_position_threshold:.4f}, '
                    f'stalled={result.stalled}, effort={result.effort:.3f}'
                )
                return

            if result.reached_goal:
                raise RuntimeError(
                    f'gripper {label} reached the fully closed target; '
                    'no object was detected between the fingers'
                )

            raise RuntimeError(
                f'gripper {label} did not confirm a grasp: '
                f'status={response.status}, position={result.position:.4f}, '
                f'required>={self.grasp_position_threshold:.4f}, '
                f'stalled={result.stalled}, '
                f'reached_goal={result.reached_goal}'
            )

        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'Gripper {label} complete: position={result.position:.4f}, '
                f'reached_goal={result.reached_goal}'
            )
            return

        raise RuntimeError(
            f'gripper {label} action failed: status={response.status}, '
            f'position={result.position:.4f}, effort={result.effort:.3f}, '
            f'stalled={result.stalled}, reached_goal={result.reached_goal}'
        )

    def _send_cartesian_pose(
        self,
        point: Tuple[float, float, float],
        operation_id: int,
        label: str,
    ) -> None:
        self._wait_for_server(
            self.move_group_client,
            'MoveIt',
            operation_id,
        )
        goal = MoveGroup.Goal()
        goal.request.group_name = self.move_group_name
        goal.request.allowed_planning_time = self.moveit_planning_time
        goal.request.num_planning_attempts = 3
        goal.request.max_velocity_scaling_factor = self.moveit_velocity_scale
        goal.request.max_acceleration_scaling_factor = (
            self.moveit_acceleration_scale
        )
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = point
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = self.pick_orientation

        constraints = Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.base_frame
        position_constraint.link_name = self.end_effector_link
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [self.position_tolerance] * 3
        position_constraint.constraint_region.primitives.append(box)
        position_constraint.constraint_region.primitive_poses.append(pose)
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

        if self.use_orientation_constraint:
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header.frame_id = self.base_frame
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_y_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_z_axis_tolerance = math.pi
            orientation_constraint.weight = 1.0
            constraints.orientation_constraints.append(
                orientation_constraint
            )
        goal.request.goal_constraints.append(constraints)

        goal_handle = self._send_goal(
            self.move_group_client,
            goal,
            operation_id,
            f'MoveIt {label}',
        )
        response = self._wait_result(
            goal_handle,
            operation_id,
            f'MoveIt {label}',
        )
        error_code = response.result.error_code.val
        if (
            response.status != GoalStatus.STATUS_SUCCEEDED
            or error_code != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(f'MoveIt {label} failed: error={error_code}')
        self.get_logger().info(
            f'MoveIt {label} reached '
            f'({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})'
        )

    def _send_vertical_retreat(self, operation_id: int) -> None:
        """Execute the collision-checked vertical path MoveIt can compute."""
        self._ensure_active(operation_id)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.end_effector_link,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'RETREAT skipped: tool pose is unavailable: {exc}'
            )
            return

        with self._lock:
            joints = (
                list(self._current_joint_positions)
                if self._current_joint_positions is not None
                else None
            )
        if joints is None:
            self.get_logger().warn(
                'RETREAT skipped: current arm joint states are unavailable'
            )
            return

        waypoint = Pose()
        waypoint.position.x = transform.transform.translation.x
        waypoint.position.y = transform.transform.translation.y
        waypoint.position.z = (
            transform.transform.translation.z + self.retreat_offset[2]
        )
        waypoint.orientation = transform.transform.rotation

        deadline = time.monotonic() + self.server_timeout
        while not self.cartesian_path_client.wait_for_service(timeout_sec=0.1):
            self._ensure_active(operation_id)
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    'RETREAT skipped: Cartesian path service is unavailable'
                )
                return

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.start_state.is_diff = True
        request.start_state.joint_state.name = list(self.JOINT_NAMES)
        request.start_state.joint_state.position = joints
        request.group_name = self.move_group_name
        request.link_name = self.end_effector_link
        request.waypoints = [waypoint]
        request.max_step = self.retreat_cartesian_step
        request.revolute_jump_threshold = self.retreat_joint_jump
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.moveit_velocity_scale
        request.max_acceleration_scaling_factor = (
            self.moveit_acceleration_scale
        )

        try:
            response = self._wait_future(
                self.cartesian_path_client.call_async(request),
                operation_id,
                'MoveIt RETREAT Cartesian plan',
                self.result_timeout,
            )
        except OperationCancelled:
            raise
        except Exception as exc:
            self.get_logger().warn(f'RETREAT planning failed: {exc}')
            return
        if (
            response.error_code.val != MoveItErrorCodes.SUCCESS
            or not math.isfinite(response.fraction)
            or response.fraction <= 0.0
            or len(response.solution.joint_trajectory.points) < 2
        ):
            self.get_logger().warn(
                'RETREAT skipped: MoveIt returned no executable path: '
                f'fraction={response.fraction:.3f}, '
                f'error={response.error_code.val}'
            )
            return

        if response.fraction < 0.999:
            self.get_logger().warn(
                'RETREAT partially planned: '
                f'{response.fraction * 100.0:.1f}% of '
                f'{self.retreat_offset[2] * 100.0:.1f} cm; '
                'executing the reachable vertical segment'
            )

        self._ensure_active(operation_id)
        self._wait_for_server(
            self.execute_trajectory_client,
            'MoveIt trajectory execution',
            operation_id,
        )
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal_handle = self._send_goal(
            self.execute_trajectory_client,
            goal,
            operation_id,
            'MoveIt RETREAT execution',
        )
        execution = self._wait_result(
            goal_handle,
            operation_id,
            'MoveIt RETREAT execution',
        )
        if (
            execution.status != GoalStatus.STATUS_SUCCEEDED
            or execution.result.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            raise RuntimeError(
                'MoveIt RETREAT execution failed: '
                f'error={execution.result.error_code.val}'
            )
        self.get_logger().info(
            'RETREAT completed: executed '
            f'{response.fraction * 100.0:.1f}% of the requested '
            f'{self.retreat_offset[2] * 100.0:.1f} cm lift '
            f'along {self.base_frame} Z'
        )

    def _wait_for_server(
        self,
        client: ActionClient,
        label: str,
        operation_id: int,
    ) -> None:
        self._ensure_active(operation_id)
        if not client.wait_for_server(timeout_sec=self.server_timeout):
            raise RuntimeError(f'{label} action server is unavailable')
        self._ensure_active(operation_id)

    def _send_goal(
        self,
        client: ActionClient,
        goal,
        operation_id: int,
        label: str,
    ):
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda completed: self._cancel_goal_if_stale(
                completed,
                operation_id,
            )
        )
        goal_handle = self._wait_future(
            future,
            operation_id,
            f'{label} goal response',
            self.server_timeout,
        )
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f'{label} goal was rejected')
        with self._goal_lock:
            self._active_goals.append(goal_handle)
        self._ensure_active(operation_id)
        return goal_handle

    def _cancel_goal_if_stale(self, future, operation_id: int) -> None:
        """Cancel a goal accepted after its operation was emergency-stopped."""
        try:
            goal_handle = future.result()
            with self._lock:
                stale = (
                    operation_id != self._operation_id
                    or self._stop_event.is_set()
                )
            if stale and goal_handle is not None and goal_handle.accepted:
                goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().warn(
                f'Could not cancel a stale action goal: {exc}'
            )

    def _wait_result(
        self,
        goal_handle,
        operation_id: int,
        label: str,
        timeout: Optional[float] = None,
    ):
        try:
            return self._wait_future(
                goal_handle.get_result_async(),
                operation_id,
                f'{label} result',
                self.result_timeout if timeout is None else timeout,
            )
        finally:
            with self._goal_lock:
                if goal_handle in self._active_goals:
                    self._active_goals.remove(goal_handle)

    def _wait_future(
        self,
        future,
        operation_id: int,
        label: str,
        timeout: float,
    ):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done():
            self._ensure_active(operation_id)
            if time.monotonic() >= deadline:
                raise RuntimeError(f'{label} timed out after {timeout:.1f}s')
            time.sleep(0.02)
        self._ensure_active(operation_id)
        if not future.done():
            raise RuntimeError(f'{label} interrupted by ROS shutdown')
        exception = future.exception()
        if exception is not None:
            raise RuntimeError(f'{label} failed: {exception}')
        return future.result()

    def _sleep_cancelable(self, duration: float, operation_id: int) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._ensure_active(operation_id)
            time.sleep(min(0.02, deadline - time.monotonic()))

    def _ensure_active(self, operation_id: int) -> None:
        with self._lock:
            if (
                operation_id != self._operation_id
                or self._stop_event.is_set()
            ):
                raise OperationCancelled()

    def _start_operation(self, target: Callable[[int], None]) -> None:
        with self._lock:
            self._operation_id += 1
            operation_id = self._operation_id
            self._stop_event.clear()
        worker = threading.Thread(
            target=target,
            args=(operation_id,),
            daemon=True,
        )
        self._worker = worker
        worker.start()

    def _cancel_active_goals(self) -> None:
        with self._goal_lock:
            goals = list(self._active_goals)
            self._active_goals.clear()
        for goal_handle in goals:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f'Action cancellation failed: {exc}')

    def _publish_arm_hold(self) -> None:
        with self._lock:
            positions = (
                list(self._current_joint_positions)
                if self._current_joint_positions is not None
                else None
            )
        if positions is None:
            self.get_logger().warn(
                'No joint_states received; arm hold trajectory was not sent'
            )
            return
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        self._set_duration(point.time_from_start, 0.1)
        trajectory.points.append(point)
        self.arm_hold_publisher.publish(trajectory)

    @staticmethod
    def _set_duration(duration, seconds: float) -> None:
        whole = int(seconds)
        duration.sec = whole
        duration.nanosec = int((seconds - whole) * 1e9)

    @staticmethod
    def _add_xyz(
        first: Tuple[float, float, float],
        second: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        return tuple(a + b for a, b in zip(first, second))

    def _set_state(
        self,
        new_state: TaskState,
        operation_id: Optional[int] = None,
    ) -> None:
        if operation_id is not None:
            self._ensure_active(operation_id)
        with self._lock:
            old_state = self.state
            self.state = new_state
            if old_state != new_state:
                self._palm_gate.reset()
            self._publish_state()
        if old_state != new_state:
            self.get_logger().info(
                f'STATE {old_state.value} -> {new_state.value}'
            )

    def _restore_state_authority(self) -> None:
        with self._lock:
            enabled = manual_drive_allowed(self.state)
        self._set_manual_drive(enabled, force=True)

    def _set_manual_drive(self, enabled: bool, force: bool = False) -> None:
        with self._lock:
            changed = enabled != self._manual_drive_enabled
            self._manual_drive_enabled = enabled
        if changed or force:
            message = Bool()
            message.data = enabled
            self.manual_drive_publisher.publish(message)
        if not enabled:
            self._publish_zero_velocity()

    def _base_safety_loop(self) -> None:
        if not self._manual_drive_enabled:
            self._publish_zero_velocity()

    def _publish_zero_velocity(self) -> None:
        self.velocity_publisher.publish(Twist())

    def _publish_state(self) -> None:
        message = String()
        message.data = self.state.value
        self.state_publisher.publish(message)

    def _publish_target(self) -> None:
        message = String()
        message.data = self.current_target_class
        self.target_publisher.publish(message)

    def _publish_command_status(
        self,
        accepted: bool,
        action: str,
        target_class: str,
        message: str,
    ) -> None:
        status = RobotCommandStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.accepted = accepted
        status.action = action
        status.target_class = target_class
        status.message = message
        self.status_publisher.publish(status)

    def _fail_operation(
        self,
        operation_id: int,
        message: str,
        auto_home: bool = True,
    ) -> None:
        try:
            self._ensure_active(operation_id)
        except OperationCancelled:
            return
        self._cancel_active_goals()
        self._publish_zero_velocity()
        self._set_state(TaskState.ERROR, operation_id)
        self.get_logger().error(message)
        if not auto_home:
            self._set_manual_drive(True, force=True)
            return
        self._set_manual_drive(False, force=True)
        try:
            self._set_state(TaskState.RETURN_HOME, operation_id)
            self.get_logger().warn('Operation failed: returning arm HOME')
            self._send_joint_pose(self.home_pose, operation_id, 'HOME')
            self._finish_task(operation_id)
        except OperationCancelled:
            return
        except Exception as exc:
            try:
                self._set_state(TaskState.ERROR, operation_id)
            except OperationCancelled:
                return
            self._set_manual_drive(True, force=True)
            self.get_logger().error(
                f'Automatic HOME recovery failed: {exc}. '
                'Press X to retry HOME.'
            )

    def destroy_node(self) -> None:
        """Cancel active work and stop the mobile base before shutdown."""
        with self._lock:
            self._operation_id += 1
            self._stop_event.set()
        if rclpy.ok():
            self._cancel_active_goals()
            self._publish_zero_velocity()
        super().destroy_node()


def main(args=None) -> None:
    """Run the controller with callbacks available during action waits."""
    rclpy.init(args=args)
    node = ControlNode()
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
