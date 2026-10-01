"""Check that the grasp retreat uses reachable vertical segments."""

import threading
from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetCartesianPath
from trajectory_msgs.msg import JointTrajectoryPoint

from control_pkg.control_node import ControlNode


class CartesianClient:
    """Capture a planning request without running a robot."""

    def __init__(self):
        self.request = None

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        self.request = request
        return request


def make_controller(fraction, error_code=MoveItErrorCodes.SUCCESS):
    """Create only the ROS edges used by the retreat method."""
    transform = TransformStamped()
    transform.transform.translation.x = 0.31
    transform.transform.translation.y = -0.07
    transform.transform.translation.z = 0.12
    transform.transform.rotation.w = 1.0
    planner = CartesianClient()
    response = GetCartesianPath.Response()
    response.error_code.val = error_code
    response.fraction = fraction
    response.solution.joint_trajectory.points = [
        JointTrajectoryPoint(),
        JointTrajectoryPoint(),
    ]
    calls = []
    controller = SimpleNamespace(
        tf_buffer=SimpleNamespace(
            lookup_transform=lambda *args, **kwargs: transform
        ),
        _lock=threading.RLock(),
        _current_joint_positions=[0.1, 0.2, 0.3, 0.4],
        JOINT_NAMES=ControlNode.JOINT_NAMES,
        retreat_offset=(0.0, 0.0, 0.15),
        base_frame='base_link',
        end_effector_link='end_effector_link',
        server_timeout=1.0,
        result_timeout=1.0,
        move_group_name='arm',
        retreat_cartesian_step=0.005,
        retreat_joint_jump=0.35,
        moveit_velocity_scale=0.1,
        moveit_acceleration_scale=0.1,
        cartesian_path_client=planner,
        execute_trajectory_client=SimpleNamespace(),
        _ensure_active=lambda operation_id: None,
        _wait_future=lambda *args: response,
        _wait_for_server=lambda *args: None,
        _send_goal=lambda *args: calls.append(args[1]) or object(),
        _wait_result=lambda *args: SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(
                error_code=SimpleNamespace(val=MoveItErrorCodes.SUCCESS),
            ),
        ),
        get_logger=lambda: SimpleNamespace(
            info=lambda message: None,
            warn=lambda message: None,
        ),
    )
    return controller, planner, calls


def test_vertical_retreat_uses_current_tool_xy():
    controller, planner, calls = make_controller(1.0)

    ControlNode._send_vertical_retreat(controller, 1)

    waypoint = planner.request.waypoints[0]
    assert (waypoint.position.x, waypoint.position.y) == (0.31, -0.07)
    assert waypoint.position.z == pytest.approx(0.27)
    assert planner.request.header.frame_id == 'base_link'
    assert planner.request.max_step == 0.005
    assert planner.request.avoid_collisions
    assert len(calls) == 1


def test_partial_cartesian_path_is_executed():
    controller, _, calls = make_controller(0.75)

    ControlNode._send_vertical_retreat(controller, 1)

    assert len(calls) == 1


@pytest.mark.parametrize(
    'fraction,error_code',
    [(0.0, MoveItErrorCodes.SUCCESS), (0.5, MoveItErrorCodes.FAILURE)],
)
def test_no_executable_path_still_allows_cargo(fraction, error_code):
    controller, _, calls = make_controller(fraction, error_code)

    ControlNode._send_vertical_retreat(controller, 1)

    assert calls == []
