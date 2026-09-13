"""Navigator node: A* path planning with obstacle avoidance.

Uses perception data for obstacle positions, basic_motion for movement execution.
"""

import math
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger

from uv_msgs.msg import ObjectPositionArray, PoseInfo, Waypoint, WaypointPath
from uv_msgs.srv import RunTask

from uv_nav.astar import AStarPlanner


class NavigatorNode(Node):
    """Navigator node: A* path planning and execution."""

    def __init__(self):
        super().__init__('navigator')

        self.state = PoseInfo()
        self._have_pose = False
        self.obstacles = []  # list of (x, y)
        self._state_lock = threading.Lock()

        self.planner = AStarPlanner(resolution=0.5, safe_radius=2.0)

        # Subscribers
        # Pose comes from basic_motion (works in sim AND on the real AUV).
        # /auv/state has no publisher and must not be used.
        self.create_subscription(PoseInfo, '/basic_motion/pose_info', self._state_cb, 10)
        self.create_subscription(ObjectPositionArray, '/perception/objects', self._objects_cb, 10)

        # Publishers
        self.pub_path = self.create_publisher(WaypointPath, '/nav/path', 10)

        # Services
        self.create_service(RunTask, '/nav/navigate_to', self._navigate_to_cb)

        self.get_logger().info(
            'Navigator node started (pose source: /basic_motion/pose_info)')

    def _state_cb(self, msg: PoseInfo):
        with self._state_lock:
            self.state = msg
            self._have_pose = True

    def _objects_cb(self, msg: ObjectPositionArray):
        """Update obstacle list from perception."""
        self.obstacles = []
        for obj in msg.objects:
            self.obstacles.append((obj.world_x, obj.world_y))

    def navigate_to(self, goal_x: float, goal_y: float,
                    goal_z: float = None, goal_yaw: float = None) -> bool:
        """Navigate to goal using A* + basic_motion.

        This method is designed to be called from task_runner.
        In the current implementation, it publishes the path for visualization
        and logs navigation commands. Actual movement execution would require
        importing basic_motion or using ROS2 actions.

        Args:
            goal_x, goal_y: Goal position (world NED)
            goal_z: Goal depth (optional)
            goal_yaw: Goal yaw in radians (optional)

        Returns:
            True if navigation planned successfully
        """
        with self._state_lock:
            s = self.state
            have_pose = self._have_pose

        if not have_pose:
            self.get_logger().warn(
                'navigate_to rejected: no pose yet on /basic_motion/pose_info '
                '(is basic_motion running and has START been sent?)')
            return False

        # PoseInfo carries odom-frame NED position in metres and yaw in DEGREES.
        start_x, start_y, start_z = s.robot_x, s.robot_y, s.robot_z
        start_yaw_rad = math.radians(s.robot_yaw)

        # Plan path
        next_x, next_y = self.planner.plan(
            start_x, start_y, goal_x, goal_y, self.obstacles
        )

        # Publish planned path (Waypoint.yaw is in radians)
        path_msg = WaypointPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()

        wp_start = Waypoint()
        wp_start.x = start_x
        wp_start.y = start_y
        wp_start.z = start_z
        wp_start.yaw = start_yaw_rad
        path_msg.waypoints.append(wp_start)

        wp_next = Waypoint()
        wp_next.x = next_x
        wp_next.y = next_y
        wp_next.z = goal_z if goal_z is not None else start_z
        wp_next.yaw = goal_yaw if goal_yaw is not None else start_yaw_rad
        path_msg.waypoints.append(wp_next)

        self.pub_path.publish(path_msg)

        self.get_logger().info(
            f'Navigation: ({start_x:.1f}, {start_y:.1f}) -> '
            f'({next_x:.1f}, {next_y:.1f}) -> ({goal_x:.1f}, {goal_y:.1f})'
        )

        return True

    def _navigate_to_cb(self, request, response):
        """Service handler for navigate_to."""
        # Parse task name as "x,y" or "x,y,z"
        try:
            parts = request.task_name.split(',')
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2]) if len(parts) > 2 else None
            success = self.navigate_to(x, y, z)
            response.success = success
            response.message = 'Navigation planned' if success else 'Navigation failed'
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is raised when the signal handler already
        # shut the context down (e.g. Ctrl-C / SIGTERM via ros2 launch).
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
