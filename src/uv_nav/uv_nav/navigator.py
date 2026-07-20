"""Navigator node: A* path planning with obstacle avoidance.

Uses perception data for obstacle positions, basic_motion for movement execution.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from uv_msgs.msg import AuvState, ObjectPositionArray, Waypoint, WaypointPath
from uv_msgs.srv import RunTask

from uv_nav.astar import AStarPlanner


class NavigatorNode(Node):
    """Navigator node: A* path planning and execution."""

    def __init__(self):
        super().__init__('navigator')

        self.state = AuvState()
        self.obstacles = []  # list of (x, y)
        self._state_lock = threading.Lock()

        self.planner = AStarPlanner(resolution=0.5, safe_radius=2.0)

        # Subscribers
        self.create_subscription(AuvState, '/auv/state', self._state_cb, 10)
        self.create_subscription(ObjectPositionArray, '/perception/objects', self._objects_cb, 10)

        # Publishers
        self.pub_path = self.create_publisher(WaypointPath, '/nav/path', 10)

        # Services
        self.create_service(RunTask, '/nav/navigate_to', self._navigate_to_cb)

        self.get_logger().info('Navigator node started')

    def _state_cb(self, msg: AuvState):
        with self._state_lock:
            self.state = msg

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
        s = self.state
        start_x, start_y = s.pos_x, s.pos_y

        # Plan path
        next_x, next_y = self.planner.plan(
            start_x, start_y, goal_x, goal_y, self.obstacles
        )

        # Publish planned path
        path_msg = WaypointPath()
        path_msg.header.stamp = self.get_clock().now().to_msg()

        wp_start = Waypoint()
        wp_start.x = start_x
        wp_start.y = start_y
        wp_start.z = s.pos_z
        wp_start.yaw = s.yaw
        path_msg.waypoints.append(wp_start)

        wp_next = Waypoint()
        wp_next.x = next_x
        wp_next.y = next_y
        wp_next.z = goal_z if goal_z is not None else s.pos_z
        wp_next.yaw = goal_yaw if goal_yaw is not None else s.yaw
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
