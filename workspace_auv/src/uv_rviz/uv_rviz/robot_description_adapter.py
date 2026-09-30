"""Forward the active robot_state_publisher URDF to Foxy RViz RobotModel."""

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from auv_protocol.topics import VIZ_ROBOT_DESCRIPTION


class RobotDescriptionAdapter(Node):
    """Read the running description parameter without publishing any TF."""

    def __init__(self):
        super().__init__('uv_rviz_robot_description_adapter')
        self.declare_parameter(
            'robot_state_publisher_node', '/robot_state_publisher')
        node_name = str(self.get_parameter(
            'robot_state_publisher_node').value).strip()
        if not node_name.startswith('/'):
            node_name = '/' + node_name
        node_name = node_name.rstrip('/')

        self._parameter_client = self.create_client(
            GetParameters, node_name + '/get_parameters')
        self._description_publisher = self.create_publisher(
            String,
            VIZ_ROBOT_DESCRIPTION,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._future = None
        self._description = None
        self._last_subscription_count = 0
        self._warned_empty = False
        self._timer = self.create_timer(0.5, self._poll)

    def _poll(self):
        if self._description is None:
            if self._future is None:
                if not self._parameter_client.service_is_ready():
                    return
                request = GetParameters.Request()
                request.names = ['robot_description']
                self._future = self._parameter_client.call_async(request)
                return

            if not self._future.done():
                return

            future = self._future
            self._future = None
            try:
                response = future.result()
            except Exception as error:  # retry after transient service failures
                self.get_logger().debug(
                    'Waiting for robot_description parameter: {}'.format(error))
                return

            if not response.values or not response.values[0].string_value:
                if not self._warned_empty:
                    self.get_logger().warning(
                        'robot_state_publisher has no non-empty '
                        'robot_description parameter yet')
                    self._warned_empty = True
                return

            self._description = response.values[0].string_value
            self.get_logger().info(
                'Loaded the active robot_state_publisher description')

        subscriber_count = self._description_publisher.get_subscription_count()
        if subscriber_count > self._last_subscription_count:
            message = String()
            message.data = self._description
            self._description_publisher.publish(message)
            self.get_logger().info(
                'Published robot model on {}'.format(VIZ_ROBOT_DESCRIPTION))
        self._last_subscription_count = subscriber_count


def main(args=None):
    rclpy.init(args=args)
    node = RobotDescriptionAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
