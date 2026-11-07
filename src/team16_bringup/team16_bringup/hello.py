import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class HelloNode(Node):
    def __init__(self):
        super().__init__('team16_hello')
        self._publisher = self.create_publisher(String, 'team16/status', 10)
        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info('Team 16 workspace is running on ROS 2 Jazzy.')

    def _tick(self):
        message = String()
        message.data = 'ready'
        self._publisher.publish(message)
        self.get_logger().info('Publishing on /team16/status: ready')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
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
