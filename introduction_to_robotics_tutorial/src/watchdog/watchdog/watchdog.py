import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class WatchdogNode(Node):
    is_started: bool = False

    def __init__(self):
        super().__init__('watchdog')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'input_cmd', self.cmd_callback, 10)
        self.create_subscription(String, 'controller_cmd', self.controller_callback, 10)
        self.get_logger().info('Watchdog node started')

    def cmd_callback(self, msg):
        self.get_logger().info(f"GOT MESSAGE!!! -> {msg}")
        if self.is_started and type(msg) is Twist:
            msg: Twist = msg
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            self.get_logger().info("Turtle is started and Twist was send. Removing angular motion")
        elif not self.is_started and type(msg) is Twist:
            msg: Twist = msg
            msg.linear.x = 0
            msg.linear.y = 0
            msg.linear.z = 0
            self.get_logger().info("Turtle is not started and Twist was send. Removing linear motion")
        # this makes the turle go backwards
        # (just so you know its working)
        msg.linear.x = -1 * msg.linear.x
        self.get_logger().info(f'msg.linear.x: {msg.linear.x}')
        self.publisher.publish(msg)

    def controller_callback(self, msg):
        if msg.data == "start":
            self.is_started = True
        elif msg.data == "stop":
            self.is_started = False

        self.get_logger().warn(f'The controller says I should {msg.data} the turtle ...')
        self.get_logger().info(f"GOT MESSAGE!!! {msg}")


def main(args=None):
    rclpy.init(args=args)

    node = WatchdogNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
