import math
from queue import Queue
from time import sleep
from typing import Optional

import rclpy
import tf_transformations
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from sympy import Point2D

from state_estimation.goal_rotation_manager import GoalRotationManager


class VelocityController(Node):

    rotation_manager: GoalRotationManager = None

    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 0

        # last known position
        self.position: Queue[Point2D] = Queue(maxsize=1)
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(PoseStamped, 'nav/goal', self.goal_cb, 10)
        self.create_subscription(PointStamped, 'position', self.position_cb, 10)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose_marker', 10)
        self.create_timer(0.1, self.timer_cb)
        self.get_logger().info('controller node started')

        self.V_MAX = 0.25

    async def timer_cb(self):
        if self.rotation_manager is not None:
            rotation = next(self.rotation_manager)
            position = self.rotation_manager.last_known_position
            speed = 0.1
        else:
            speed = 0.0
            rotation = 0.0
            position = None

        if rotation != 0:
            speed = 0.0

        if abs(rotation) > math.pi:
            rotation -= math.copysign(2*math.pi, rotation)

        self.get_logger().info(f"Moving: speed={speed}, rotation={rotation}")

        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = rotation

        self.publisher.publish(msg)

        if position is not None:
            self.publish_marker((float(position.x), float(position.y)), 0, relative=True)

        # if self.state_handler is not None:
        #     p = next(self.state_handler)
        #
        #     if p is not None:
        #         self.publish_marker((float(p.x), float(p.y)), 0, relative=True)

    def goal_cb(self, msg: PoseStamped):

        goal = Point2D(msg.pose.position.x, msg.pose.position.y)

        if self.rotation_manager is None:
            self.get_logger().info(f"GOT NEW GOAL! {goal}")
            self.rotation_manager = GoalRotationManager(goal=goal, logger=self.get_logger())
        elif self.rotation_manager.goal != goal:
            self.get_logger().info(f"GOT NEW GOAL! {goal}")
            self.rotation_manager.new_goal(goal)

        # if self.state_handler is None or self.state_handler.goal != goal:
        #     self.get_logger().info(f"Got new GOAL! {goal.x}, {goal.y}")
        #     self.state_handler = ControllerStateHandler(v_max=0.1, max_rads=1.0, logger=self.get_logger(),
        #                                                 publisher=self.publisher, goal=goal)

    def laser_cb(self, msg):
        self.forward_distance = msg.ranges[0]

    def position_cb(self, msg: PointStamped):
        """
        Callback for position data from locator
        @param msg: Location information
        """
        time = msg.header.stamp.sec
        if self.rotation_manager is not None:
            self.rotation_manager.new_position(Point2D(float(msg.point.x), float(msg.point.y)), time)

        # if self.state_handler is None:
        #     self.get_logger().info("State handler is empty. Skipping position update...")
        #     return
        # else:
        #     self.state_handler.update_position(Point2D(round(float(msg.point.x), 1), round(float(msg.point.y), 1)))

    def publish_marker(self, position, angle, relative=False):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if not relative:
            msg.header.frame_id = 'map'
        else:
            msg.header.frame_id = 'base_link'

        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, angle)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pose_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = VelocityController()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
