import math
import random

import rclpy
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class VelocityController(DrivingSwarmNode):

    def __init__(self, name: str) -> None:
        super().__init__(name)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.forward_distance = 1.0
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()
        
    def timer_cb(self):
        msg = Twist()
        if self.forward_distance > 0.8:
            msg.angular.z = math.radians(90*(1-self.forward_distance))
        self.get_logger().info(f"{self.forward_distance}")
        msg.linear.x = min(0.05, self.forward_distance)
        #self.get_logger().info(f"sending: {msg}")
        self.publisher.publish(msg)
    
    def laser_cb(self, msg: LaserScan):
        #self.get_logger().warn(f"Got message: {msg}")
        raw_value = max(min(msg.ranges[int(len(msg.ranges) / 2)], msg.range_max), msg.range_min)
        self.forward_distance = 1-(raw_value - msg.range_min) / (msg.range_max - msg.range_min)


def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
