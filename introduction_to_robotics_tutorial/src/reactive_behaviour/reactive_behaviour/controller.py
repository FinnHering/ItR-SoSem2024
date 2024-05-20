import math
import random
import statistics
from enum import Enum

import numpy as np
import rclpy
from driving_swarm_utils.node import DrivingSwarmNode, main_fn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TurnDirection(Enum):
    LEFT = -1
    RIGHT = 1


class CollisionType(Enum):
    X = 1
    Y = 2


class VelocityController(DrivingSwarmNode):
    # Distances between [0..1]
    front_distance: float = 1
    right_distance: float = 1
    back_distance: float = 1
    left_distance: float = 1

    # Distance at which the robot does collision avoidance
    MIN_DISTANCE_X = 0.13

    # Maximum speed
    MAX_SPEED = 0.1


    def __init__(self, name: str) -> None:
        super().__init__(name)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.laser_cb, rclpy.qos.qos_profile_sensor_data)
        self.create_timer(0.1, self.timer_cb)
        self.setup_command_interface()

    def timer_cb(self):
        msg = Twist()
        self.publisher.publish(self._get_next_move())

    def laser_cb(self, msg: LaserScan):
        # Get distances of robot
        self.front_distance = (
            min([self._get_distance(msg, x + 0) for x in np.arange(-10, 10, math.degrees(msg.angle_increment))]))

        # self.get_logger().info(
        #     f"Distances: front: {self.front_distance}, right: {self.right_distance}, back: {self.back_distance}, "
        #     f"left: {self.left_distance}")

    def _get_next_move(self):
        res = Twist()
        res.linear.x = self.MAX_SPEED

        self.get_logger().info(
            f"front={self.front_distance}, back={self.back_distance}, right={self.right_distance}, left={self.left_distance}")

        if self.front_distance < self.MIN_DISTANCE_X:
            res.angular.z = math.radians(random.uniform(-180.0, 180.0))
            res.linear.x = 0.0

        return res

    def _get_distance(self, msg: LaserScan, angle: float) -> float:
        """
        @param msg: LaserScan to use as base for distance calculations
        @param angle: angle of sensor to get distance from (deg)
        @return: distance between [msg.range_min..msg.range_max]
        """

        range_idx = self._get_range_idx(angle_start=msg.angle_min, angle_end=msg.range_max,
                                        angle_increment=msg.angle_increment,
                                        angle=angle)

        raw_distance = min(max(msg.ranges[range_idx], msg.range_min), msg.range_max)

        # Calculate relative distance
        return (raw_distance - msg.range_min) / (msg.range_max - msg.range_min)

    @staticmethod
    def _get_range_idx(angle_start: float, angle_end: float, angle_increment: float, angle: float) -> int:
        """

        Helper function for getting distance given angle relative to front of robot

        @param angle_start: start angle of sensor (rad)
        @param angle_end: end angle of sensor (rad)
        @param angle_increment: increment of angles for measurement (rad)
        @param angle: angle to get range idx for (deg)

        @return range index to get distance for
        """

        # convert to degrees because it's easier to comprehend
        angle_start = math.degrees(angle_start)
        angle_end = math.degrees(angle_end)
        angle_increment = math.degrees(angle_increment)

        # normalize angle to 0-359
        angle = angle % 360

        return math.floor(angle / angle_increment)


def main():
    main_fn('controller', VelocityController)


if __name__ == '__main__':
    main()
