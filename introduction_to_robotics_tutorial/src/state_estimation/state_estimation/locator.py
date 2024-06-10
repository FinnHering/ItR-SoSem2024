from collections import deque
from dataclasses import dataclass
from time import time
from typing import Deque, Optional

import rclpy
import numpy as np
import scipy.spatial
import sympy
from rclpy.node import Node
from rclpy.time import Time
from scipy.optimize import least_squares
from sympy import Circle

from driving_swarm_messages.msg import Range
from geometry_msgs.msg import PointStamped, Point


@dataclass
class Sphere:
    x: float
    y: float
    z: float
    r: float


class LocatorNode(Node):

    def __init__(self):
        super().__init__('locator_node')
        self.anchor_ranges: Deque[(Range, float)] = deque(maxlen=5)
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')

    def range_cb(self, msg: Range):
        self.get_logger().info(str(msg))
        self.anchor_ranges.append((msg, time()))
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        res = self.calculate_position()
        if res is None:
            return

        pos, old_timestamp = res
        msg.point.x, msg.point.y, msg.point.z = float(pos.x), float(pos.y), float(pos.z)
        msg.header.frame_id = 'world'
        msg.header.stamp = Time(seconds=round(old_timestamp)).to_msg()
        self.position_pub.publish(msg)

    def calculate_position(self) -> Optional[tuple[sympy.Point3D, float]]:
        unique_ranges, oldest_timestamp = self.get_recent_unique_ranges(4)

        if len(unique_ranges) < 4:
            return None

        p: sympy.Point2D = self.triangulate(unique_ranges)

        self.get_logger().info(f"X: {float(p.x)}, Y: {float(p.y)}")

        # reset ranges
        self.anchor_ranges = []

        return sympy.Point3D(float(p.x), float(p.y), 0.0), oldest_timestamp

    def triangulate(self, ranges: list[Range]) -> sympy.Point2D:
        """
        Given 3 unique anchored ranges calculates the position.
        If the ranges are not unique the result will most likely be wrong!

        @param ranges: range
        @return: position in 3D space
        """

        circles = list(map(self.range_to_circle, ranges))

        result = least_squares(self.triangulate_equation, [0, 0, 0], args=[circles], method='lm')

        x, y, z = result.x

        return sympy.Point(x, y)

    @staticmethod
    def range_to_circle(r: Range) -> Sphere:
        return Sphere(x=r.anchor.x, y=r.anchor.y, z=r.anchor.z, r=r.range)

    @staticmethod
    def triangulate_equation(guess: tuple[float, float, float], anchors: list[Sphere]) -> np.array:
        """

        @param guess: (x, y, z, r) of a guessed circle
        @param anchors: anchor circles
        @return: for each circle in anchors the result of the circle inner tangent function
        """

        x, y, z = guess

        res = []
        for c in anchors:
            res.append((((x - c.x) ** 2 + (y - c.y) ** 2 + (z - c.z) ** 2) - c.r ** 2))

        return np.array(res, dtype=float)

    def get_recent_unique_ranges(self, count: int) -> tuple[list[Range], float]:
        """
        Return recent ranges which have a unique anchor point.

        @param count: amount of ranges to return, count must be >= 1
        @return: at most count ranges or less if range store didn't have enough unique anchored ranges
        """

        if count < 1:
            raise ValueError("Count must be >= 1")

        use_ranges = []
        min_timestamp = None
        anchors: set[sympy.Point] = set()

        for r, timestamp in reversed(list(self.anchor_ranges)):

            # filter out duplicate anchors
            if sympy.Point(r.anchor.x, r.anchor.y) in anchors:
                continue

            anchors.add(sympy.Point(r.anchor.x, r.anchor.y))
            use_ranges.append(r)

            if min_timestamp is None or timestamp < min_timestamp:
                min_timestamp = timestamp

            if len(anchors) >= count:
                break

        self.anchor_ranges.clear()

        return use_ranges, min_timestamp


def main(args=None):
    rclpy.init(args=args)

    node = LocatorNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
