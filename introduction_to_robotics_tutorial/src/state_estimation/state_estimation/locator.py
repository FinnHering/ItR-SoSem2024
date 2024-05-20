from dataclasses import dataclass

import rclpy
import numpy as np
import scipy.spatial
import sympy
from rclpy.node import Node
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
        self.anchor_ranges: list[Range] = []
        self.create_subscription(Range, 'range', self.range_cb, 10)
        self.position_pub = self.create_publisher(PointStamped, 'position', 10)
        self.initialized = False
        self.create_timer(1.0, self.timer_cb)
        self.get_logger().info('locator node started')

    def range_cb(self, msg: Range):
        self.get_logger().info(str(msg))
        self.anchor_ranges.append(msg)
        self.anchor_ranges = self.anchor_ranges[-10:]
        if not self.initialized:
            self.initialized = True
            self.get_logger().info('first range received')

    def timer_cb(self):
        if not self.initialized:
            return
        msg = PointStamped()
        msg.point.x, msg.point.y, msg.point.z = self.calculate_position()
        msg.header.frame_id = 'world'
        self.position_pub.publish(msg)

    def calculate_position(self):
        unique_ranges = self.get_recent_unique_ranges(3)

        if len(unique_ranges) < 3:
            return 0.0, 0.0, 0.0

        p: sympy.Point2D = self.triangulate(unique_ranges)

        self.get_logger().info(f"X: {float(p.x)}, Y: {float(p.y)}")

        return float(p.x), float(p.y), 0.0

    def triangulate(self, ranges: (Range, Range, Range)) -> sympy.Point2D:
        """
        Given 3 unique anchored ranges calculates the position.
        If the ranges are not unique the result will most likely be wrong!

        @param ranges: range
        @return: position in 3D space
        """

        circles = list(map(self.range_to_circle, ranges))

        result = least_squares(self.triangulate_equation, [0, 0, 0, 0], args=[circles])

        x, y, z, r = result.x

        return sympy.Point(x, y)

    @staticmethod
    def range_to_circle(r: Range) -> Sphere:
        return Sphere(x=r.anchor.x, y=r.anchor.y, z=r.anchor.z, r=r.range)

    @staticmethod
    def triangulate_equation(guess: tuple[float, float, float, float], anchors: list[Sphere]) -> np.array:
        """

        @param guess: (x, y, z, r) of a guessed circle
        @param anchors: anchor circles
        @return: for each circle in anchors the result of the circle inner tangent function
        """

        x, y, z, r = guess

        res = []
        for c in anchors:
            res.append(((x - c.x)**2 + (y - c.y)**2 + (z - c.z)**2 - (r - c.r)**2))

        return np.array(res, dtype=float)

    def get_recent_unique_ranges(self, count: int) -> list[Range]:
        """
        Return recent ranges which have a unique anchor point.

        @param count: amount of ranges to return, count must be >= 1
        @return: at most count ranges or less if range store didn't have enough unique anchored ranges
        """

        if count < 1:
            raise ValueError("Count must be >= 1")

        use_ranges = []
        anchors: set[sympy.Point] = set()

        for r in reversed(self.anchor_ranges):

            # filter out duplicate anchors
            if sympy.Point(r.anchor.x, r.anchor.y) in anchors:
                continue

            anchors.add(sympy.Point(r.anchor.x, r.anchor.y))
            use_ranges.append(r)

            if len(anchors) >= count:
                break

        return use_ranges


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
