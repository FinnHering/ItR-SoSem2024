import math
from collections import deque
from time import time
from typing import Deque, Optional, Final

from rclpy.impl.rcutils_logger import RcutilsLogger
from sympy import Point2D, Ray2D, Line

from state_estimation import constants


class GoalRotationManager:
    """
    Iterator calculating the rotation needed to move towards the goal
    """

    # Deque of size 2. Queue acts like a time window displaying the last two positions known.
    # The oldest position is left
    _position_history: Deque[Point2D] = None

    # Tuple consisting of direction and unique id of that direction
    _robot_direction: Optional[tuple[Ray2D, int]] = None

    # ID of last distance accessed
    _last_direction_id_accessed: Optional[int] = None

    _goal: Point2D

    _rotation_left: Optional[float] = None

    # indicates a new goal
    _new_goal: bool = False

    _last_rotation_timestamp: Optional[int] = None

    _logger: Final[RcutilsLogger]

    @property
    def goal(self) -> Point2D:
        """
        Current goal rotation manager tries to steer towards
        @return: Point of goal in 2D space
        """
        return self._goal.copy()

    @property
    def last_known_position(self) -> Optional[Point2D]:
        """
        Last known position of known.
        @return: Point in 2D space if position is known; None if no position is known
        """
        l_p: list[Point2D] = list(self._position_history)
        if l_p:
            return l_p[-1].copy()
        else:
            return None

    def __init__(self, goal: Point2D, logger: RcutilsLogger):
        self._position_history = deque(maxlen=2)
        self._goal = goal.copy()
        self._logger = logger

    def __next__(self) -> Optional[float]:
        """
        Returns the rotation
        @return rotation/s (rad)
        """

        # no direction known
        if self._robot_direction is None:
            return 0.0

        if self._rotation_left is not None:
            return self._calc_rotation()

        # outdated direction -> rotation needs to be updated
        elif self._last_direction_id_accessed != self._robot_direction[1] or self._new_goal:
            self._new_goal = False
            self._last_direction_id_accessed = self._robot_direction[1]

            direction: Ray2D = self._robot_direction[0]

            current_pos = self._robot_direction[0].p1

            goal_direction: Ray2D = Ray2D(current_pos, self._goal)

            self._rotation_left = goal_direction.closing_angle(direction)

            self._logger.info(
                f"OUR DIRECTION: {(float(direction.direction.x), float(direction.direction.y))} "
                f"| DIRECTION TO GOAL: {(float(goal_direction.direction.x), float(goal_direction.direction.y))} "
                f"| rotation: {math.degrees(self._rotation_left)}°")

        return self._calc_rotation()

    def new_position(self, new_position: Point2D, timestamp: int):
        """
        Report a new known position of the robot
        @param new_position: new current position
        @param timestamp: unixtime from where the position was measured
        """

        # if we're rotating ignore position
        if self._rotation_left is not None:
            return

        self._position_history.append(new_position)

        if len(self._position_history) < self._position_history.maxlen:
            return
        else:

            self._logger.info(f"{list(map(lambda p: (float(p.x), float(p.y)), list(self._position_history)))}")
            pl = list(self._position_history)
            p1: Point2D = pl[0]
            p2: Point2D = pl[1]

            self._logger.info(f"{Line(p1, p2).direction}")

            # id of distance if timestamp in ms
            self._robot_direction = (Ray2D(p1, p2), round(time() * 1000))

    def new_goal(self, new_goal: Point2D):
        """
        Report new goal to move towards
        @param new_goal: new goal to move towards
        @return:
        """

        self._goal = new_goal.copy()
        self._rotation_left = None
        self._new_goal = True

    def _calc_rotation(self) -> float:
        """
        Get rotation parameter needed for the current tick
        @return: rotation in rads
        """

        if self._rotation_left is None:
            r = 0.0

        # Rotation left can be done in one tick. Reset rotation_left to None and send final rotation
        elif abs(self._rotation_left) < constants.R_MAX_PER_TICK:
            r = self._rotation_left * constants.TICKS_PER_SECOND
            self._rotation_left = None
            self._position_history.clear()
            self._last_rotation_timestamp = round(time())
        else:
            r = math.copysign(constants.R_MAX_PER_TICK * constants.TICKS_PER_SECOND, self._rotation_left)
            self._rotation_left -= math.copysign(constants.R_MAX_PER_TICK, self._rotation_left)

        return float(r)
