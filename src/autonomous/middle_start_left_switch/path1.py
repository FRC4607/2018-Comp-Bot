import math
import os
from collections import namedtuple
import pathfinder as pf
from utilities.functions import GenerateTalonMotionProfileArcPath
from constants import X_ROBOT_LENGTH, Y_ROBOT_WIDTH, Y_WALL_TO_EXCHANGE_FAR

PathFinderSettings = namedtuple("PathFinderSettings", ["order", "samples", "period", "maxVelocity", "maxAcceleration", "maxJerk"])
settings = PathFinderSettings(order=pf.FIT_HERMITE_QUINTIC,
                              samples=1000000,
                              period=0.01,
                              maxVelocity=5.0,
                              maxAcceleration=10,
                              maxJerk=30)

# The waypoints are entered as X, Y, and Theta.  +X is forward, +Y is left, and +Theta is measured from +X to +Y
xOffset = 0.5 * X_ROBOT_LENGTH
yOffset = -(Y_WALL_TO_EXCHANGE_FAR + 0.5 * Y_ROBOT_WIDTH)

waypoints = [
    pf.Waypoint(xOffset, yOffset, 0),
    pf.Waypoint(100 / 12 + xOffset,  -48 / 12 + yOffset, 0),
]

GenerateTalonMotionProfileArcPath(os.path.dirname(__file__), "path1", waypoints, settings)
