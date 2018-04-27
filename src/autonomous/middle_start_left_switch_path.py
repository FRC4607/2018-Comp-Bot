import math
import os
import pathfinder as pf
from collections import namedtuple
from constants import X_ROBOT_LENGTH, Y_ROBOT_WIDTH, Y_WALL_TO_EXCHANGE_FAR, \
    X_WALL_TO_SWITCH_NEAR, Y_WALL_TO_SWITCH_NEAR
from utilities.functions import GeneratePath, GenerateTalonMotionProfileArcPath


#===================================================================================================
# class settings():
#     order = pf.FIT_HERMITE_QUINTIC
#     samples = 1000000
#     period = 0.01
#     maxVelocity = 5.0
#     maxAcceleration = 10
#     maxJerk = 30
# 
# waypoints = [
#     pf.Waypoint(0, 0, 0),
#     pf.Waypoint(100 / 12, -48 / 12, 0),
# ]
# 
# GeneratePath(os.path.dirname(__file__), "middle_start_left_switch", waypoints, settings)
#===================================================================================================

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
    pf.Waypoint(0, 0, 0),
    pf.Waypoint(100 / 12, 60 / 12, 0),
]

GenerateTalonMotionProfileArcPath(os.path.dirname(__file__), "middle_start_left_switch", waypoints, settings)
