import math
import os
import pathfinder as pf
from collections import namedtuple
from constants import X_ROBOT_LENGTH, Y_ROBOT_WIDTH, Y_WALL_TO_START, X_WALL_TO_SCALE_NEAR, \
    Y_WALL_TO_SCALE_FAR, X_WALL_TO_SWITCH_FAR, Y_WALL_TO_EXCHANGE_FAR
from utilities.functions import GeneratePath, GenerateTalonMotionProfileArcPath

#===================================================================================================
# class settings():
#     order = pf.FIT_HERMITE_QUINTIC
#     samples = 1000000
#     period = 0.02
#     maxVelocity = 7.0
#     maxAcceleration = 10
#     maxJerk = 30
# 
# # The waypoints are entered as X, Y, and Theta. Theta is measured clockwise from the X-axis and
# # is in units of radians. It is important to generate the paths in a consistent manner to those
# # used by the controller. For example, use either metric or imperial units.  Also, use a
# # consistent frame of reference. This means that +X is forward, -X is backward, +Y is right, and
# # -Y is left, +headings are going from +X towards +Y, and -headings are going from +X to -Y.
# waypoints = [
#     pf.Waypoint(0.5 * X_ROBOT_LENGTH,                                                                       Y_WALL_TO_SCALE_FAR + 0.5 * Y_ROBOT_WIDTH, 0),
#     pf.Waypoint(X_WALL_TO_SWITCH_FAR - 0.5 * X_ROBOT_LENGTH,                                                Y_WALL_TO_SCALE_FAR + 0.5 * Y_ROBOT_WIDTH, 0),
#     pf.Waypoint(X_WALL_TO_SCALE_NEAR + math.sin(pf.d2r(20.0)) * 0.5 * Y_ROBOT_WIDTH - 0.5 * X_ROBOT_LENGTH, Y_WALL_TO_SCALE_FAR,                  pf.d2r(-20.0)),
# ]
# 
# GeneratePath(os.path.dirname(__file__), "right_start_right_scale", waypoints, settings)
#===================================================================================================

PathFinderSettings = namedtuple("PathFinderSettings", ["order", "samples", "period", "maxVelocity", "maxAcceleration", "maxJerk"])
settings = PathFinderSettings(order=pf.FIT_HERMITE_QUINTIC,
                              samples=1000000,
                              period=0.02,
                              maxVelocity=7.0,
                              maxAcceleration=10,
                              maxJerk=30)

# The waypoints are entered as X, Y, and Theta.  +X is forward, +Y is left, and +Theta is measured from +X to +Y
xOffset = 0.5 * X_ROBOT_LENGTH
yOffset = -(Y_WALL_TO_EXCHANGE_FAR + 0.5 * Y_ROBOT_WIDTH)

waypoints = [
     pf.Waypoint(0, 0, 0),
     pf.Waypoint(X_WALL_TO_SWITCH_FAR - 0.5 * X_ROBOT_LENGTH,                                                Y_WALL_TO_SCALE_FAR + 0.5 * Y_ROBOT_WIDTH, 0),
     pf.Waypoint(X_WALL_TO_SCALE_NEAR + math.sin(pf.d2r(20.0)) * 0.5 * Y_ROBOT_WIDTH - 0.5 * X_ROBOT_LENGTH, Y_WALL_TO_SCALE_FAR,                  pf.d2r(-20.0)),
]

GenerateTalonMotionProfileArcPath(os.path.dirname(__file__), "left_start_left_scale", waypoints, settings)
