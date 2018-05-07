import math
import os
import pathfinder as pf
from collections import namedtuple
from constants import X_ROBOT_LENGTH, Y_ROBOT_WIDTH, Y_WALL_TO_START, X_WALL_TO_SWITCH_CENTER
from utilities.functions import GeneratePath, GenerateTalonMotionProfileArcPath


#===================================================================================================
# class settings():
#     order = pf.FIT_HERMITE_CUBIC
#     samples = 1000000
#     period = 0.02
#     maxVelocity = 7.0
#     maxAcceleration = 10
#     maxJerk = 30
#  
# waypoints = [
#     pf.Waypoint(0.5 * X_ROBOT_LENGTH,           Y_WALL_TO_START + 0.5 * Y_ROBOT_WIDTH,       0),
#     pf.Waypoint(X_WALL_TO_SWITCH_CENTER - 4,    Y_WALL_TO_START + 0.5 * Y_ROBOT_WIDTH - 1.5, 0),
#     pf.Waypoint(X_WALL_TO_SWITCH_CENTER,        Y_WALL_TO_START + 0.5 * Y_ROBOT_WIDTH + 2,   pf.d2r(90.0)),
# ]
#  
# GeneratePath(os.path.dirname(__file__), "left_start_left_switch", waypoints, settings)
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
yOffset = -(Y_WALL_TO_START + 0.5 * Y_ROBOT_WIDTH)

waypoints = [
    pf.Waypoint(xOffset, yOffset, 0),
    pf.Waypoint(X_WALL_TO_SWITCH_CENTER - 48 /12,  18 / 12 + yOffset, 0),
    pf.Waypoint(X_WALL_TO_SWITCH_CENTER,           -24 / 12 + yOffset, pf.d2r(90.0)),
]



GenerateTalonMotionProfileArcPath(os.path.dirname(__file__), "left_start_left_switch", waypoints, settings)
