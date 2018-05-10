import os
from collections import namedtuple
import pathfinder as pf
from utilities.functions import GenerateTalonMotionProfileArcPath
from constants import X_ROBOT_LENGTH, Y_ROBOT_WIDTH, X_WALL_TO_SWITCH_FAR, X_WALL_TO_SCALE_NEAR, Y_WALL_TO_START, Y_WALL_TO_SCALE_NEAR

PathFinderSettings = namedtuple("PathFinderSettings", ["order", "samples", "period", "maxVelocity", "maxAcceleration", "maxJerk"])
settings = PathFinderSettings(order=pf.FIT_HERMITE_QUINTIC,
                              samples=1000000,
                              period=0.01,
                              maxVelocity=7.0,
                              maxAcceleration=10,
                              maxJerk=30)

# The waypoints are entered as X, Y, and Theta.  +X is forward, +Y is left, and +Theta is measured from +X to +Y
xOffset = 0.5 * X_ROBOT_LENGTH
yOffset = -(Y_WALL_TO_START + 0.5 * Y_ROBOT_WIDTH)

waypoints = [
     pf.Waypoint(xOffset,                         yOffset, 0),
     pf.Waypoint(xOffset,                         yOffset, 0),
]
# This function will generate the path using pathfinder and then convert the output into Talon Motion Profile Arc inputs.
#   path_name:        This is the file system path to where the pickled path file will be created
#   file_name:        This is the file name of the pickled path file
#   waypoints:        These are the waypoints fed into pathfinder
#   settings:         These are the pathfinder settings used to generate the path
#   reverse:          This boolean flag will tell the function that the robot will follow this path goin backwards
#   heading_override: This boolean flag will tell the function to ignore the heading values of pathfinder
#   heading_value:    This is used in conjunction with heading_override and defines the heading value to use in lieu of pathfinder's
GenerateTalonMotionProfileArcPath(os.path.dirname(__file__), "path5", waypoints, settings)
