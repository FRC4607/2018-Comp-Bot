import math
import os
import pathfinder as pf
from constants import X_ROBOT_LENGTH, Y_ROBOT_WIDTH, Y_WALL_TO_START, X_WALL_TO_SWITCH_CENTER
from utilities.functions import GeneratePath


class settings():
    order = pf.FIT_HERMITE_CUBIC
    samples = 1000000
    period = 0.02
    maxVelocity = 7.0
    maxAcceleration = 10
    maxJerk = 30


# The waypoints are entered as X, Y, and Theta. Theta is measured clockwise from the X-axis and
# is in units of radians. It is important to generate the paths in a consistent manner to those
# used by the controller. For example, use either metric or imperial units.  Also, use a
# consistent frame of reference. This means that +X is forward, -X is backward, +Y is right, and
# -Y is left, +headings are going from +X towards +Y, and -headings are going from +X to -Y.
y_offset = 27 - (Y_WALL_TO_START + 0.5 * Y_ROBOT_WIDTH)
waypoints = [
    pf.Waypoint(0.5 * X_ROBOT_LENGTH,           y_offset,       0),
    pf.Waypoint(X_WALL_TO_SWITCH_CENTER - 4,    y_offset + 1.5, 0),
    pf.Waypoint(X_WALL_TO_SWITCH_CENTER,        y_offset - 2,   pf.d2r(-90.0)),
]

GeneratePath(os.path.dirname(__file__), "right_start_right_scale", waypoints, settings)
