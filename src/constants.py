import enum
import logging

"""
LOGGING CONSTANTS
"""
LOGGER_LEVEL = logging.INFO

"""
ROBOT MEASUREMENTS
"""
X_ROBOT_LENGTH = 39 / 12
Y_ROBOT_WIDTH = 33.5 / 12
ROBOT_WHEELBASE_FT = 1.9614
ROBOT_WHEEL_DIAMETER_FT = 0.5  # ***** TODO *****

"""
DRIVETRAIN MEASUREMENTS
"""
# Practice drivetrain
DRIVETRAIN_RIGHT_KV = 0.9478                # V / ft / s
DRIVETRAIN_RIGHT_KA = 0.2251                # V / ft / s^2
DRIVETRAIN_RIGHT_V_INTERCEPT = 1.0913       # V
DRIVETRAIN_LEFT_KV = 1.0641                 # V / ft / s
DRIVETRAIN_LEFT_KA = 0.1961                 # V / ft / s^2
DRIVETRAIN_LEFT_V_INTERCEPT = 0.9611        # V
DRIVETRAIN_MAX_VELOCITY = 10.0              # ft / s
DRIVETRAIN_MAX_ACCELERATION = 14.5          # ft / s^2

# comp bot drivetrain
#===================================================================================================
# DRIVETRAIN_RIGHT_KV = 0.9478                # V / ft / s
# DRIVETRAIN_RIGHT_KA = 0.2251                # V / ft / s^2
# DRIVETRAIN_RIGHT_V_INTERCEPT = 1.0913       # V
# DRIVETRAIN_LEFT_KV = 0.9478                 # V / ft / s
# DRIVETRAIN_LEFT_KA = 0.2251                 # V / ft / s^2
# DRIVETRAIN_LEFT_V_INTERCEPT = 1.0913        # V
# DRIVETRAIN_MAX_VELOCITY = 10.0              # ft / s
# DRIVETRAIN_MAX_ACCELERATION = 14.5          # ft / s^2
#===================================================================================================


"""
AUTONOMOUS MEASUREMENTS
"""
# Field measurements
# These measurements should be field verified.  X_WALL_TO_CROSSOVER_FT = 21.79 is the centerline
# between the switch and the platform with about a foot of margin on either side.
# X_WALL_TO_SWITCH_FT = 17.42 is the measurement from the back wall (where the drivers are) to the
# side of the switch which faces the scale.  Y_WALL_TO_SWITCH_FT = 18.1 is the measurement from
# the side wall (where the refs are) to the center of the far switch platform.  Y_WALL_TO_START =
# 2.5 is the measurement fromt the side wall to the starting postion of the robot (when it's
# against the back wall and the 45deg wall)

X_WALL_TO_SCALE_NEAR = 299.65 / 12
X_WALL_TO_SWITCH_NEAR = 140 / 12
X_WALL_TO_SWITCH_FAR = 196 / 12
X_WALL_TO_NULL_ZONE = 288 / 12
X_WALL_TO_PLATFORM = 261.47 / 12
X_WALL_TO_START = 36 / 12
X_WALL_TO_CROSSOVER = X_WALL_TO_SWITCH_FAR + 16 / 12
X_WALL_TO_SWITCH_CENTER = 169 / 12

Y_WALL_TO_SWITCH_NEAR = 82.25 / 12
Y_WALL_TO_SWITCH_FAR = 238.5 / 12
Y_WALL_TO_SCALE_NEAR = 71.57 / 12
Y_WALL_TO_SCALE_FAR = 27 - (71.57 / 12)
Y_WALL_TO_NULL_ZONE = 95.25 / 12
Y_WALL_TO_START = 29.69 / 12
Y_CENTER_TO_SWITCH_CENTER = 60 / 12
Y_WALL_TO_EXCHANGE_FAR = 158 / 12

"""
OPERATOR AND DRIVER CONTANTS
"""
# Intake controller
INTAKE_IN = 0.75
INTAKE_OUT_SCALE = 1.0
INTAKE_OUT_SWITCH_FAST = 0.75
INTAKE_OUT_SWITCH_SLOW = 0.45

# Driver turning gain
DRIVER_Z_ROTATION_GAIN = 0.7

"""
PORT MAPPINGS
"""
# Joysticks
DRIVER_JOYSTICK = 0
OPERATOR_JOYSTICK = 1

# Drivetrain
DRIVETRAIN_FRONT_LEFT_MOTOR = 2
DRIVETRAIN_FRONT_RIGHT_MOTOR = 14
DRIVETRAIN_REAR_LEFT_MOTOR = 10
DRIVETRAIN_REAR_RIGHT_MOTOR = 13
DRIVETRAIN_PIGEON = 15

# Intake
INTAKE_LEFT_MOTOR = 4
INTAKE_RIGHT_MOTOR = 15

# Boom
BOOM_MOTOR = 8

# Pneumatics
INTAKE_SOLENOID = 22
INTAKE_FORWARD_SOLENOID = 0
INTAKE_REVERSE_SOLENOID = 1

"""
TALON SRX CONSTANTS
"""
TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS = 160
TALON_DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS = 10

"""
MISC CONSTANTS
"""
FILE_OUTPUT_PATH = "C:\\Users\\ejmcc\\CIS4607\\git\\"


class BOOM_STATE(enum.IntEnum):
    """
    BOOM STATE MACHINE ENUMERATIONS
    """
    Scale = 0
    Switch = 1
    Intake = 2
    Unknown = 3
