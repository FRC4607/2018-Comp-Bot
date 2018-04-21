from wpilib import Joystick
from wpilib.buttons import JoystickButton as btn
from commands.intake_motors_in import IntakeMotorsIn
from commands.intake_motors_out import IntakeMotorsOut
from commands.intake_pneumatics_open_close import IntakePneumaticsOpenClose
from commands.boom_to_switch import BoomToSwitch
from commands.boom_to_intake import BoomToIntake
from commands.boom_to_scale import BoomToScale
from constants import DRIVER_JOYSTICK, OPERATOR_JOYSTICK, INTAKE_OUT_SWITCH_FAST, \
    INTAKE_OUT_SWITCH_SLOW, INTAKE_OUT_SCALE, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class OI(object):
    """
    The operator interface controls how the driver and operator control the robot.
    """
    def __init__(self, robot):

        # Setup the joystick for the driver
        self.driverJoystick = Joystick(DRIVER_JOYSTICK)
        # self.driverRightAnalog = Joystick(DRIVER_JOYSTICK)
        # self.driverRightAnalog.setXChannel(4)
        # self.driverRightAnalog.setYChannel(5)

        # Setup the joystick for the operator
        self.operatorJoystick = Joystick(OPERATOR_JOYSTICK)
        # self.operatorRightAnalog = Joystick(DRIVER_JOYSTICK)
        # self.operatorRightAnalog.setXChannel(4)
        # self.operatorRightAnalog.setYChannel(5)

        # Driver commands and control
        btn(self.driverJoystick, 1).whileHeld(IntakeMotorsIn(robot))
        btn(self.driverJoystick, 2).whileHeld(IntakeMotorsOut(robot, INTAKE_OUT_SWITCH_FAST))
        btn(self.driverJoystick, 3).whileHeld(IntakeMotorsOut(robot, INTAKE_OUT_SCALE))
        btn(self.driverJoystick, 4).whileHeld(IntakeMotorsOut(robot, INTAKE_OUT_SWITCH_SLOW))

        # Operator commands and control
        # Per Dylan, boom control A-intake X-switch Y- scale
        btn(self.operatorJoystick, 1).whenPressed(IntakePneumaticsOpenClose(robot))
        btn(self.operatorJoystick, 3).whenPressed(BoomToSwitch(robot))
        btn(self.operatorJoystick, 2).whenPressed(BoomToIntake(robot))
        btn(self.operatorJoystick, 4).whenPressed(BoomToScale(robot))
