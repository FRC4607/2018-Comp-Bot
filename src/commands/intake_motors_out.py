from wpilib.command import Command
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class IntakeMotorsOut(Command):
    """
    This command will turn on the intake motors and spin them to shoot the cubes out.  This command
    is specifically used for switch and has a low power launch.
    """
    def __init__(self, robot, motor_speed):
        super().__init__()
        self.requires(robot.intakeMotors)
        self.robot = robot
        self.motorSpeed = motor_speed

    def execute(self):
        self.robot.intakeMotors.rightTalon.setInverted(True)
        self.robot.intakeMotors.leftTalon.setInverted(True)
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                               self.motorSpeed)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                              self.motorSpeed)

    def end(self):
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
