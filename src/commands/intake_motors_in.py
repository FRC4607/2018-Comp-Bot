from wpilib.command import Command
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import INTAKE_IN, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class IntakeMotorsIn(Command):
    """
    This command will turn on the intake motors and spin them to collect cubes.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.intakeMotors)
        self.robot = robot

    def execute(self):
        self.robot.intakeMotors.rightTalon.setInverted(False)
        self.robot.intakeMotors.leftTalon.setInverted(False)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, INTAKE_IN)
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, INTAKE_IN)

    def end(self):
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
