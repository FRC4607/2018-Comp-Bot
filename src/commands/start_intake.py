from wpilib.command import Command
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import INTAKE_IN, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class StartIntake(Command):
    """
    This command will turn on the intake motors and spin them to collect cubes.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.intakeMotors)
        self.robot = robot
        self._startTimer = True
        self.finished = True

    def initialize(self):
        self._startTimer = True
        self.finished = False

    def execute(self):
        if self._startTimer:
            self._startTime = self.robot.timer.get()
            self._startTimer = False
        self.robot.intakeMotors.rightTalon.setInverted(False)
        self.robot.intakeMotors.leftTalon.setInverted(False)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, INTAKE_IN)
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, INTAKE_IN)
        if self.robot.timer.get() - self._startTime > 1.0:
            self.finished = True

    def isFinished(self):
        return self.finished

    def end(self):
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
