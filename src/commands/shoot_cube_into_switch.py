from wpilib.command import Command
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import INTAKE_OUT_SWITCH_SLOW
import logging
from constants import LOGGER_LEVEL
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class ShootCubeIntoSwitch(Command):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.intakeMotors)
        self.robot = robot
        self.finished = True

    def initialize(self):
        self.finished = False
        self.startTime = self.robot.timer.get()
        self.robot.intakeMotors.rightTalon.setInverted(True)
        self.robot.intakeMotors.leftTalon.setInverted(True)
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                               INTAKE_OUT_SWITCH_SLOW)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput,
                                              INTAKE_OUT_SWITCH_SLOW)

    def execute(self):
        if self.robot.timer.get() - self.startTime > 1.0:
            self.finished = True

    def isFinished(self):
        return self.finished

    def end(self):
        self.robot.intakeMotors.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
        self.robot.intakeMotors.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)
