from wpilib.command import Command
from wpilib.doublesolenoid import DoubleSolenoid
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class CloseIntake(Command):

    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.requires(robot.intakePneumatics)
        self.finished = True

    def initialize(self):
        self.finished = False

    def execute(self):
        self.robot.intakePneumatics.solenoid.set(DoubleSolenoid.Value.kReverse)
        self.finished = True

    def isFinished(self):
        return self.finished
