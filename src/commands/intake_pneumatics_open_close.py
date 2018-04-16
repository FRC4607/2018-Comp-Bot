from wpilib.command import Command
from wpilib.doublesolenoid import DoubleSolenoid
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class IntakePneumaticsOpenClose(Command):

    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        self.requires(robot.intakePneumatics)
        self.finished = True

    def initialize(self):
        self.finished = False

    def execute(self):
        if self.robot.intakePneumatics.solenoid.get() == DoubleSolenoid.Value.kForward:
            self.robot.intakePneumatics.solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.robot.intakePneumatics.solenoid.set(DoubleSolenoid.Value.kForward)
        self.finished = True

    def isFinished(self):
        return self.finished
