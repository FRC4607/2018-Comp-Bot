from wpilib.command import Command
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class CharacterizeDrivetrain(Command):
    """
    This command will use the driver inputs to drive the robot.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)
        self.robot = robot
        self.finished = True

    def initialize(self):
        self.robot.driveTrain.initQuadratureEncoder()
        self.robot.driveTrain.setQuadratureStatusFramePeriod(20)
        self.startTimer = True
        self.finished = False

    def execute(self):

        if self.startTimer:
            self.startTime = self.robot.timer.get()
            self.startTimer = False

        if self.robot.timer.get() - self.startTime < 2.0:
            self.robot.smartDashboard.putNumber("rVelocity",
                                                self.robot.driveTrain.getRightVelocity())
            self.robot.smartDashboard.putNumber("lVelocity",
                                                self.robot.driveTrain.getLeftVelocity())
            self.robot.smartDashboard.putNumber("rVoltage",
                                                self.robot.driveTrain.getRightVoltage())
            self.robot.smartDashboard.putNumber("lVoltage",
                                                self.robot.driveTrain.getLeftVoltage())
            self.robot.smartDashboard.putNumber("TimeStamp",
                                                self.robot.timer.get())
            self.robot.driveTrain.diffDrive.arcadeDrive(0.4, 0.0)
        else:
            self.finished = True
            self.robot.driveTrain.diffDrive.arcadeDrive(0.0, 0.0)

    def isFinished(self):
        return self.finished
