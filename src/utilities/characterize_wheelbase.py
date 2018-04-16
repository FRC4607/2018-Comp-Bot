from wpilib.command import Command
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class MeasureWheelbase(Command):
    """
    This command will use the driver inputs to drive the robot.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)
        self.robot = robot

    def initialize(self):
        self.robot.driveTrain.initQuadratureEncoder()
        self.robot.driveTrain.setQuadratureStatusFramePeriod(20)
        self.robot.driveTrain.navX.reset()

    def execute(self):
        self.robot.smartDashboard.putNumber("lPosition",
                                            self.robot.driveTrain.getLeftQuadraturePosition())
        self.robot.smartDashboard.putNumber("rPosition",
                                            self.robot.driveTrain.getRightQuadraturePosition())
        self.robot.smartDashboard.putNumber("Gyro", self.robot.driveTrain.navX.getAngle())
        self.robot.smartDashboard.putNumber("TimeStamp", self.robot.timer.get())

        # Get the joystick inputs
        zRotation = self.robot.oi.driverJoystick.getX()

        # Apply a joystick deadband
        if zRotation < 0.10 and zRotation > -0.10:
            zRotation = 0.0

        # Blake likes the arcade drive best.
        self.robot.driveTrain.diffDrive.arcadeDrive(0.0, zRotation)
