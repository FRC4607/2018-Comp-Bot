from wpilib.command import Command
from constants import DRIVER_Z_ROTATION_GAIN, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class DriveJoystick(Command):
    """
    This command will use the driver inputs to drive the robot.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)
        self.robot = robot

    def execute(self):

        # Get the joystick inputs
        xSpeed = -self.robot.oi.driverJoystick.getY()
        zRotation = self.robot.oi.driverJoystick.getX() * DRIVER_Z_ROTATION_GAIN

        # Apply a joystick deadband
        if xSpeed < 0.05 and xSpeed > -0.05:
            xSpeed = 0.0
        if zRotation < 0.05 and zRotation > -0.05:
            zRotation = 0.0

        # Blake likes the arcade drive best.
        self.robot.driveTrain.diffDrive.arcadeDrive(xSpeed, zRotation)
