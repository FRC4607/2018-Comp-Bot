from wpilib.command import Command
from utilities.drivetrain_mp_controller import DrivetrainMPController
import logging
from constants import LOGGER_LEVEL
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class DrivetrainPathFollower(Command):
    """
    This command will call the path which will go forward. The pickle file should have been
    created on the PC and placed into the autonomous folder to be uploaded with the robot code.
    """
    def __init__(self, robot, path, reverse, pid_kludge=False):
        super().__init__()
        self.requires(robot.driveTrain)
        # self.setInterruptible(False)

        # Create references to the robot and the path to follow
        self.robot = robot
        self.path = path
        self.reverse = reverse
        self.pidKludge = pid_kludge

        # Control variables
        self.finished = True

        # 4th value in MP's is sample period.  Assume the left and right sides are the same.  The
        # divide by 2 value is used to set the Talon control frames and notifier to twice the rate
        # of the trajectory duration.
        self._streamRate = int(self.path['left'][0][3] / 2)

    def isFinished(self):
        """
        This returns whether or not this command is complete.
        """
        return self.finished

    def initialize(self):
        """
        Create the left and right path follower controller objects and start the process of the
        following the path.
        """
        self.finished = False
        self.robot.driveTrain.initiaizeDrivetrainMotionProfileControllers(self._streamRate)
        if self.pidKludge:
            self.robot.driveTrain.pidKludge()
        self.pathFollower = DrivetrainMPController(self.robot.driveTrain.leftTalon,
                                                   self.path['left'],
                                                   self.robot.driveTrain.rightTalon,
                                                   self.path['right'],
                                                   self.reverse,
                                                   self.robot.driveTrain.MP_SLOT0_SELECT,
                                                   self.robot.driveTrain.MP_SLOT1_SELECT)
        self.pathFollower.start()

    def execute(self):
        """
        If the path followers has finished following the path, mark this command as complete.
        Otherwise, call the control method to update and act upon the controllers state machine.
        """
        if self.pathFollower.isFinished():
            self.finished = True
        else:
            self.pathFollower.control(self.robot.timer.get(), self.robot.smartDashboard)

    def end(self):
        '''
        Exit the DrivetrainMotionProfileControllers
        '''
        self.robot.driveTrain.cleanUpDrivetrainMotionProfileControllers()
