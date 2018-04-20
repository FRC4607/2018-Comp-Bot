from wpilib.command import Command
from utilities.drivetrain_mp_controller import DrivetrainMPController
import os.path
import pickle
import logging
from constants import LOGGER_LEVEL
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class ForwardPathFollower(Command):
    """
    This command will call the path which will go forward. The pickle file should have been
    created on the PC and placed into the autonomous folder to be uploaded with the robot code.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)
        self.setInterruptible(False)

        # Create references to the robot
        self.robot = robot

        # Read up the pickled path file
        with open(os.path.join(os.path.dirname(__file__), 'forward_path.pickle'), "rb") as fp:
            self.path = pickle.load(fp)

        # Control variables
        self.finished = True
        self._streamRate = int(self.path['left'][0][2] / 2)

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
        self.pathFollower = DrivetrainMPController(self.robot.driveTrain.leftTalon,
                                                   self.path['left'],
                                                   self.robot.driveTrain.rightTalon,
                                                   self.path['right'],
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
