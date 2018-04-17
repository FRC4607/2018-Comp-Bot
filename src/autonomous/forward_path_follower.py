from wpilib.command import Command
from utilities.motion_profile_controller import MotionProfileController
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

        self.robot.driveTrain.initiaizeDrivetrainMotionProfileControllers()

        self.leftFollower = MotionProfileController(self.robot.driveTrain.leftTalon,
                                                    self.path['left'],
                                                    False,
                                                    self.robot.driveTrain.MP_SLOT0_SELECT,
                                                    self.robot.driveTrain.MP_SLOT1_SELECT)
        self.rightFollower = MotionProfileController(self.robot.driveTrain.rightTalon,
                                                     self.path['right'],
                                                     False,
                                                     self.robot.driveTrain.MP_SLOT0_SELECT,
                                                     self.robot.driveTrain.MP_SLOT1_SELECT)
        self.leftFollower.start()
        self.rightFollower.start()

    def execute(self):
        """
        If the left and right path followers have finished following the path, mark this command as
        complete.  Otherwise, call the control method to update and act upon the controllers state
        machine.
        """
        if self.leftFollower.isFinished() and self.rightFollower.isFinished():
            self.finished = True
        else:
            self.leftFollower.control()
            self.rightFollower.control()
