import os
import pickle
from wpilib.command import CommandGroup
from utilities.drivetrain_path_follower import DrivetrainPathFollower


class AutonForward(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__), 'forward', 'path1.pickle'), "rb") as fp:
            path1 = pickle.load(fp)

        # Zero gyro and encoders
        robot.driveTrain.zeroGyro()
        robot.driveTrain.zeroQuadratureEncoder()

        # Add commands to run
        self.addSequential(DrivetrainPathFollower(robot, path1, False))
