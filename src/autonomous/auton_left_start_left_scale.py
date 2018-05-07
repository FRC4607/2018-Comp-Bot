import os
import pickle
from wpilib.command import CommandGroup
from utilities.drivetrain_path_follower import DrivetrainPathFollower
from commands.shoot_cube_into_scale import ShootCubeIntoScale


class AutonLeftStartLeftScale(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__), 'left_start_left_scale.pickle'), "rb") as fp:
            path = pickle.load(fp)

        # Add commands to run
        self.addSequential(DrivetrainPathFollower(robot, path, False))
        self.addParallel(ShootCubeIntoScale(robot))
