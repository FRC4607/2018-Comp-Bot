import os
import pickle
from wpilib.command import CommandGroup
from utilities.drivetrain_path_follower import DrivetrainPathFollower
from commands.shoot_cube_into_scale import ShootCubeIntoScale


class AutonRightStartRightScale(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file
        with open(os.path.join(os.path.dirname(__file__), 'right_start_right_scale', 'path1.pickle'), "rb") as fp:
            path1 = pickle.load(fp)

        # Add commands to run
        self.addSequential(DrivetrainPathFollower(robot, path1, False))
        self.addParallel(ShootCubeIntoScale(robot))
