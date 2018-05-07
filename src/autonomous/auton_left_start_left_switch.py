import os
import pickle
from wpilib.command import CommandGroup
from commands.shoot_cube_into_switch import ShootCubeIntoSwitch
from commands.boom_to_switch import BoomToSwitch
from utilities.drivetrain_path_follower import DrivetrainPathFollower


class AutonLeftStartLeftSwitch(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__), 'left_start_left_switch', 'path1.pickle'), "rb") as fp:
            path1 = pickle.load(fp)

        # Add commands to run
        self.addParallel(BoomToSwitch(robot))
        self.addSequential(DrivetrainPathFollower(robot, path1, False))
        self.addSequential(ShootCubeIntoSwitch(robot))
