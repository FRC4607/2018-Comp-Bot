import os
import pickle
from wpilib.command import CommandGroup
from commands.shoot_cube_into_switch import ShootCubeIntoSwitch
from commands.boom_to_switch import BoomToSwitch
from utilities.drivetrain_path_follower import DrivetrainPathFollower


class AutonRightStartRightSwitch(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__), 'right_start_right_switch.pickle'), "rb") as fp:
            path = pickle.load(fp)

        # Add commands to run
        self.addSequential(DrivetrainPathFollower(robot, path, False))
        self.addSequential(DrivetrainPathFollower(robot, path, False))
        self.addSequential(ShootCubeIntoSwitch(robot))
