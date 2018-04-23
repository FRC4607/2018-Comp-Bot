import os
import pickle
from wpilib.command import CommandGroup
from utilities.drivetrain_path_follower import DrivetrainPathFollower
from commands.boom_to_switch import BoomToSwitch
from commands.boom_to_intake import BoomToIntake
from commands.shoot_cube_into_switch import ShootCubeIntoSwitch


class AutonMiddleStartRightSwitch(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__),
                               'middle_start_right_switch.pickle'), "rb") as fp:
            path = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__),
                               'middle_start_right_switch_reverse.pickle'), "rb") as fp:
            reversePath = pickle.load(fp)

        # Add master commands to run
        self.addSequential(StartToSwitch(robot, path))
        self.addSequential(SwitchToStart(robot, reversePath))


class StartToSwitch(CommandGroup):

    def __init__(self, robot, path):
        super().__init__()
        self.requires(robot.driveTrain)

        # Add commands to run
        self.addParallel(DrivetrainPathFollower(robot, path, False))
        self.addParallel(BoomToSwitch(robot))


class SwitchToStart(CommandGroup):

    def __init__(self, robot, path):
        super().__init__()
        self.requires(robot.driveTrain)

        # Add commands to run
        self.addParallel(DrivetrainPathFollower(robot, path, True))
        self.addSequential(ShootCubeIntoSwitch(robot))
        # self.addParallel(BoomToIntake(robot))
