import os
import pickle
from wpilib.command import CommandGroup
from utilities.drivetrain_path_follower import DrivetrainPathFollower
from commands.boom_to_switch import BoomToSwitch
from commands.boom_to_intake import BoomToIntake
from commands.shoot_cube_into_switch import ShootCubeIntoSwitch
from commands.open_intake import OpenIntake
from commands.close_intake import CloseIntake
from commands.start_intake import StartIntake


class AutonMiddleStartRightSwitch(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__), 'middle_start_right_switch', 'path1.pickle'), "rb") as fp:
            path1 = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__), 'middle_start_right_switch', 'path2.pickle'), "rb") as fp:
            path2 = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__), 'middle_start_right_switch', 'path3.pickle'), "rb") as fp:
            path3 = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__), 'middle_start_right_switch', 'path4.pickle'), "rb") as fp:
            path4 = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__), 'middle_start_right_switch', 'path5.pickle'), "rb") as fp:
            path5 = pickle.load(fp)

        # Zero gyro and encoders
        robot.driveTrain.zeroGyro()
        robot.driveTrain.zeroQuadratureEncoder()

        # Go to switch
        self.addParallel(BoomToSwitch(robot))
        self.addSequential(DrivetrainPathFollower(robot, path1, False))
        self.addParallel(ShootCubeIntoSwitch(robot))

        # Go to cube retrieval position
        self.addParallel(BoomToIntake(robot))
        self.addParallel(OpenIntake(robot))
        self.addSequential(DrivetrainPathFollower(robot, path2, True))

        # Go to pick up the next cube
        self.addSequential(DrivetrainPathFollower(robot, path3, False, True))
        self.addParallel(StartIntake(robot))
        self.addParallel(CloseIntake(robot))

        # Go back to start
        self.addParallel(BoomToSwitch(robot))
        self.addSequential(DrivetrainPathFollower(robot, path4, True))

        # Go to switch
        self.addSequential(DrivetrainPathFollower(robot, path5, False, True))
        self.addSequential(ShootCubeIntoSwitch(robot))

