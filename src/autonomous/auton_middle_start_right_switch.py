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
        with open(os.path.join(os.path.dirname(__file__),
                               'middle_start_right_switch.pickle'), "rb") as fp:
            path = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__),
                               'right_switch_cube_retrieval.pickle'), "rb") as fp:
            cubePosPath = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__),
                               'right_cube_retrieval_cube_get.pickle'), "rb") as fp:
            cubeGetPath = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__),
                               'right_cube_get_switch_prep.pickle'), "rb") as fp:
            cubeSwitchPrepPath = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__),
                               'switch_prep_right_switch.pickle'), "rb") as fp:
            cubeSwitchPath = pickle.load(fp)

        # Zero gyro and encoders
        robot.driveTrain.zeroGyro()
        robot.driveTrain.zeroQuadratureEncoder()

        # Go to switch
        self.addParallel(BoomToSwitch(robot))
        self.addSequential(DrivetrainPathFollower(robot, path, False))
        self.addParallel(ShootCubeIntoSwitch(robot))

        # Go to cube retrevial position
        self.addParallel(OpenIntake(robot))
        self.addParallel(BoomToIntake(robot))
        self.addSequential(DrivetrainPathFollower(robot, cubePosPath, True))

        # Go to pick up the next cube
        self.addSequential(DrivetrainPathFollower(robot, cubeGetPath, False, True))
        self.addParallel(StartIntake(robot))
        self.addParallel(CloseIntake(robot))

        # Go back to start
        self.addParallel(BoomToSwitch(robot))
        self.addSequential(DrivetrainPathFollower(robot, cubeSwitchPrepPath, True))

        # Go to switch
        # self.addSequential(DrivetrainPathFollower(robot, cubeSwitchPath, False, True))
        # self.addSequential(ShootCubeIntoSwitch(robot))
