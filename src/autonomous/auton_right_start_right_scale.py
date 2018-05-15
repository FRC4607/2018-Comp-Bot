import os
import pickle
from wpilib.command import CommandGroup
from utilities.drivetrain_path_follower import DrivetrainPathFollower
from commands.shoot_cube_into_scale import ShootCubeIntoScale
from commands.boom_to_scale import BoomToScale
from commands.boom_to_intake import BoomToIntake
from commands.open_intake import OpenIntake
from commands.close_intake import CloseIntake
from commands.start_intake import StartIntake


class AutonRightStartRightScale(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        # Read up the pickled path file of trajectories
        with open(os.path.join(os.path.dirname(__file__), 'right_start_right_scale', 'path1.pickle'), "rb") as fp:
            path1 = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__), 'right_start_right_scale', 'path2.pickle'), "rb") as fp:
            path2 = pickle.load(fp)        
        with open(os.path.join(os.path.dirname(__file__), 'right_start_right_scale', 'path3.pickle'), "rb") as fp:
            path3 = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__), 'right_start_right_scale', 'path4.pickle'), "rb") as fp:
            path4 = pickle.load(fp)
        #=======================================================================
        # with open(os.path.join(os.path.dirname(__file__), 'right_start_right_scale', 'path5.pickle'), "rb") as fp:
        #     path5 = pickle.load(fp)            
        #=======================================================================
              
        # Zero gyro and encoders
        robot.driveTrain.zeroGyro()
        robot.driveTrain.zeroQuadratureEncoder()

        # Go to scale
        self.addSequential(DrivetrainPathFollower(robot, path1, False))
        self.addParallel(ShootCubeIntoScale(robot))

        # Zero encoders
        robot.driveTrain.zeroQuadratureEncoder()

        # Go to cube retrieval position
        self.addParallel(BoomToScale(robot))
        self.addSequential(DrivetrainPathFollower(robot, path2, True))
 
        # Zero encoders
        robot.driveTrain.zeroQuadratureEncoder()
 
        # Go to pick up the next cube
        self.addSequential(DrivetrainPathFollower(robot, path3, False))
        self.addParallel(StartIntake(robot))
        self.addParallel(CloseIntake(robot))
 
        # Zero encoders
        robot.driveTrain.zeroQuadratureEncoder()
 
        # Go to scale / ready position
        self.addParallel(BoomToScale(robot))
        self.addSequential(DrivetrainPathFollower(robot, path4, True))
        self.addSequential(ShootCubeIntoScale(robot))
   
        #=======================================================================
        # # Go to scale
        # self.addSequential(DrivetrainPathFollower(robot, path5, False))
        # self.addSequential(ShootCubeIntoScaleS(robot))
        #=======================================================================
