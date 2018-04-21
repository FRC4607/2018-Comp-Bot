from wpilib.command import CommandGroup
from autonomous.left_start_left_scale_path_follower import LeftStartLeftScalePathFollower


class AutonMiddleStartLeftScale(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        self.addSequential(LeftStartLeftScalePathFollower(robot))
