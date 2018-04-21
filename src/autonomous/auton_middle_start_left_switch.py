from wpilib.command import CommandGroup
from autonomous.middle_start_left_switch_path_follower import MiddleStartLeftSwitchPathFollower


class AutonMiddleStartLeftSwitch(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        self.addSequential(MiddleStartLeftSwitchPathFollower(robot))
