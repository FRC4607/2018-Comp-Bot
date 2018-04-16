from wpilib.command import CommandGroup
from autonomous.forward_path_follower import ForwardPathFollower


class AutonForward(CommandGroup):

    def __init__(self, robot):
        super().__init__()
        self.requires(robot.driveTrain)

        self.addSequential(ForwardPathFollower(robot))
