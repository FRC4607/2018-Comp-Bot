from wpilib.command import Command
from utilities.motion_profile_controller import MotionProfileController
from constants import BOOM_STATE, LOGGER_LEVEL
import os
import pickle
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class BoomToSwitch(Command):
    """
    This command will move the boom to the intake position.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.boom)
        self.robot = robot
        self.finished = True

        # Read up the pickled path file
        with open(os.path.join(os.path.dirname(__file__),
                               'boom_intake_to_switch.pickle'), "rb") as fp:
            self.intakeToSwitchPath = pickle.load(fp)
        with open(os.path.join(os.path.dirname(__file__),
                               'boom_switch_to_scale.pickle'), "rb") as fp:
            self.switchToScalePath = pickle.load(fp)

    def initialize(self):
        self.finished = False

        # The boom is currently at the intake
        if self.robot.boomState == BOOM_STATE.Intake:

            # Double-check the pot to ensure the boom is at the switch
            startPotError = abs(self.robot.boom.getPotPositionInDegrees() -
                                self.robot.boom.POT_INTAKE_POSITION_DEG)
            if startPotError < self.robot.boom.POT_ERROR_LIMIT:

            # Create the motion profile controller object
                startingPostion = self.robot.boom.getPotPosition()
                self.motionProfileController = MotionProfileController(self.robot.boom.talon,
                                                                       self.intakeToSwitchPath,
                                                                       False,
                                                                       0,
                                                                       0)
                # The start method will signal the motion profile controller to start
                self.motionProfileController.start()
    
            else:
                logger.warning("Boom to Switch Command not started - StartPotError: %3.1f" %
                               (startPotError))
                self.finished = True

        # The boom is currently at the scale
        elif self.robot.boomState == BOOM_STATE.Scale:

            # Double-check the pot to ensure the boom is at the switch
            startPotError = abs(self.robot.boom.getPotPositionInDegrees() -
                                self.robot.boom.POT_SCALE_POSITION_DEG)
            if startPotError < self.robot.boom.POT_ERROR_LIMIT:

                # Create the motion profile controller object
                startingPostion = self.robot.boom.getPotPosition()
                self.motionProfileController = MotionProfileController(self.robot.boom.talon,
                                                                       self.switchToScalePath,
                                                                       True,
                                                                       0,
                                                                       0)
                # The start method will signal the motion profile controller to start
                self.motionProfileController.start()

            else:
                logger.warning("Boom to Switch Command not started - StartPotError: %3.1f" %
                               (startPotError))
                self.finished = True

        # The boom will be at unknown position
        else:
            logger.warning("Boom to Switch Command not started - BoomState: %s" %
                           (self.robot.boomState))
            self.finished = True

    def execute(self):
        if not self.finished:
            if self.motionProfileController.isFinished():
                self.finished = True
            else:
                self.motionProfileController.control()

            # Output debug data to the smartdashboard
            if LOGGER_LEVEL == logging.DEBUG:
                self.robot.smartDashboard.putNumber("EncPos",
                                                    self.robot.boom.talon.getSensorCollection().getAnalogInRaw())
                self.robot.smartDashboard.putNumber("ActPos",
                                                    self.robot.boom.talon.getActiveTrajectoryPosition())
                self.robot.smartDashboard.putNumber("EncVel",
                                                    self.robot.boom.talon.getAnalogInVel())
                self.robot.smartDashboard.putNumber("ActVel",
                                                    self.robot.boom.talon.getActiveTrajectoryVelocity())
                self.robot.smartDashboard.putNumber("PrimaryTarget",
                                                    self.robot.boom.talon.getClosedLoopTarget(0))
                self.robot.smartDashboard.putNumber("PrimaryError",
                                                    self.robot.boom.talon.getClosedLoopError(0))
                self.robot.smartDashboard.putNumber("TimeStamp", self.robot.timer.get())

    def isFinished(self):
        return self.finished

    def end(self):
        endPotError = (self.robot.boom.getPotPositionInDegrees() -
                       self.robot.boom.POT_SWITCH_POSITION_DEG)
        if abs(endPotError) < self.robot.boom.POT_ERROR_LIMIT:
            self.robot.boomState = BOOM_STATE.Switch
        else:
            self.robot.boomState = BOOM_STATE.Unknown
            logger.warning("Boom to Switch Command finish poorly - endPotError: %3.1f" %
                           (endPotError))
        self.robot.boom.initOpenLoop()
