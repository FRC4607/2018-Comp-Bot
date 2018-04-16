from wpilib.command import Command
from utilities.motion_profile_controller import MotionProfileController
from constants import BOOM_STATE, LOGGER_LEVEL
import os
import pickle
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class BoomSwitchToIntake(Command):
    """
    This command will move the boom from the intake position to the switch position.
    """
    def __init__(self, robot):
        super().__init__()
        self.requires(robot.boom)
        self.robot = robot
        self.finished = True

        # Read up the pickled path file.  The intake-to-switch motion profile is symetric, so it
        # should be good for using here.
        with open(os.path.join(os.path.dirname(__file__),
                               'boom_intake_to_switch.pickle'), "rb") as fp:
            self.path = pickle.load(fp)

    def initialize(self):
        self.finished = False

        # Make sure the boom is currently at the switch.  If it isn't, set this command to complete
        # and log it.
        startPotError = abs(self.robot.boom.getPotPositionInDegrees() -
                            self.robot.boom.POT_SWITCH_POSITION_DEG)
        if BOOM_STATE.Switch or abs(startPotError) < 45.0:

            # References to the talon PIDF slot index and the encoder tics per revolution
            slotIndex = self.robot.boom.INTAKE_TO_SWITCH_SLOT_INDEX

            # Create the motion profile controller object
            self.motionProfileController = MotionProfileController(self.robot.boom.talon,
                                                                   self.path,
                                                                   slotIndex,
                                                                   0)

            # The start method will signal the motion profile controller to start
            self.motionProfileController.start()
        else:
            logger.warning("Boom Intake to Switch Command not started - StartPotError: %3.1f, "
                           "Boom State: %s" % (startPotError, self.robot.boomState))
            self.finished = True

    def execute(self):
        if not self.finished:
            if self.motionProfileController.isFinished():
                self.finished = True
            else:
                self.motionProfileController.control()

            # Output debug data to the smartdashboard
            if LOGGER_LEVEL == logging.DEBUG:
                self.robot.smartDashboard.putNumber("BItS_P", self.kP)
                self.robot.smartDashboard.putNumber("BItS_I", self.kI)
                self.robot.smartDashboard.putNumber("BItS_D", self.kD)
                self.robot.smartDashboard.putNumber("BItS_F", self.kF)
                self.robot.smartDashboard.putNumber("BItS_EncPos",
                                                    self.robot.boom.getPotPositionInDegrees())
                self.robot.smartDashboard.putNumber("BItS_ActPos",
                                                    self.robot.boom.getActiveMPPosition())
                self.robot.smartDashboard.putNumber("BItS_EncVel",
                                                    self.robot.boom.getPotVelocityInDegPer100ms())
                self.robot.smartDashboard.putNumber("BItS_ActVel",
                                                    self.robot.boom.getActiveMPVelocity())
                self.robot.smartDashboard.putNumber("BItS_Target",
                                                    self.robot.boom.getPrimaryClosedLoopTarget())
                self.robot.smartDashboard.putNumber("BItS_Error",
                                                    self.robot.boom.getPrimaryClosedLoopError())
                self.robot.smartDashboard.putNumber("TimeStamp",
                                                    self.robot.timer.get())

    def isFinished(self):
        return self.finished

    def end(self):
        self.robot.boomState = BOOM_STATE.Intake
