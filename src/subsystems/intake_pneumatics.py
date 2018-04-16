from wpilib.command import Subsystem
from wpilib.doublesolenoid import DoubleSolenoid
from constants import INTAKE_SOLENOID, INTAKE_FORWARD_SOLENOID, INTAKE_REVERSE_SOLENOID, \
    LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class IntakePneumatics(Subsystem):
    """
    The intake pneumatics subsystem is used by the operator to control the intake opening and
    closing.
    """
    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Map the solenoids
        self.solenoid = DoubleSolenoid(INTAKE_SOLENOID, INTAKE_FORWARD_SOLENOID,
                                       INTAKE_REVERSE_SOLENOID)

        # Set the initial state of the solenoid
        # self.solenoid.set(DoubleSolenoid.Value.kForward)
        self.solenoid.set(DoubleSolenoid.Value.kReverse)
