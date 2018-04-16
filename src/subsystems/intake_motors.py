from wpilib.command import Subsystem
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import INTAKE_RIGHT_MOTOR, INTAKE_LEFT_MOTOR, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class IntakeMotors(Subsystem):
    """
    The intake motors subsystem is used by the operator to control the direction and speed of the
    intake motors.
    """
    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Map the CIM motors to the TalonSRX's
        self.rightTalon = WPI_TalonSRX(INTAKE_RIGHT_MOTOR)
        self.leftTalon = WPI_TalonSRX(INTAKE_LEFT_MOTOR)

    def initControllerSetup(self):
        """
        This method will setup the default settings of the motor controllers.
        """
        # This function will intiliaze the drivetrain motor controllers to the factory defaults.
        # Only values which do not match the factory default will be written.  Any values which
        # are explicity listed will be skipped (ie any values written prior in this method).

        #  ***** TODO *****  #
