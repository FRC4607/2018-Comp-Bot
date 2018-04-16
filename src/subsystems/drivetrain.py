from wpilib import SpeedControllerGroup
from wpilib.command import Subsystem
from wpilib.drive.differentialdrive import DifferentialDrive
from ctre.wpi_talonsrx import WPI_TalonSRX
from robotpy_ext.common_drivers.navx import AHRS
from commands.drive_joystick import DriveJoystick
from constants import DRIVETRAIN_FRONT_LEFT_MOTOR, DRIVETRAIN_REAR_LEFT_MOTOR, \
    DRIVETRAIN_FRONT_RIGHT_MOTOR, DRIVETRAIN_REAR_RIGHT_MOTOR, \
    TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class DriveTrain(Subsystem):
    """
    The DriveTrain subsytem is used by the driver as well as the Pathfinder and Motion Profile
    controllers.  The default command is DriveJoystick.  Each side of the differential drive is
    connected to CTRE's magnetic encoders.
    """

    ENCODER_TICKS_PER_REV = 4096
    MP_SLOT0_SELECT = 0
    MP_SLOT1_SELECT = 0

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Map the CIM motors to the TalonSRX's
        self.frontLeft = WPI_TalonSRX(DRIVETRAIN_FRONT_LEFT_MOTOR)
        self.leftTalon = WPI_TalonSRX(DRIVETRAIN_REAR_LEFT_MOTOR)
        self.frontRight = WPI_TalonSRX(DRIVETRAIN_FRONT_RIGHT_MOTOR)
        self.rightTalon = WPI_TalonSRX(DRIVETRAIN_REAR_RIGHT_MOTOR)

        # Set the front motors to be the followers of the rear motors
        self.frontLeft.set(WPI_TalonSRX.ControlMode.Follower, DRIVETRAIN_REAR_LEFT_MOTOR)
        self.frontRight.set(WPI_TalonSRX.ControlMode.Follower, DRIVETRAIN_REAR_RIGHT_MOTOR)

        # Add the motors to the speed controller groups and create the differential drivetrain
        self.leftDrive = SpeedControllerGroup(self.frontLeft, self.leftTalon)
        self.rightDrive = SpeedControllerGroup(self.frontRight, self.rightTalon)
        self.diffDrive = DifferentialDrive(self.leftDrive, self.rightDrive)

        # Setup the default motor controller setup
        self.initControllerSetup()

        # Create the NavX gyro instance
        self.navX = AHRS.create_spi()

    def initControllerSetup(self):
        """
        This method will setup the default settings of the motor controllers.
        """
        # Feedback sensor phase
        self.leftTalon.setSensorPhase(True)
        self.rightTalon.setSensorPhase(True)

        # Diable the motor-safety
        self.diffDrive.setSafetyEnabled(False)

        # This function will intiliaze the drivetrain motor controllers to the factory defaults.
        # Only values which do not match the factory default will be written.  Any values which
        # are explicity listed will be skipped (ie any values written prior in this method).

        #  ***** TODO *****  #

    def initiaizeDrivetrainMotionProfileControllers(self):
        """
        This method will initialize the Talon's for motion profiling
        """
        # Enable voltage compensation for 12V
        self.leftTalon.configVoltageCompSaturation(12.0, 10)
        self.leftTalon.enableVoltageCompensation(True)
        self.rightTalon.configVoltageCompSaturation(12.0, 10)
        self.rightTalon.enableVoltageCompensation(True)

        # Enable brake mode
        self.leftTalon.setNeutralMode(WPI_TalonSRX.NeutralMode.Brake)
        self.rightTalon.setNeutralMode(WPI_TalonSRX.NeutralMode.Brake)

    def initQuadratureEncoder(self):
        """
        This method will initialize the encoders for quadrature feedback.
        """
        self.leftTalon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative,
                                                    0, 10)
        self.rightTalon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative,
                                                     0, 10)
        self.leftTalon.getSensorCollection().setQuadraturePosition(0, 10)
        self.rightTalon.getSensorCollection().setQuadraturePosition(0, 10)

    def getLeftQuadraturePosition(self):
        """
        This method will return the left-side sensor quadrature position.  The sign needs to
        manually be handled here since this function is used to provide the sensor postion outide
        of the talon.
        """
        return -self.leftTalon.getSensorCollection().getQuadraturePosition()

    def getRightQuadraturePosition(self):
        """
        This method will return the right-side sensor quadrature position.  The sign needs to
        manually be handled here since this function is used to provide the sensor postion outide
        of the talon.
        """
        return self.rightTalon.getSensorCollection().getQuadraturePosition()

    def setQuadratureStatusFramePeriod(self, sample_period_ms):
        """
        This method will set the status frame persiod of the quadrature encoder
        """
        self.leftTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature,
                                            sample_period_ms, 10)
        self.rightTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature,
                                             sample_period_ms, 10)

    def setDefaultQuadratureStatusFramePeriod(self):
        """
        This method will set the status frame persiod of the quadrature encoder back to the factory
        default.
        """
        self.leftTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature,
                                            TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS, 10)
        self.rightTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature,
                                             TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS, 10)

    def pathFinderDrive(self, leftOutput, rightOutput):
        """
        This method will take the Pathfinder Controller motor output and apply them to the
        drivetrain.
        """
        self.leftTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, leftOutput)
        self.rightTalon.set(WPI_TalonSRX.ControlMode.PercentOutput, -rightOutput)

    def getLeftVelocity(self):
        return self.leftTalon.getSensorCollection().getQuadratureVelocity()

    def getRightVelocity(self):
        return self.rightTalon.getSensorCollection().getQuadratureVelocity()

    def getLeftVoltage(self):
        return self.leftTalon.getMotorOutputVoltage()

    def getRightVoltage(self):
        return self.rightTalon.getMotorOutputVoltage()

    def initDefaultCommand(self):
        """
        This method will set the default command for this subsystem.
        """
        self.setDefaultCommand(DriveJoystick(self.robot))
