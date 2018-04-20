from wpilib import SpeedControllerGroup
from wpilib.command import Subsystem
from wpilib.drive.differentialdrive import DifferentialDrive
from ctre.wpi_talonsrx import WPI_TalonSRX
from ctre.pigeonimu import PigeonIMU
from ctre._impl.autogen.ctre_sim_enums import RemoteSensorSource
from commands.drive_joystick import DriveJoystick
from constants import DRIVETRAIN_FRONT_LEFT_MOTOR, DRIVETRAIN_REAR_LEFT_MOTOR, DRIVETRAIN_PIGEON, \
    DRIVETRAIN_FRONT_RIGHT_MOTOR, DRIVETRAIN_REAR_RIGHT_MOTOR, LOGGER_LEVEL, \
    TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS, TALON_DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS

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

        # Map the pigeon.  This will be connected to an unused Talon.
        self.talonPigeon = WPI_TalonSRX(DRIVETRAIN_PIGEON)
        self.pigeonIMU = PigeonIMU(self.talonPigeon)

    def initControllerSetup(self):
        """
        This method will setup the default settings of the motor controllers.
        """
        # Feedback sensor phase
        self.leftTalon.setSensorPhase(True)
        self.rightTalon.setSensorPhase(True)

        # Diable the motor-safety
        self.diffDrive.setSafetyEnabled(False)

        # Enable brake/coast mode
        self.leftTalon.setNeutralMode(WPI_TalonSRX.NeutralMode.Coast)
        self.rightTalon.setNeutralMode(WPI_TalonSRX.NeutralMode.Coast)

        # This function will intiliaze the drivetrain motor controllers to the factory defaults.
        # Only values which do not match the factory default will be written.  Any values which
        # are explicity listed will be skipped (ie any values written prior in this method).

        #  ***** TODO *****  #

    def initiaizeDrivetrainMotionProfileControllers(self, stream_rate):
        """
        This method will initialize the Talon's for motion profiling
        """
        # Invert right motors
        self.rightTalon.setInverted(True)
        self.frontRight.setInverted(True)

        # Enable voltage compensation for 12V
        self.leftTalon.configVoltageCompSaturation(12.0, 10)
        self.leftTalon.enableVoltageCompensation(True)
        self.rightTalon.configVoltageCompSaturation(12.0, 10)
        self.rightTalon.enableVoltageCompensation(True)

        # PIDF slot index 0 is for autonomous
        # There are 4096 encoder units per rev.  1 rev of the wheel is pi * diameter.  That
        # evaluates to 2607.6 encoder units per foot.  For the feed-forward system, we expect very
        # tight position control, so use a P-gain which drives full throttle at 8" of error.  This
        # evaluates to 0.588 = (1.0 * 1023) / (8 / 12 * 2607.6)
        self.leftTalon.config_kP(0, 0.6, 10)
        self.leftTalon.config_kI(0, 0.0, 10)
        self.leftTalon.config_kD(0, 0.0, 10)
        self.leftTalon.config_kF(0, 1023 / 12, 10)   # 10-bit ADC / 12 V
        self.rightTalon.config_kP(0, 0.6, 10)
        self.rightTalon.config_kI(0, 0.0, 10)
        self.rightTalon.config_kD(0, 0.0, 10)
        self.rightTalon.config_kF(0, 1023 / 12, 10)  # 10-bit ADC / 12 V

        # Change the control frame period
        self.leftTalon.changeMotionControlFramePeriod(stream_rate)
        self.rightTalon.changeMotionControlFramePeriod(stream_rate)

        # Initilaize the quadrature encoders and pigeon IMU
        self.initQuadratureEncoder()
        self.initPigeonIMU()

    def cleanUpDrivetrainMotionProfileControllers(self):
        '''
        This mothod will be called to cleanup the Talon's motion profiling
        '''
        # Invert right motors again so the open-loop joystick driving works
        self.rightTalon.setInverted(False)
        self.frontRight.setInverted(False)

        # Change the control frame period back to the default
        framePeriod = TALON_DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS
        self.leftTalon.changeMotionControlFramePeriod(framePeriod)
        self.rightTalon.changeMotionControlFramePeriod(framePeriod)

    def initPigeonIMU(self):
        # false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
        # true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
        self.rightTalon.configAuxPIDPolarity(False, 10)
        self.leftTalon.configAuxPIDPolarity(True, 10)

        # select a gadgeteer pigeon for remote 0
        self.rightTalon.configRemoteFeedbackFilter(self.talonPigeon.getDeviceID(),
                                                   RemoteSensorSource.GadgeteerPigeon_Yaw,
                                                   0, 10)
        self.leftTalon.configRemoteFeedbackFilter(self.talonPigeon.getDeviceID(),
                                                  RemoteSensorSource.GadgeteerPigeon_Yaw,
                                                  0, 10)

        # Select the remote feedback sensor for PID1
        self.rightTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)
        self.leftTalon.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 1, 10)

        # Using the config feature, scale units to 3600 per rotation.  This is nice as it keeps
        # 0.1 deg resolution, and is fairly intuitive.
        self.rightTalon.configSelectedFeedbackCoefficient(3600 / 8192, 1, 10)
        self.leftTalon.configSelectedFeedbackCoefficient(3600 / 8192, 1, 10)

        # Zero the sensor
        self.pigeonIMU.setYaw(0, 10)
        self.pigeonIMU.setAccumZAngle(0, 10)

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
