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
    The DriveTrain subsytem is used by the driver as well as the Motion Profile controllers.  The default command is DriveJoystick.  Each side of the
    differential drive is connected to CTRE's magnetic encoders.  There is a Pigeon IMU connected to one of the spare Talon's and the data is
    available over the CANbus.
    """
    
    MP_SLOT0_SELECT = 0
    MP_SLOT1_SELECT = 1
    
    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Map the CIM motors to the TalonSRX's
        self.frontLeft = WPI_TalonSRX(DRIVETRAIN_FRONT_LEFT_MOTOR)
        self.leftTalon = WPI_TalonSRX(DRIVETRAIN_REAR_LEFT_MOTOR)
        self.frontRight = WPI_TalonSRX(DRIVETRAIN_FRONT_RIGHT_MOTOR)
        self.rightTalon = WPI_TalonSRX(DRIVETRAIN_REAR_RIGHT_MOTOR)

        # Add the motors to the speed controller groups and create the differential drivetrain
        self.leftDrive = SpeedControllerGroup(self.frontLeft, self.leftTalon)
        self.rightDrive = SpeedControllerGroup(self.frontRight, self.rightTalon)
        self.diffDrive = DifferentialDrive(self.leftDrive, self.rightDrive)

        # Map the pigeon
        self.talonPigeon = WPI_TalonSRX(DRIVETRAIN_PIGEON)
        self.pigeonIMU = PigeonIMU(self.talonPigeon)

        # Setup the default motor controller setup
        self.initControllerSetup()

    def initControllerSetup(self):
        """
        This method will setup the default settings of the motor controllers.
        """
        # Set the front motors to be the followers of the rear motors
        self.frontLeft.set(WPI_TalonSRX.ControlMode.Follower, DRIVETRAIN_REAR_LEFT_MOTOR)
        self.frontRight.set(WPI_TalonSRX.ControlMode.Follower, DRIVETRAIN_REAR_RIGHT_MOTOR)

        # Set the neutral output mode to Brake/Coast/
        self.leftTalon.setNeutralMode(WPI_TalonSRX.NeutralMode.Brake)
        self.rightTalon.setNeutralMode(WPI_TalonSRX.NeutralMode.Brake)

        # Diable the motor-safety
        self.diffDrive.setSafetyEnabled(False)

        # Set the feedback sensor phases
        self.leftTalon.setSensorPhase(True)
        self.rightTalon.setSensorPhase(True)

        # Setup the Pigeon IMU and Talon Mag Encoders
        self.initPigeonIMU()
        self.initQuadratureEncoder()

        # Set the voltage compensation to 12V and disable it for now
        self.leftTalon.configVoltageCompSaturation(12.0, 10)
        self.leftTalon.enableVoltageCompensation(False)
        self.rightTalon.configVoltageCompSaturation(12.0, 10)
        self.rightTalon.enableVoltageCompensation(False)

        # PIDF slot index 0 is for autonomous wheel postion
        self.leftTalon.config_kP(0, 0.8, 10)
        self.leftTalon.config_kI(0, 0.0, 10)
        self.leftTalon.config_kD(0, 0.0, 10)
        self.leftTalon.config_kF(0, 1023 / 12, 10)   # 10-bit ADC units / 12 V
        self.rightTalon.config_kP(0, 0.8, 10)
        self.rightTalon.config_kI(0, 0.0, 10)
        self.rightTalon.config_kD(0, 0.0, 10)
        self.rightTalon.config_kF(0, 1023 / 12, 10)  # 10-bit ADC units / 12 V

        # PIDF slot index 1 is for autonomous heading postion
        self.leftTalon.config_kP(1, 1.0, 10)
        self.leftTalon.config_kI(1, 0, 10)
        self.leftTalon.config_kD(1, 0, 10)
        self.leftTalon.config_kF(1, 0, 10)
        self.rightTalon.config_kP(1, 1.0, 10)
        self.rightTalon.config_kI(1, 0, 10)
        self.rightTalon.config_kD(1, 0, 10)
        self.rightTalon.config_kF(1, 0, 10)

    def pidKludge(self):
        """
        This method is here until we can figure out why some of the profiles have an instaneous output for only a few milliseconds.  The issue has
        been isolated to the encoder position feed-back loop.
        """
        self.leftTalon.config_kP(0, 0.01, 10)
        self.rightTalon.config_kP(0, 0.01, 10)

    def initiaizeDrivetrainMotionProfileControllers(self, stream_rate_ms):
        """
        This method will initialize the Talon's for motion profiling.
        """
        # Invert the right-side motors.  This is done here becuase manual driving commnds account for the motor inversion.
        self.rightTalon.setInverted(True)
        self.frontRight.setInverted(True)

        # Enable voltage compensation for 12V.  This is important since the motion profile feed-forward is in terms of volts.
        self.leftTalon.enableVoltageCompensation(True)
        self.rightTalon.enableVoltageCompensation(True)

        # Change the control frame period
        self.leftTalon.changeMotionControlFramePeriod(stream_rate_ms)
        self.rightTalon.changeMotionControlFramePeriod(stream_rate_ms)

    def cleanUpDrivetrainMotionProfileControllers(self):
        '''
        This mothod will be called to cleanup the Talon's motion profiling
        '''
        # Invert right motors again so the open-loop joystick driving works
        self.rightTalon.setInverted(False)
        self.frontRight.setInverted(False)

        # Change the control frame period back to the default
        self.leftTalon.changeMotionControlFramePeriod(TALON_DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS)
        self.rightTalon.changeMotionControlFramePeriod(TALON_DEFAULT_MOTION_CONTROL_FRAME_PERIOD_MS)

    def initPigeonIMU(self):
        # The default AUX1 polarity behavior is False, PID0 + PID1.  To flip this behaviour, set to True and the result will be PID0 - PID1.
        # For our implementation, both motor controllers are setup as primary, but that doesn't change the behavior of the AUX1 polarity.
        self.rightTalon.configAuxPIDPolarity(False, 10)  # PID0 + PID1
        self.leftTalon.configAuxPIDPolarity(True, 10)    # PID0 - PID1

        # Select a gadgeteer pigeon for remote 0
        self.rightTalon.configRemoteFeedbackFilter(self.talonPigeon.getDeviceID(),          # Device ID of the Talon the Pigeon is connected to
                                                   RemoteSensorSource.GadgeteerPigeon_Yaw,  # Using the ribbon-cable-connected Yaw for feedback
                                                   0,                                       # Remote source, either 0 or 1
                                                   10)                                      # Timeout MS
        self.leftTalon.configRemoteFeedbackFilter(self.talonPigeon.getDeviceID(),           # Device ID of the Talon the Pigeon is connected to
                                                  RemoteSensorSource.GadgeteerPigeon_Yaw,   # Using the ribbon-cable-connected Yaw for feedback
                                                  0,                                        # Remote source, either 0 or 1
                                                  10)                                       # Timeout MS

        # Select the remote feedback sensor for PID1
        self.rightTalon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.RemoteSensor0,  # Select the remote sensor we just created
                                                     1,                                          # PID 0 or 1
                                                     10)                                         # Timeout MS
        self.leftTalon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.RemoteSensor0,   # Select the remote sensor we just created
                                                    1,                                           # PID 0 or 1
                                                    10)                                          # Timeout MS

        # The Pigeon IMU uses a 14-bit DAC to capture the +- yaw data.  A scaling factor of 3600 units/rotation will be applied to the yaw feedback
        # sensor data.  This will leave roughly +-2.25 rotations of yaw data.  When adding a targets for the pigeon heading,
        # multiply 3600 units/rotation by heading rotations.
        self.rightTalon.configSelectedFeedbackCoefficient(3600 / 8192, 1, 10)
        self.leftTalon.configSelectedFeedbackCoefficient(3600 / 8192, 1, 10)

    def zeroGyro(self):
        """
        This method will set the yaw and accumulated z-angle of the gyro to 0.
        """
        self.pigeonIMU.setYaw(0, 10)
        self.pigeonIMU.setAccumZAngle(0, 10)

    def initQuadratureEncoder(self):
        """
        This method will initialize the encoders for quadrature feedback.
        """
        self.leftTalon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)
        self.rightTalon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10)

    def zeroQuadratureEncoder(self):
        self.leftTalon.getSensorCollection().setQuadraturePosition(0, 10)
        self.rightTalon.getSensorCollection().setQuadraturePosition(0, 10)

    def seedQuadratureEncoder(self, seedValue):
        self.leftTalon.getSensorCollection().setQuadraturePosition(seedValue, 10)
        self.rightTalon.getSensorCollection().setQuadraturePosition(seedValue, 10)

    def getLeftQuadraturePosition(self):
        """
        This method will return the left-side sensor quadrature position.  The sign needs to manually be handled here since this function is used to
        provide the sensor postion outide of the talon.
        """
        return -self.leftTalon.getSensorCollection().getQuadraturePosition()

    def getRightQuadraturePosition(self):
        """
        This method will return the right-side sensor quadrature position.  The sign needs to manually be handled here since this function is used to
        provide the sensor postion outide of the talon.
        """
        return self.rightTalon.getSensorCollection().getQuadraturePosition()

    def setQuadratureStatusFramePeriod(self, sample_period_ms):
        """
        This method will set the status frame persiod of the quadrature encoder.
        """
        self.leftTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature, sample_period_ms, 10)
        self.rightTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature, sample_period_ms, 10)

    def setDefaultQuadratureStatusFramePeriod(self):
        """
        This method will set the status frame persiod of the quadrature encoder back to the factory
        default.
        """
        self.leftTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature, TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS, 10)
        self.rightTalon.setStatusFramePeriod(WPI_TalonSRX.StatusFrameEnhanced.Status_3_Quadrature, TALON_DEFAULT_QUADRATURE_STATUS_FRAME_PERIOD_MS, 10)

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
