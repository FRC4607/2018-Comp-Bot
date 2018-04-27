from wpilib.command import Subsystem
from ctre.wpi_talonsrx import WPI_TalonSRX
from ctre._impl.autogen.ctre_sim_enums import LimitSwitchSource, LimitSwitchNormal
from commands.boom_joystick import BoomJoystick
from constants import BOOM_MOTOR, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class Boom(Subsystem):
    """
    The boom subsystem is used by the operator and also the boom controller.  There is a pot
    mounted to the linear actuator which will provide feeback for the boom controller.
    """

    ENCODER_TICS_PER_REV = 102.4  # 10-turn analog pot on 10-bit ADC: 1024 / 10
    POT_INTAKE_POSITION_DEG = 83 * (3600 / 1023)
    POT_SWITCH_POSITION_DEG = 348 * (3600 / 1023)  # ** TODO **
    POT_SCALE_POSITION_DEG = 825 * (3600 / 1023)  # ** TODO **
    POT_SCALE_POSITION = 825
    POT_SCALE_UPPER_ERROR = 1005
    POT_SCALE_LOWER_ERROR = 645
    POT_ERROR_LIMIT = 360.0
    # FORWARD_SOFT_LIMIT = 840
    # REVERSE_SOFT_LIMIT = 90

    def __init__(self, robot):
        super().__init__()
        self.robot = robot

        # Map the CIM motors to the TalonSRX's
        self.talon = WPI_TalonSRX(BOOM_MOTOR)

        # Setup the default motor controller setup
        self.initOpenLoop()
        self.initClosedLoop()

    def initOpenLoop(self):
        """
        This method will setup the default settings of the motor controllers.
        """
        # Set to percent output mode at neutral
        self.talon.set(WPI_TalonSRX.ControlMode.PercentOutput, 0.0)

        # Add a ramp-rate limiter to joystick control
        self.talon.configOpenLoopRamp(0.2, 10)

        # Add current limiter
        # self.talon.configPeakCurrentLimit(BOOM_MAX_CURRENT, 10)
        # self.talon.configPeakCurrentDuration(0, 10)  # Necessary to avoid errata
        # self.talon.configContinuousCurrentLimit(BOOM_MAX_CURRENT, 10)
        # self.talon.enableCurrentLimit(True)

        # Add soft limits
        # self.talon.configForwardSoftLimitThreshold(self.FORWARD_SOFT_LIMIT, 0)
        # self.talon.configForwardSoftLimitEnable(True, 0)
        # self.talon.configReverseSoftLimitThreshold(self.REVERSE_SOFT_LIMIT, 0)
        # self.talon.configReverseSoftLimitEnable(True, 0)

        # Configured forward and reverse limit switch of Talon to be from a feedback connector and
        # be normally open
        self.talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                                  LimitSwitchNormal.NormallyOpen, 0, 10)
        self.talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                                                  LimitSwitchNormal.NormallyOpen, 0, 10)

        # Set brake/coast mode
        self.talon.setNeutralMode(WPI_TalonSRX.NeutralMode.Brake)


    def initClosedLoop(self):
        # PIDF slot index 0 is for single-position moves
        self.talon.config_kP(0, 5.0, 10)
        self.talon.config_kI(0, 0.0, 10)
        self.talon.config_kD(0, 0.0, 10)
        self.talon.config_kF(0, 20, 10)

        # PIDF slot index 1 is for multi-position moves
        self.talon.config_kP(1, 1.0, 10)
        self.talon.config_kI(1, 0.0, 10)
        self.talon.config_kD(1, 0.0, 10)
        self.talon.config_kF(1, 22, 10)

        # Use analog potentiometer
        self.initAnalogPotentiometer()

    def initAnalogPotentiometer(self):
        """
        This method will initialize the analog potentiometer for position feedback.
        """
        self.talon.configSelectedFeedbackSensor(WPI_TalonSRX.FeedbackDevice.Analog, 0, 10)

    def getPotPositionInDegrees(self):
        """
        This method will return the potentiometer position.
        """
        # Lead: 0.20", Potentiometer for a 6.85:1 ratio, 12 / 0.2 = 60 => 60 / 6.85 = 8.76
        # The potentiometer will turn 8.76 times during a 12-inch travel
        # 0V - 5V maps to 0 - 1023
        degreesPerADC = 3600 / 1024
        degrees = self.talon.getSensorCollection().getAnalogInRaw() * degreesPerADC
        # logger.debug('Pot Degrees: %1.2f' % (degrees))
        return degrees

    def getPotPosition(self):
        """
        This method will return the potentiometer position.
        """
        return self.talon.getSensorCollection().getAnalogInRaw()

    def getPotVelocityInDegPer100ms(self):
        """
        This method will return the potentiometer velocity in deg / 100ms
        """
        return self.talon.getSensorCollection().getAnalogInVel()

    def getActiveMPPosition(self):
        """
        This method will return the active motion profile position
        """
        return self.talon.getActiveTrajectoryPosition()

    def getActiveMPVelocity(self):
        """
        This method will return the active motion profile position
        """
        return self.talon.getActiveTrajectoryVelocity()

    def getPrimaryClosedLoopTarget(self):
        """
        This method will return the closed loop target.
        """
        return self.talon.getClosedLoopTarget(0)

    def getPrimaryClosedLoopError(self):
        """
        This method will return the closed loop error for the primary conrol loop.
        """
        return self.talon.getClosedLoopError(0)

    def initDefaultCommand(self):
        """
        This method will set the default command for this subsystem.
        """
        self.setDefaultCommand(BoomJoystick(self.robot))
