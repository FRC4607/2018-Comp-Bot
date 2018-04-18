from constants import TALON_CMD_TO_MS
from ctre import FeedbackDevice

class TalonReset():
    
    def __init__(self):
        pass
    
    def resetValues(self, talon):
        encoderType = FeedbackDevice.CTRE_MagEncoder_Relative
        self._talon = talon
        
        self._talon.configOpenloopRamp(0, TALON_CMD_TO_MS)
        self._talon.configClosedloopRamp(0, TALON_CMD_TO_MS)
        self._talon.configPeakOutputForward(1, TALON_CMD_TO_MS)
        self._talon.configPeakOutputReverse(-1, TALON_CMD_TO_MS)
        self._talon.configNominalOutputForward(0, TALON_CMD_TO_MS)
        self._talon.configNominalOutputReverse(0, TALON_CMD_TO_MS)
        self._talon.configNeutralDeadband(0.04, TALON_CMD_TO_MS)
        self._talon.configVoltageCompSaturation(0, TALON_CMD_TO_MS)    
        self._talon.configVoltageMeasurementFilter(32, TALON_CMD_TO_MS)    
        self._talon.configSelectedFeedbackSensor(encoderType, 0, TALON_CMD_TO_MS)
        #===========================================================================================
        # self._talon.configSelectedFeedbackCoefficient()
        # self._talon.configRemoteFeedbackFilter()
        #===========================================================================================
        self._talon.configSensorTerm(0, 0, TALON_CMD_TO_MS)
        self._talon.configSensorTerm(1, 0, TALON_CMD_TO_MS)
        self._talon.configSensorTerm(2, 0, TALON_CMD_TO_MS)
        self._talon.configSensorTerm(3, 0, TALON_CMD_TO_MS)
        self._talon.configVelocityMeasurementPeriod(100, TALON_CMD_TO_MS)
        self._talon.configVelocityMeasurementWindow(64, TALON_CMD_TO_MS)
        self._talon.configForwardLimitSwitchSource(0, 0, TALON_CMD_TO_MS)
        self._talon.configReverseLimitSwitchSource(0, 0, TALON_CMD_TO_MS)
        self._talon.configForwardSoftLimitThreshold(0, TALON_CMD_TO_MS)
        self._talon.configReverseSoftLimitThreshold(0, TALON_CMD_TO_MS)
        self._talon.configForwardSoftLimitEnable(False, TALON_CMD_TO_MS)
        self._talon.configReverseSoftLimitEnable(False, TALON_CMD_TO_MS)
        self._talon.config_kP(0, 0, TALON_CMD_TO_MS)
        self._talon.config_kI(0, 0, TALON_CMD_TO_MS)
        self._talon.config_kD(0, 0, TALON_CMD_TO_MS)
        self._talon.config_kF(0, 0, TALON_CMD_TO_MS)
        self._talon.config_IntegralZone(0, 0, TALON_CMD_TO_MS)
        self._talon.configAllowableClosedloopError(0, 0, TALON_CMD_TO_MS)
        self._talon.configMaxIntegralAccumulator(0, 0, TALON_CMD_TO_MS)
        #===========================================================================================
        # self._talon.configClosedLoopPeakOutput
        # self._talon.configClosedLoopPeriod   
        # self._talon.configAuxPIDPolarity    
        #===========================================================================================
        self._talon.configMotionCruiseVelocity(0, TALON_CMD_TO_MS)    
        self._talon.configMotionAcceleration(0, TALON_CMD_TO_MS)    
        self._talon.configMotionProfileTrajectoryPeriod(0, TALON_CMD_TO_MS)  
        self._talon.configSetCustomParam(0, 0, TALON_CMD_TO_MS)
        self._talon.configPeakCurrentLimit(0, TALON_CMD_TO_MS)
        self._talon.configPeakCurrentDuration(0, TALON_CMD_TO_MS)
        self._talon.configContinuousCurrentLimit(0, TALON_CMD_TO_MS) 