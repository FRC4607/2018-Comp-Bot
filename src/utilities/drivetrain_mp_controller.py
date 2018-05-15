from wpilib.notifier import Notifier
from ctre._impl.autogen.ctre_sim_enums import SetValueMotionProfile
from ctre._impl.motionprofilestatus import MotionProfileStatus
from ctre.trajectorypoint import TrajectoryPoint
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class DrivetrainMPController():
    """
    This controller is is used to manage the interface when runing Motion Profiles on the TalonSRX
    motor controllers.

    This code is based off of the following java example:
    https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java/MotionProfiles
    """

    NOTIFIER_DEBUG_CNT = 100
    MIN_NUM_POINTS = 5
    NUM_LOOPS_TIMEOUT = 15
    LEFT_DELAY_POINTS = 0

    def __init__(self, left_talon, left_points, right_talon, right_points, reverse, profile_slot_select0, profile_slot_select1):

        # Reference to the motion profile to run
        self._leftPoints = left_points
        self._rightPoints = right_points
        self._profileSlotSelect0 = profile_slot_select0
        self._profileSlotSelect1 = profile_slot_select1
        self.reverse = reverse

        # Reference to the Talon SRX being used
        self._leftTalon = left_talon
        self._rightTalon = right_talon

        # Create objects to receive the status.  This way, the status will be saved in a nice named tuple.
        self._leftStatus = MotionProfileStatus
        self._rightStatus = MotionProfileStatus

        # Control variables
        self._start = False
        self._state = 0
        self._finished = True
        self._loopTimeout = -1
        self._debugCnt = self.NOTIFIER_DEBUG_CNT

        # Create a _notifier to stream trajectory points into the talon.  If the input stream_rate_ms is greater than 40ms, then it would be better
        # to call streamMotionProfileBuffer() in the teleop/autonomous loops since we will stream at twice the rate of the motion profile...assuming
        # all of the motion profile points match the first.
        self._streamRateMS = int(left_points[0][3] / 2)   # MP Duration
        self._notifier = Notifier(self._streamToMotionProfileBuffer)

    def start(self):
        """
        This method is called by a command to begin execution of the motion profile.
        """
        self._initialize()
        self._start = True
        self._state = 0
        self._finished = False
        self._loopTimeout = -1
        self._debugCnt = self.NOTIFIER_DEBUG_CNT

    def isFinished(self):
        """
        This method is called by a command to know when this controller is finished.
        """
        return self._finished

    def control(self, time_stamp, smart_dashboard):
        """
        This method is called by the command every 20ms from autonomous or teleop.
        """
        # Get the current status of the motion profiler controller
        self._leftStatus = self._leftTalon.getMotionProfileStatus()
        self._rightStatus = self._rightTalon.getMotionProfileStatus()

        # In this state, we are waiting for the start signal.  Once received, make sure the Talon MPE is in a state to begin executing a new
        # motion profile.  The talon's should already be in a good start from the _initialize() method.
        if self._state == 0:
            if self._start:
                # Make sure the Talon MPE is disabled before servicing the start signal
                if self._leftStatus.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Left Talon MPE")
                    self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Disable)
                elif self._rightStatus.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Right Talon MPE")
                    self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Disable)

                # Log any prior underrun conitions
                elif self._leftStatus.hasUnderrun:
                    logger.warning("Clearing Left Talon MPE underrun")
                    self._leftTalon.clearMotionProfileHasUnderrun(0)
                elif self._rightStatus.hasUnderrun:
                    logger.warning("Clearing Right Talon MPE underrun")
                    self._rightTalon.clearMotionProfileHasUnderrun(0)

                # Make sure the top and bottom buffers are empty
                elif self._leftStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
                    logger.warning("Clearing Left Talon MPE buffer(s)")
                    self._leftTalon.clearMotionProfileTrajectories()
                elif self._rightStatus.btmBufferCnt != 0 or self._rightStatus.topBufferCnt != 0:
                    logger.warning("Clearing Left Talon MPE buffer(s)")
                    self._rightTalon.clearMotionProfileTrajectories()

                # The Talon MPE status is in a good state, start filling the top buffer and kick off the notifier which moves the top buffer data
                # into the bottom buffer.
                else:
                    logger.info("Starting the Motion Profile Controller")
                    self._start = False
                    self._state = 1
                    self._loopTimeout = self.NUM_LOOPS_TIMEOUT
                    self._startFilling()
                    self._notifier.startPeriodic(self._streamRateMS / 1000)

        # In this state, the Talon MPE has started filling the buffer.  Once enough points have been loaded into the bottom buffer, enable the
        # Talon MPE.
        elif self._state == 1:
            if self._leftStatus.btmBufferCnt > self.MIN_NUM_POINTS and self._rightStatus.btmBufferCnt > self.MIN_NUM_POINTS:
                logger.info("Talon MPE bottom buffer is ready, enabling the Talon MPE")
                self._loopTimeout = self.NUM_LOOPS_TIMEOUT
                self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Enable)
                self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Enable)
                self._state = 2

        # In this state, check status of the MP and if there isn't an underrun condition, reset the loop timeout.  This is basically waiting for the
        # motion profile executer to complete processing the trajectories.
        elif self._state == 2:
            if not self._leftStatus.isUnderrun and not self._rightStatus.isUnderrun:
                self._loopTimeout = self.NUM_LOOPS_TIMEOUT
            else:
                self._outputStatus()

            # If both of the Talon's are at their last trajectory points then stop the notifier, disable the Motion Profile Executer and move on to
            # state 3.
            if self._leftStatus.activePointValid and self._leftStatus.isLast and self._rightStatus.activePointValid and self._rightStatus.isLast:
                logger.info("Talon MPEs are at the last trajectory point")
                self._notifier.stop()
                self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Disable)
                self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Disable)
                self._state = 3

            # Output debug data to the smartdashboard.  This will include all of the data needed to dig into the closed loop motion profile.
            if LOGGER_LEVEL == logging.DEBUG:
                self._outputData(smart_dashboard, time_stamp)

        # In this state, we are ready to exit the motion profile.  Mark the command as complete and remove the notifier.
        elif self._state == 3:
            logger.info("Stopping the Motion Profile Controller")
            self._finished = True

            # Call the free method on the notifier to hopefully stop/destroy it.  Dereference the notifier object...not sure if the garbage collector
            # will do the rest.  This needs Chris's help.
            self._notifier.free()
            del(self._notifier)

        # Service the loop timeout.  If the loop is stalled out, report the motion profile status, set the state to 3, and stop the notifier.  This
        # will hopefully exit gracefully.
        if self._loopTimeout < 0:
            pass
        else:
            if self._loopTimeout == 0:
                logger.warning("No progress being made - State = %i" % (self._state))
                self._outputStatus()
                self._state = 3
                self._notifier.stop()
            else:
                self._loopTimeout -= 1

    def _initialize(self):
        """
        This method will initialize the motion profile controller by clearing out any trajectories still in the buffer, setting the control mode to
        motion profile (disabled), change the control frame period to the stream rate, and clearing any prior buffer underruns.
        """
        self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Disable)
        self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc, SetValueMotionProfile.Disable)
        if self._leftStatus.hasUnderrun:
            logger.warning("Clearing Left Talon UNDERRUN condition during initialization")
            self._leftTalon.clearMotionProfileHasUnderrun(0)
        if self._rightStatus.hasUnderrun:
            logger.warning("Clearing Right Talon UNDERRUN condition during initialization")
            self._rightTalon.clearMotionProfileHasUnderrun(0)
        if self._leftStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
            logger.warning("Clearing Left Talon MPE buffer(s) during initialization")
            self._leftTalon.clearMotionProfileTrajectories()
        if self._rightStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
            logger.warning("Clearing Right Talon MPE buffer(s) during initialization")
            self._rightTalon.clearMotionProfileTrajectories()

    def _streamToMotionProfileBuffer(self):
        """
        This method will move the trajectory points from the top-buffer to the bottom-buffer.
        """
        # Print out a message letting the us know that the notifier is running.
        if self._debugCnt == 0:
            if self._finished:
                logger.warning('Motion Profile Controller notifier is still running')
            self._debugCnt = self.NOTIFIER_DEBUG_CNT
        else:
            self._debugCnt -= 1

        # The bottom buffer size is 128.  Make sure there is space before moving trying to move the data.
        if self._leftStatus.btmBufferCnt < 100:
            self._leftTalon.processMotionProfileBuffer()
        if self._rightStatus.btmBufferCnt < 100:
            self._rightTalon.processMotionProfileBuffer()

    def _startFilling(self):
        """
        This method will start filling the top buffer of the Talon MPE.  This will execute quickly.  If the motion profile is meant to have the robot
        drive backwards, then use negative postion target and negative velocity/FF.  The closed loop postion values will be zero'd out at the
        beginning of each path.  This does not include the zero'ing of the gyro.
        """
        
        for i in range(self.LEFT_DELAY_POINTS):
            point = TrajectoryPoint(0.0,                                                                    # Position
                                    0.0,                                                                    # Velocity / Feed-Forward
                                    self._leftPoints[0][2],                                                 # Heading
                                    self._profileSlotSelect0,                                               # PID0 slot index
                                    self._profileSlotSelect1,                                               # PID0 slot index
                                    False,                                                                  # Last point flag
                                    True if i == 0 else False,                                              # Zero position flag
                                    self._getTrajectoryDuration(self._leftPoints[i][3]))                    # Duration
            self._leftTalon.pushMotionProfileTrajectory(point)                                              # Push the trajectory point (top buffer)
        
        for i in range(len(self._leftPoints)):
            if self.LEFT_DELAY_POINTS != 0:
                point = TrajectoryPoint(-self._leftPoints[i][0] if self.reverse else self._leftPoints[i][0],    # Position
                                        -self._leftPoints[i][1] if self.reverse else self._leftPoints[i][1],    # Velocity / Feed-Forward
                                        self._leftPoints[i][2],                                                 # Heading
                                        self._profileSlotSelect0,                                               # PID0 slot index
                                        self._profileSlotSelect1,                                               # PID0 slot index
                                        True if i+1 == len(self._leftPoints) else False,                        # Last point flag
                                        False,                                                                  # Zero position flag
                                        self._getTrajectoryDuration(self._leftPoints[i][3]))                    # Duration                
            else:
                point = TrajectoryPoint(-self._leftPoints[i][0] if self.reverse else self._leftPoints[i][0],    # Position
                                        -self._leftPoints[i][1] if self.reverse else self._leftPoints[i][1],    # Velocity / Feed-Forward
                                        self._leftPoints[i][2],                                                 # Heading
                                        self._profileSlotSelect0,                                               # PID0 slot index
                                        self._profileSlotSelect1,                                               # PID0 slot index
                                        True if i+1 == len(self._leftPoints) else False,                        # Last point flag
                                        True if i == 0 else False,                                              # Zero position flag
                                        self._getTrajectoryDuration(self._leftPoints[i][3]))                    # Duration
            self._leftTalon.pushMotionProfileTrajectory(point)                                              # Push the trajectory point (top buffer)
            point = TrajectoryPoint(-self._rightPoints[i][0] if self.reverse else self._rightPoints[i][0],
                                    -self._rightPoints[i][1] if self.reverse else self._rightPoints[i][1],
                                    self._leftPoints[i][2],
                                    self._profileSlotSelect0,
                                    self._profileSlotSelect1,
                                    True if i+1 == len(self._rightPoints) else False,
                                    True if i == 0 else False,
                                    self._getTrajectoryDuration(self._rightPoints[i][3]))
            self._rightTalon.pushMotionProfileTrajectory(point)

    def _getTrajectoryDuration(self, duration):
        """
        This method will return the trajectory duration enumeration.
        """
        if duration == 0:
            return TrajectoryPoint.TrajectoryDuration.T0ms
        elif duration == 5:
            return TrajectoryPoint.TrajectoryDuration.T5ms
        elif duration == 10:
            return TrajectoryPoint.TrajectoryDuration.T10ms
        elif duration == 20:
            return TrajectoryPoint.TrajectoryDuration.T20ms
        elif duration == 30:
            return TrajectoryPoint.TrajectoryDuration.T30ms
        elif duration == 40:
            return TrajectoryPoint.TrajectoryDuration.T40ms
        elif duration == 50:
            return TrajectoryPoint.TrajectoryDuration.T50ms
        elif duration == 100:
            return TrajectoryPoint.TrajectoryDuration.T100ms
        else:
            logger.warning("Unsupported duration %i" % (duration))
            return TrajectoryPoint.TrajectoryDuration.T0ms

    def _outputStatus(self):
        logger.warning("LEFT: isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, topBufferCnt: %i, btmBufferCnt: %i, activePointValid: %s, "
                       " isLast: %s, mode: %i, timeDureMS: %i" %
                       (self._leftStatus.isUnderrun, self._leftStatus.hasUnderrun, self._leftStatus.topBufferRem, self._leftStatus.topBufferCnt,
                        self._leftStatus.btmBufferCnt, self._leftStatus.activePointValid, self._leftStatus.isLast, self._leftStatus.outputEnable,
                        self._leftStatus.timeDurMs))
        logger.warning("RIGHT: isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, topBufferCnt: %i, btmBufferCnt: %i, activePointValid: %s, "
                       "isLast: %s, mode: %i, timeDureMS: %i" %
                       (self._rightStatus.isUnderrun, self._rightStatus.hasUnderrun, self._rightStatus.topBufferRem, self._rightStatus.topBufferCnt,
                        self._rightStatus.btmBufferCnt, self._rightStatus.activePointValid, self._rightStatus.isLast, self._rightStatus.outputEnable,
                        self._rightStatus.timeDurMs))

    def _outputData(self, smart_dashboard, time_stamp):
        smart_dashboard.putNumber("RightEncPos", self._rightTalon.getSensorCollection().getQuadraturePosition())
        smart_dashboard.putNumber("RightActPos", self._rightTalon.getActiveTrajectoryPosition())
        smart_dashboard.putNumber("RightEncVel", self._rightTalon.getAnalogInVel())
        smart_dashboard.putNumber("RightActVel", self._rightTalon.getActiveTrajectoryVelocity())
        smart_dashboard.putNumber("RightPrimaryError", self._rightTalon.getClosedLoopError(0))
        smart_dashboard.putNumber("RightSecondaryError", self._rightTalon.getClosedLoopError(1))
        smart_dashboard.putNumber("LeftEncPos", self._leftTalon.getSensorCollection().getQuadraturePosition())
        smart_dashboard.putNumber("LeftActPos", self._leftTalon.getActiveTrajectoryPosition())
        smart_dashboard.putNumber("LeftEncVel", self._leftTalon.getAnalogInVel())
        smart_dashboard.putNumber("LeftActVel", self._leftTalon.getActiveTrajectoryVelocity())
        smart_dashboard.putNumber("LeftPrimaryError", self._leftTalon.getClosedLoopError(0))
        smart_dashboard.putNumber("LeftSecondaryError", self._leftTalon.getClosedLoopError(1))
        smart_dashboard.putNumber("RightTopBufferCount", self._rightStatus.topBufferCnt)
        smart_dashboard.putNumber("LeftTopBufferCount", self._leftStatus.topBufferCnt)
        smart_dashboard.putNumber("LeftBottomBufferCount", self._leftStatus.btmBufferCnt)
        smart_dashboard.putNumber("RightBottomBufferCount", self._rightStatus.btmBufferCnt)
        smart_dashboard.putNumber("TimeStamp", time_stamp)
