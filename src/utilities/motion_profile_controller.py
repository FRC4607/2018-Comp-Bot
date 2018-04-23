from wpilib.notifier import Notifier
from ctre._impl.autogen.ctre_sim_enums import SetValueMotionProfile
from ctre._impl.motionprofilestatus import MotionProfileStatus
from ctre.trajectorypoint import TrajectoryPoint
from ctre.wpi_talonsrx import WPI_TalonSRX
from constants import LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class MotionProfileController():
    """
    This controller is is used to manage the interface when runing Motion Profiles on the TalonSRX
    motor controllers.

    This code is based off of the following java example:
    https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/tree/master/Java/MotionProfiles
    """

    NOTIFIER_DEBUG_CNT = 100
    MIN_NUM_POINTS = 5
    NUM_LOOPS_TIMEOUT = 10

    def __init__(self, talon, points, reverse, profile_slot_select0, profile_slot_select1):

        # Reference to the motion profile to run
        self._points = points
        self._profileSlotSelect0 = profile_slot_select0
        self._profileSlotSelect1 = profile_slot_select1
        self.reverse = reverse

        # Reference to the Talon SRX being used
        self._talon = talon

        # Control variables
        self._start = False
        self._state = 0
        self._finished = True
        self._loopTimeout = -1
        self._debugCnt = self.NOTIFIER_DEBUG_CNT
        self._status = MotionProfileStatus

        # Create a _notifier to stream trajectory points into the talon.  If the input
        # stream_rate_ms is greater than 40ms, then it would be better to call
        # streamMotionProfileBuffer() in the teleop/autonomous loops since we will stream at twice
        # the rate of the motion profile
        self._streamRateMS = int(points[0][3] / 2)
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
        This method is called by a command to know when the path follower is finished.
        """
        return self._finished

    def control(self):
        """
        This method is called by the command every 20ms in autonomous or teleop.
        """
        # Get the current _status
        self._status = self._talon.getMotionProfileStatus()

        # Waiting for the start signal.  Once received, make sure the Talon MPE is in a state to
        # begin executing a new motion profile.
        if self._state == 0:
            if self._start:
                # Make sure the Talon MPE is disabled before servicing the start signal
                if self._status.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Talon MPE")
                    self._talon.set(WPI_TalonSRX.ControlMode.MotionProfile,
                                    SetValueMotionProfile.Disable)

                # Log any prior underrun conitions
                elif self._status.hasUnderrun:
                    logger.warning("Clearing Talon MPE underrun")
                    self._talon.clearMotionProfileHasUnderrun(0)

                # Make sure the top and bottom buffers are empty
                elif self._status.btmBufferCnt != 0 or self._status.topBufferCnt != 0:
                    logger.warning("Clearing Talon MPE buffer(s)")
                    self._talon.clearMotionProfileTrajectories()

                # The Talon MPE status is in a good state, start filling the top buffer and kick
                # off the notifier which moves the top buffer data into the bottom buffer.
                else:
                    logger.info("Starting the Motion Profile Controller")
                    self._start = False
                    self._state = 1
                    self._loopTimeout = self.NUM_LOOPS_TIMEOUT
                    self._startFilling()
                    self._notifier.startPeriodic(self._streamRateMS / 1000)

        # The Talon MPE has started filling the buffer.  Once enough points have been loaded into
        # the bottom buffer, enable the Talon MPE.
        elif self._state == 1:
            if self._status.btmBufferCnt > self.MIN_NUM_POINTS:
                logger.info("Talon MPE bottom buffer is ready, enabling the Talon MPE")
                self._state = 2
                self._loopTimeout = self.NUM_LOOPS_TIMEOUT
                self._talon.set(WPI_TalonSRX.ControlMode.MotionProfile,
                                SetValueMotionProfile.Enable)

        # Check status of the MP and if there isn't an underrun condition, reset the loop timeout.
        # Once the last point has been processed, stop the notifier and set the Talon MPE to hold
        # the final position.
        # ***** TODO ***** (may want to go to neutral instead)
        elif self._state == 2:
            if not self._status.isUnderrun:
                self._loopTimeout = self.NUM_LOOPS_TIMEOUT
            else:
                self._outputStatus()

            if self._status.activePointValid and self._status.isLast:
                logger.info("Talon MPE is at the last trajectory point")
                self._state = 3
                self._notifier.stop()
                logger.info("Called to stop Motion Profile Controller notifier")
                self._talon.set(WPI_TalonSRX.ControlMode.MotionProfile,
                                SetValueMotionProfile.Disable)

        elif self._state == 3:
            self._state = 0
            self._loopTimeout -= 1
            self._finished = True

            # Remove the reference to the notifier and hopefully the notifier will be stopped
            self._notifier.free()
            del(self._notifier)

        # Service the loop timeout
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
        This method will initialize the motion profile controller by clearing out any trajectories
        still in the buffer, setting the control mode to motion profile (disabled), change the
        control frame period to the stream rate, and clearing any prior buffer underruns.
        """
        self._talon.changeMotionControlFramePeriod(self._streamRateMS)
        self._talon.set(WPI_TalonSRX.ControlMode.MotionProfile,
                        SetValueMotionProfile.Disable)
        if self._status.hasUnderrun:
            logger.warning("Clearing UNDERRUN condition during reset")
            self._talon.clearMotionProfileHasUnderrun(0)
        if self._status.btmBufferCnt != 0 or self._status.topBufferCnt != 0:
            logger.warning("Clearing MPE buffer(s)")
            self._talon.clearMotionProfileTrajectories()

    def _streamToMotionProfileBuffer(self):
        """
        This method will move the trajectory points from the top-buffer to the bottom-buffer.
        """
        if self._debugCnt == 0:
            if self._finished:
                logger.warning('Motion Profile Controller notifier is running')
            self._debugCnt = self.NOTIFIER_DEBUG_CNT
        else:
            self._debugCnt -= 1
        if self._status.btmBufferCnt < 100:
            self._talon.processMotionProfileBuffer()

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
            logger.critical("Unsupported duration %i" % (duration))
            return TrajectoryPoint.TrajectoryDuration.T0ms

    def _startFilling(self):
        """
        This method will start filling the top buffer of the Talon MPE.  This will execute quickly.
        """
        for i in range(len(self._points)):
            point = TrajectoryPoint(-self._points[i][0] if self.reverse else self._points[i][0],
                                    -self._points[i][1] if self.reverse else self._points[i][1],
                                    self._points[i][2],
                                    self._profileSlotSelect0,
                                    self._profileSlotSelect1,
                                    True if i+1 == len(self._points) else False,
                                    True if i == 0 else False,
                                    self._getTrajectoryDuration(self._points[0][3]))
            self._talon.pushMotionProfileTrajectory(point)

    def _outputStatus(self):
        logger.warning("isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, topBufferCnt: %i, "
                       "btmBufferCnt: %i, activePointValid: %s, isLast: %s" %
                       (self._status.isUnderrun, self._status.hasUnderrun,
                        self._status.topBufferRem, self._status.topBufferCnt,
                        self._status.btmBufferCnt, self._status.activePointValid,
                        self._status.isLast))
        # "profileSlotSelect0", "outputEnable", "timeDurMs", "profileSlotSelect1",
