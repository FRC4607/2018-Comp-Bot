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
    NUM_LOOPS_TIMEOUT = 10000
    SYNC_CONSTANT = 2
    SYNC_SIDE_LEFT = True
    MP_TYPE = WPI_TalonSRX.ControlMode.MotionProfileArc
    #  MP_TYPE = WPI_TalonSRX.ControlMode.MotionProfile

    def __init__(self, left_talon, left_points, right_talon, right_points, reverse,
                 profile_slot_select0, profile_slot_select1):

        # Reference to the motion profile to run
        self._leftPoints = left_points
        self._rightPoints = right_points
        self._profileSlotSelect0 = profile_slot_select0
        self._profileSlotSelect1 = profile_slot_select1
        self.reverse = reverse

        # Reference to the Talon SRX being used
        self._leftTalon = left_talon
        self._rightTalon = right_talon

        # Create objects to receive the status
        self._leftStatus = MotionProfileStatus
        self._rightStatus = MotionProfileStatus

        # Control variables
        self._start = False
        self._state = 0
        self._finished = True
        self._loopTimeout = -1
        self._debugCnt = self.NOTIFIER_DEBUG_CNT

        # Create a _notifier to stream trajectory points into the talon.  If the input
        # stream_rate_ms is greater than 40ms, then it would be better to call
        # streamMotionProfileBuffer() in the teleop/autonomous loops since we will stream at twice
        # the rate of the motion profile
        self._streamRateMS = int(left_points[0][3] / 2)
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

    def control(self, time_stamp, smart_dashboard):
        """
        This method is called by the command every 20ms in autonomous or teleop.
        """
        # Get the current status
        self._leftStatus = self._leftTalon.getMotionProfileStatus()
        self._rightStatus = self._rightTalon.getMotionProfileStatus()

        # Waiting for the start signal.  Once received, make sure the Talon MPE is in a state to
        # begin executing a new motion profile.
        if self._state == 0:
            if self._start:
                # Make sure the Talon MPE is disabled before servicing the start signal
                if self._leftStatus.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Left Talon MPE")
                    self._leftTalon.set(self.MP_TYPE,
                                        SetValueMotionProfile.Disable)
                elif self._rightStatus.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Right Talon MPE")
                    self._rightTalon.set(self.MP_TYPE,
                                         SetValueMotionProfile.Disable)

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
            if (self._leftStatus.btmBufferCnt > self.MIN_NUM_POINTS and
                    self._rightStatus.btmBufferCnt > self.MIN_NUM_POINTS):
                logger.info("Talon MPE bottom buffer is ready, enabling the Talon MPE")
                self._state = 2
                self._loopTimeout = self.NUM_LOOPS_TIMEOUT
                self._leftTalon.set(self.MP_TYPE,
                                    SetValueMotionProfile.Enable)
                self._rightTalon.set(self.MP_TYPE,
                                     SetValueMotionProfile.Enable)
        # Check status of the MP and if there isn't an underrun condition, reset the loop timeout.
        # Once the last point has been processed, stop the notifier and set the Talon MPE to hold
        # the final position.
        elif self._state == 2:
            if not self._leftStatus.isUnderrun and not self._rightStatus.isUnderrun:
                self._loopTimeout = self.NUM_LOOPS_TIMEOUT
            else:
                self._outputStatus()

            if (self._leftStatus.activePointValid and self._leftStatus.isLast and
                    self._rightStatus.activePointValid and self._rightStatus.isLast):
                logger.info("Talon MPEs are at the last trajectory point")
                self._state = 3
                self._notifier.stop()
                logger.info("Called to stop Motion Profile Controller notifier")
                self._leftTalon.set(self.MP_TYPE,
                                    SetValueMotionProfile.Disable)
                self._rightTalon.set(self.MP_TYPE,
                                     SetValueMotionProfile.Disable)

            # Output debug data to the smartdashboard
            if LOGGER_LEVEL == logging.DEBUG:
                smart_dashboard.putNumber("RightEncPos",
                                          self._rightTalon.getSensorCollection().getQuadraturePosition())
                smart_dashboard.putNumber("RightActPos",
                                          self._rightTalon.getActiveTrajectoryPosition())
                smart_dashboard.putNumber("RightEncVel",
                                          self._rightTalon.getAnalogInVel())
                smart_dashboard.putNumber("RightActVel",
                                          self._rightTalon.getActiveTrajectoryVelocity())
                smart_dashboard.putNumber("RightPrimaryError",
                                          self._rightTalon.getClosedLoopError(0))
                smart_dashboard.putNumber("RightSecondaryError",
                                          self._rightTalon.getClosedLoopError(1))

                smart_dashboard.putNumber("LeftEncPos",
                                          self._leftTalon.getSensorCollection().getQuadraturePosition())
                smart_dashboard.putNumber("LeftActPos",
                                          self._leftTalon.getActiveTrajectoryPosition())
                smart_dashboard.putNumber("LeftEncVel",
                                          self._leftTalon.getAnalogInVel())
                smart_dashboard.putNumber("LeftActVel",
                                          self._leftTalon.getActiveTrajectoryVelocity())
                smart_dashboard.putNumber("LeftPrimaryError",
                                          self._leftTalon.getClosedLoopError(0))
                smart_dashboard.putNumber("LeftSecondaryError",
                                          self._leftTalon.getClosedLoopError(1))
                smart_dashboard.putNumber("RightTopBufferCount", self._rightStatus.topBufferCnt)
                smart_dashboard.putNumber("LeftTopBufferCount", self._leftStatus.topBufferCnt)
                smart_dashboard.putNumber("LeftBottomBufferCount", self._leftStatus.btmBufferCnt)
                smart_dashboard.putNumber("RightBottomBufferCount", self._rightStatus.btmBufferCnt)
                smart_dashboard.putNumber("TimeStamp", time_stamp)

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
        self._leftTalon.set(self.MP_TYPE,
                            SetValueMotionProfile.Disable)
        self._rightTalon.set(self.MP_TYPE,
                             SetValueMotionProfile.Disable)
        if self._leftStatus.hasUnderrun:
            logger.warning("Clearing Left Talon UNDERRUN condition during reset")
            self._leftTalon.clearMotionProfileHasUnderrun(0)
        if self._rightStatus.hasUnderrun:
            logger.warning("Clearing Right Talon UNDERRUN condition during reset")
            self._rightTalon.clearMotionProfileHasUnderrun(0)
        if self._leftStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
            logger.warning("Clearing Left Talon MPE buffer(s) during reset")
            self._leftTalon.clearMotionProfileTrajectories()
        if self._rightStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
            logger.warning("Clearing Right Talon MPE buffer(s) during reset")
            self._rightTalon.clearMotionProfileTrajectories()

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
        if self._leftStatus.btmBufferCnt < 100 and self._rightStatus.btmBufferCnt < 100:
            self._leftTalon.processMotionProfileBuffer()
            self._rightTalon.processMotionProfileBuffer()

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
        # This will insert a few 0 points at the beginning of the profile in order to try and get
        # both of the Talon's in sync with one another.

        # This will fill the top buffers of the Talons.
        for i in range(self.SYNC_CONSTANT):
            if self.SYNC_SIDE_LEFT:
                point = TrajectoryPoint(0.0,
                                        0.0,
                                        self._leftPoints[i][2],
                                        self._profileSlotSelect0,
                                        self._profileSlotSelect1,
                                        False,
                                        False if i == 0 else True,
                                        self._getTrajectoryDuration(self._leftPoints[0][3]))
                self._leftTalon.pushMotionProfileTrajectory(point)
        for i in range(len(self._leftPoints)):
            point = TrajectoryPoint(-self._leftPoints[i][0] if self.reverse else self._leftPoints[i][0],
                                    -self._leftPoints[i][1] if self.reverse else self._leftPoints[i][1],
                                    self._leftPoints[i][2],
                                    self._profileSlotSelect0,
                                    self._profileSlotSelect1,
                                    True if i+1 == len(self._leftPoints) else False, # Last point
                                    True if i == 0 and self.SYNC_CONSTANT == 0 else False, # zero
                                    self._getTrajectoryDuration(self._leftPoints[i][3]))
            self._leftTalon.pushMotionProfileTrajectory(point)
            point = TrajectoryPoint(-self._rightPoints[i][0] if self.reverse else self._rightPoints[i][0],
                                    -self._rightPoints[i][1] if self.reverse else self._rightPoints[i][1],
                                    self._leftPoints[i][2],
                                    self._profileSlotSelect0,
                                    self._profileSlotSelect1,
                                    True if i+1 == len(self._rightPoints) else False,
                                    True if i == 0 else False,
                                    self._getTrajectoryDuration(self._rightPoints[i][3]))
            self._rightTalon.pushMotionProfileTrajectory(point)


    def _outputStatus(self):
        logger.warning("LEFT: isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, "
                       "topBufferCnt: %i, btmBufferCnt: %i, activePointValid: %s, isLast: %s, "
                       "mode: %i, timeDureMS: %i" %
                       (self._leftStatus.isUnderrun, self._leftStatus.hasUnderrun,
                        self._leftStatus.topBufferRem, self._leftStatus.topBufferCnt,
                        self._leftStatus.btmBufferCnt, self._leftStatus.activePointValid,
                        self._leftStatus.isLast, self._leftStatus.outputEnable,
                        self._leftStatus.timeDurMs))
        logger.warning("RIGHT: isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, "
                       "topBufferCnt: %i, btmBufferCnt: %i, activePointValid: %s, isLast: %s, "
                       "mode: %i, timeDureMS: %i" %
                       (self._rightStatus.isUnderrun, self._rightStatus.hasUnderrun,
                        self._rightStatus.topBufferRem, self._rightStatus.topBufferCnt,
                        self._rightStatus.btmBufferCnt, self._rightStatus.activePointValid,
                        self._rightStatus.isLast, self._rightStatus.outputEnable,
                        self._rightStatus.timeDurMs))
