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
    MIN_NUM_POINTS = 50
    NUM_LOOPS_TIMEOUT = 10
    SYNC_CONSTANT = 2
    SYNC_SIDE_LEFT = True

    def __init__(self, left_talon, left_points, right_talon, right_points,
                 profile_slot_select0, profile_slot_select1):

        # Reference to the motion profile to run
        self._leftPoints = left_points
        self._rightPoints = right_points
        self._profileSlotSelect0 = profile_slot_select0
        self._profileSlotSelect1 = profile_slot_select1

        # Reference to the Talon SRX being used
        self._leftTalon = left_talon
        self._rightTalon = right_talon

        # Reference used to collect the status of the named tuple data transfer object
        self._leftStatus = MotionProfileStatus
        self._rightStatus = MotionProfileStatus

        # Reference used to collect the trajectory point of the named tuple data transfer object
        self._leftPoint = TrajectoryPoint
        self._rightPoint = TrajectoryPoint

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
        self._getTrajectoryDuration(left_points[0][2])
        self._streamRateMS = int(left_points[0][2] / 2)
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

        # Service the loop timeout
        if self._loopTimeout < 0:
            pass
        else:
            if self._loopTimeout == 0:
                logger.warning("No progress being made - State = %i" % (self._state))
                self._outputStatus()
            else:
                self._loopTimeout -= 1

        # Waiting for the start signal.  Once received, make sure the Talon MPE is in a state to
        # begin executing a new motion profile.
        if self._state == 0:
            if self._start:
                logger.info("Starting the Motion Profile Controller")

                # Make sure the Talon MPE is disabled before servicing the start signal
                if self._leftStatus.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Left Talon MPE")
                    self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                                        SetValueMotionProfile.Disable)
                elif self._rightStatus.outputEnable != SetValueMotionProfile.Disable:
                    logger.warning("Disabling the Right Talon MPE")
                    self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
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
                    self._start = False
                    self._state = 1
                    self._loopTimeout = self.MIN_NUM_POINTS
                    self._startFilling()
                    self._notifier.startPeriodic(self._streamRateMS / 1000)
                    logger.info("Started Motion Profile Controller _notifier")

        # The Talon MPE has started filling the buffer.  Once enough points have been loaded into
        # the bottom buffer, enable the Talon MPE.
        elif self._state == 1:
            if (self._leftStatus.btmBufferCnt > self.MIN_NUM_POINTS and
                    self._rightStatus.btmBufferCnt > self.MIN_NUM_POINTS):
                self._state = 2
                self._loopTimeout = self.MIN_NUM_POINTS
                self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                                    SetValueMotionProfile.Enable)
                self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                                     SetValueMotionProfile.Enable)
        # Check status of the MP and if there isn't an underrun condition, reset the loop timeout.
        # Once the last point has been processed, stop the notifier and set the Talon MPE to hold
        # the final position.
        # ***** TODO ***** (may want to go to neutral instead)
        elif self._state == 2:
            if not self._leftStatus.isUnderrun and not self._rightStatus.isUnderrun:
                logger.debug("Left Buffer: %i, Right Buffer: %i" %
                             (self._leftStatus.topBufferCnt, self._rightStatus.topBufferCnt))
                self._loopTimeout = self.MIN_NUM_POINTS
            else:
                logger.warning("Talon MPE UNDERRUN: left: %s, Right: %s" %
                               (self._leftStatus.isUnderrun, self._rightStatus.isUnderrun))

            if (self._leftStatus.activePointValid and self._leftStatus.isLast and
                    self._rightStatus.activePointValid and self._rightStatus.isLast):
                logger.info("Talon MPEs are at the last trajectory point")
                self._state = 3
                self._notifier.stop()
                logger.info("Called to stop Motion Profile Controller notifier")
                self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                                    SetValueMotionProfile.Disable)
                self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                                     SetValueMotionProfile.Disable)

            # Output debug data to the smartdashboard
            if LOGGER_LEVEL == logging.DEBUG:
                smart_dashboard.putNumber("RightEncPos",
                                          self._rightTalon.getSensorCollection().getQuadraturePosition())
                smart_dashboard.putNumber("RightActPos",
                                          self._rightTalon.getActiveTrajectoryPosition())
                smart_dashboard.putNumber("RightEncVel",
                                          self._rightTalon.getQuadratureVelocity())
                smart_dashboard.putNumber("RightActVel",
                                          self._rightTalon.getActiveTrajectoryPosition())
                smart_dashboard.putNumber("RightPrimaryTarget",
                                          self._rightTalon.getClosedLoopTarget(0))
                smart_dashboard.putNumber("RightPrimaryError",
                                          self._rightTalon.getClosedLoopError(0))
                smart_dashboard.putNumber("LeftEncPos",
                                          self._leftTalon.getSensorCollection().getQuadraturePosition())
                smart_dashboard.putNumber("LeftActPos",
                                          self._leftTalon.getActiveTrajectoryPosition())
                smart_dashboard.putNumber("LeftEncVel",
                                          self._leftTalon.getQuadratureVelocity())
                smart_dashboard.putNumber("LeftActVel",
                                          self._leftTalon.getActiveTrajectoryPosition())
                smart_dashboard.putNumber("LeftPrimaryTarget",
                                          self._leftTalon.getClosedLoopTarget(0))
                smart_dashboard.putNumber("LeftPrimaryError",
                                          self._leftTalon.getClosedLoopError(0))
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

    def _initialize(self):
        """
        This method will initialize the motion profile controller by clearing out any trajectories
        still in the buffer, setting the control mode to motion profile (disabled), change the
        control frame period to the stream rate, and clearing any prior buffer underruns.
        """
        self._leftTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                            SetValueMotionProfile.Disable)
        self._rightTalon.set(WPI_TalonSRX.ControlMode.MotionProfileArc,
                             SetValueMotionProfile.Disable)
        if self._leftStatus.hasUnderrun:
            logger.warning("Clearing Left Talon UNDERRUN condition during reset")
            self._leftTalon.clearMotionProfileHasUnderrun(0)
        if self._rightStatus.hasUnderrun:
            logger.warning("Clearing Right Talon UNDERRUN condition during reset")
            self._rightTalon.clearMotionProfileHasUnderrun(0)
        if self._leftStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
            logger.warning("Clearing Left Talon MPE buffer(s)")
            self._leftTalon.clearMotionProfileTrajectories()
        if self._rightStatus.btmBufferCnt != 0 or self._leftStatus.topBufferCnt != 0:
            logger.warning("Clearing Right Talon MPE buffer(s)")
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
        for i in range(self.SYNC_CONSTANT):
            if self.SYNC_SIDE_LEFT:
                self._leftPoint.position = 0
                self._leftPoint.velocity = 0
                self._leftPoint.auxiliaryPos = 0
                self._leftPoint.profileSlotSelect0 = self._profileSlotSelect0
                self._leftPoint.profileSlotSelect1 = self._profileSlotSelect1
                self._leftPoint.timeDur = self._getTrajectoryDuration(self._leftPoints[0][3])
                self._leftPoint.zeroPos = False
                self._leftPoint.isLastPoint = False
                self._leftTalon.pushMotionProfileTrajectory(self._leftPoint)

        for i in range(len(self._leftPoints)):
            self._leftPoint.position = self._leftPoints[i][0]
            self._leftPoint.velocity = self._leftPoints[i][1]
            self._leftPoint.auxiliaryPos = self._leftPoints[i][2]
            self._leftPoint.profileSlotSelect0 = self._profileSlotSelect0
            self._leftPoint.profileSlotSelect1 = self._profileSlotSelect1
            self._leftPoint.timeDur = self._getTrajectoryDuration(self._leftPoints[i][3])
            self._leftPoint.zeroPos = False
            if i == 0:
                self._leftPoint.zeroPos = True
            self._leftPoint.isLastPoint = False
            if i+1 == len(self._leftPoints):
                self._leftPoint.isLastPoint = True
            self._leftTalon.pushMotionProfileTrajectory(self._leftPoint)

            self._rightPoint.position = self._rightPoints[i][0]
            self._rightPoint.velocity = self._rightPoints[i][1]
            self._rightPoint.auxiliaryPos = self._rightPoints[i][2]
            self._rightPoint.profileSlotSelect0 = self._profileSlotSelect0
            self._rightPoint.profileSlotSelect1 = self._profileSlotSelect1
            self._rightPoint.timeDur = self._getTrajectoryDuration(self._rightPoints[i][3])
            self._rightPoint.zeroPos = False
            if i == 0:
                self._rightPoint.zeroPos = True
            self._rightPoint.isLastPoint = False
            if i+1 == len(self._rightPoints):
                self._rightPoint.isLastPoint = True
            self._rightTalon.pushMotionProfileTrajectory(self._rightPoint)

    def _outputStatus(self):
        logger.warning("LEFT: isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, "
                       "topBufferCnt: %i, btmBufferCnt: %i, activePointValid: %s, isLast: %s" %
                       (self._leftStatus.isUnderrun, self._leftStatus.hasUnderrun,
                        self._leftStatus.topBufferRem, self._leftStatus.topBufferCnt,
                        self._leftStatus.btmBufferCnt, self._leftStatus.activePointValid,
                        self._leftStatus.isLast))
        logger.warning("RIGHT: isUnderrun: %s, hasUnderrun: %s, topBufferRem: %s, "
                       "topBufferCnt: %i, btmBufferCnt: %i, activePointValid: %s, isLast: %s" %
                       (self._rightStatus.isUnderrun, self._rightStatus.hasUnderrun,
                        self._rightStatus.topBufferRem, self._rightStatus.topBufferCnt,
                        self._rightStatus.btmBufferCnt, self._rightStatus.activePointValid,
                        self._rightStatus.isLast))
