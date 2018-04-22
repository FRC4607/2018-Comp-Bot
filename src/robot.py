from wpilib import Timer, TimedRobot, run, SendableChooser, CameraServer
from wpilib.command import Scheduler
from wpilib.driverstation import DriverStation
from wpilib.smartdashboard import SmartDashboard
from subsystems.intake_pneumatics import IntakePneumatics
from subsystems.intake_motors import IntakeMotors
from subsystems.drivetrain import DriveTrain
from subsystems.boom import Boom
from oi import OI
from autonomous.autonomous_chooser import *
from autonomous.auton_forward import AutonForward
from autonomous.auton_middle_start_left_switch import AutonMiddleStartLeftSwitch
from autonomous.auton_left_start_left_scale import AutonLeftStartLeftScale
from constants import BOOM_STATE, LOGGER_LEVEL
import logging
logger = logging.getLogger(__name__)
logger.setLevel(LOGGER_LEVEL)


class Pitchfork(TimedRobot):

    def robotInit(self):
        """
        Robot-wide initialization code goes here.  For the command-based programming framework,
        this means creating the subsytem objects and the operator input object.  BE SURE TO CREATE
        THE SUBSYTEM OBJECTS BEFORE THE OPERATOR INPUT OBJECT (since the operator input object
        will almost certainly have subsystem dependencies).  For further information on the
        command-based programming framework see:
        wpilib.screenstepslive.com/s/currentCS/m/java/l/599732-what-is-command-based-programming
        """
        # Create the subsytems and the operator interface objects
        self.driveTrain = DriveTrain(self)
        self.boom = Boom(self)
        self.intakeMotors = IntakeMotors(self)
        self.intakePneumatics = IntakePneumatics(self)
        self.oi = OI(self)

        # Create the smartdashboard object
        self.smartDashboard = SmartDashboard()

        # Create the sendable choosers to get the autonomous preferences
        self.scoringElementChooser = SendableChooser()
        self.scoreScale = ScoreScale()
        self.scoreSwitch = ScoreSwitch()
        self.scoringElementChooser.addObject("Score Scale", self.scoreScale)
        self.scoringElementChooser.addObject("Score Switch", self.scoreSwitch)
        self.scoringElementChooser.addDefault("Score Scale", self.scoreScale)
        self.smartDashboard.putData("Score Field Element", self.scoringElementChooser)

        self.crossFieldChooser = SendableChooser()
        self.crossFieldEnable = CrossFieldEnable()
        self.crossFieldDisable = CrossFieldDisable()
        self.crossFieldChooser.addObject("Cross Field Enable", self.crossFieldEnable)
        self.crossFieldChooser.addObject("Cross Field Disable", self.crossFieldDisable)
        self.crossFieldChooser.addDefault("Cross Field Disable", self.crossFieldDisable)
        self.smartDashboard.putData("Cross Field Enable", self.crossFieldChooser)

        self.positionChooser = SendableChooser()
        self.startingLeft = StartingLeft()
        self.startingRight = StartingRight()
        self.startingMiddle = StartingMiddle()
        self.positionChooser.addObject("Start Left", self.startingLeft)
        self.positionChooser.addObject("Start Right", self.startingRight)
        self.positionChooser.addObject("Start Middle", self.startingMiddle)
        self.positionChooser.addDefault("Start Middle", self.startingMiddle)
        self.smartDashboard.putData("Starting Position", self.positionChooser)

        # Create a timer for data logging
        self.timer = Timer()

        # Create the camera server
        CameraServer.launch()

        # Boom state start at the scale
        self.boomState = BOOM_STATE.Scale

        self.autonForward = AutonForward(self)
        self.autonMiddleStartLeftSwitch = AutonMiddleStartLeftSwitch(self)
        self.autonLeftStartLeftScale = AutonLeftStartLeftScale(self)


        # Output debug data to the smartdashboard
        if LOGGER_LEVEL == logging.DEBUG:
            #=======================================================================================
            # self.smartDashboard.putNumber("RightEncPos", 0.0)
            # self.smartDashboard.putNumber("RightActPos", 0.0)
            # self.smartDashboard.putNumber("RightEncVel", 0.0)
            # self.smartDashboard.putNumber("RightActVel", 0.0)
            # self.smartDashboard.putNumber("RightPrimaryTarget", 0.0)
            # self.smartDashboard.putNumber("RightPrimaryError", 0.0)
            # self.smartDashboard.putNumber("TimeStamp", 0.0)
            # self.smartDashboard.putNumber("LeftEncPos", 0.0)
            # self.smartDashboard.putNumber("LeftActPos", 0.0)
            # self.smartDashboard.putNumber("LeftEncVel", 0.0)
            # self.smartDashboard.putNumber("LeftActVel", 0.0)
            # self.smartDashboard.putNumber("LeftPrimaryTarget", 0.0)
            # self.smartDashboard.putNumber("LeftPrimaryError", 0.0)
            # self.smartDashboard.putNumber("RightTopBufferCount", 0.0)
            # self.smartDashboard.putNumber("LeftTopBufferCount", 0.0)
            # self.smartDashboard.putNumber("LeftBottomBufferCount", 0.0)
            # self.smartDashboard.putNumber("RightBottomBufferCount", 0.0)
            #=======================================================================================
            self.smartDashboard.putNumber("EncPos", 0.0)
            self.smartDashboard.putNumber("ActPos", 0.0)
            self.smartDashboard.putNumber("EncVel", 0.0)
            self.smartDashboard.putNumber("ActVel", 0.0)
            self.smartDashboard.putNumber("PrimaryTarget", 0.0)
            self.smartDashboard.putNumber("PrimaryError", 0.0)
            self.smartDashboard.putNumber("TimeStamp", 0.0)
            
    def disabledInit(self):
        """
        Initialization code for disabled mode should go here.  This method will be called each
        time the robot enters disabled mode.
        """
        self.timer.stop()
        self.timer.reset()

    def disabledPeriodic(self):
        """
        Periodic code for disabled mode should go here.  This method will be called every 20ms.
        """
        if self.timer.running:
            self.timer.stop()
            self.timer.reset()

    def robotPeriodic(self):
        pass

    def autonomousInit(self):
        """
        Initialization code for autonomous mode should go here.  This method will be called each
        time the robot enters autonomous mode.
        """
        self.scheduleAutonomous = True
        if not self.timer.running:
            self.timer.start()

        # Get the prioritized scoring element, robot starting posion, and the alliance
        # scale/switch data.
        self.selectedCrossFieldEnable = self.crossFieldChooser.getSelected()
        self.selectedScoringElement = self.scoringElementChooser.getSelected()
        self.selectedStartingPosition = self.positionChooser.getSelected()
        
        self.gameData = DriverStation.getInstance().getGameSpecificMessage()

        self.crossFieldEnable = self.selectedCrossFieldEnable.getCrossFieldEnable()
        self.scoringElement = self.selectedScoringElement.getScoringElement()        
        self.startingPosition = self.selectedStartingPosition.getStartingPosition()

    def autonomousPeriodic(self):
        """
        Periodic code for autonomous mode should go here.  This method will be called every 20ms.
        """
        # The game specific data will be a 3-character string representing where the teams switch,
        # scale, switch are located.  For example, "LRR" means your teams closest switch is on the
        # left (as you look out onto the field from the drivers station).  The teams scale is on
        # the right, and the switch furthest away is also on the right.
        if self.scheduleAutonomous:
            self.scheduleAutonomous = False
            logger.info("Game Data: %s" % (self.gameData))
            logger.info("Cross Field Enable: %s" % (self.crossFieldEnable))
            logger.info("Scoring Element %s" % (self.scoringElement))
            logger.info("Starting Position %s" % (self.startingPosition))    
  
            self.autonMiddleStartLeftSwitch.start()
            #self.autonLeftStartLeftScale.start()
            #self.autonForward.start()

        
        Scheduler.getInstance().run()

    def teleopInit(self):
        """
        Initialization code for teleop mode should go here.  This method will be called each time
        the robot enters teleop mode.
        """
        if not self.timer.running:
            self.timer.start()

    def teleopPeriodic(self):
        """
        Periodic code for teleop mode should go here.  This method will be called every 20ms.
        """
        Scheduler.getInstance().run()


if __name__ == "__main__":
    run(Pitchfork)
