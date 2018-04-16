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
        self.positionChooser = SendableChooser()
        self.startingLeft = StartingLeft()
        self.startingRight = StartingRight()
        self.startingMiddle = StartingMiddle()
        self.positionChooser.addObject("Start Left", self.startingLeft)
        self.positionChooser.addObject("Start Right", self.startingRight)
        self.positionChooser.addObject("Start Middle", self.startingMiddle)
        self.positionChooser.addDefault("Start Left", self.startingLeft)
        self.smartDashboard.putData("Starting Position", self.positionChooser)

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

        self.autonForward = AutonForward(self)

        # Create a timer for data logging
        self.timer = Timer()

        # Create the camera server
        CameraServer.launch()

        # Boom state start at the scale
        self.boomState = BOOM_STATE.Scale

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
        if not self.timer.running:
            self.timer.start()

        # Get the prioritized scoring element, robot starting posion, and the alliance
        # scale/switch data.
        self.startingPosition = self.positionChooser.getSelected().getStartingPosition()
        self.scoringElement = self.scoringElementChooser.getSelected().getScoringElement()
        self.crossFieldEnable = self.crossFieldChooser.getSelected().getCrossFieldEnable()

        # The game specific data will be a 3-character string representing where the teams switch,
        # scale, switch are located.  For example, "LRR" means your teams closest switch is on the
        # left (as you look out onto the field from the drivers station).  The teams scale is on
        # the right, and the switch furthest away is also on the right.
        self.gameData = DriverStation.getInstance().getGameSpecificMessage()

        logger.info("Game Data: %s" % (self.gameData))
        logger.info("Cross Field Enable: %s" % (self.crossFieldEnable))
        logger.info("Starting Position %s" % (self.startingPosition))
        logger.info("Scoring Element %s" % (self.scoringElement))

        # Starting on the left side
        if self.startingPosition == "Left":

            # Want to do scale
            if self.scoringElement == "Scale":
                if self.gameData[1] == "L":
                    logger.info("Start Left, Left Scale")

                elif self.gameData[1] == "R" and self.crossFieldEnable:
                    logger.info("Start Left, Right Scale")

                elif self.gameData[0] == "L":
                    logger.info("Start Left, Left Switch")

                else:
                    logger.info("Start Left, Go Forward")
                    self.autonForward = AutonForward(self)
                    self.autonForward.start()

            # Want to do the switch
            elif self._scoringElement == "Switch":
                if self.gameData[0] == "L":
                    logger.info("Start Left, Left Switch")

                elif self.gameData[0] == "R" and self.crossFieldEnable:
                    logger.info("Start Left, Right Switch")

                elif self.gameData[1] == "L":
                    logger.info("Start Left, Left Scale")

                else:
                    logger.info("Start Left, Go Forward")
                    self.autonForward = AutonForward(self)
                    self.autonForward.start()

        # Starting on the right side
        elif self._startingPosition == "Right":

            # Want to do scale
            if self._scoringElement == "Scale":
                if self.gameData[1] == "R":
                    logger.info("Start Right, Right Scale")

                elif self.gameData[1] == "L" and self.crossFieldEnable:
                    logger.info("Start Right, Left Scale")

                elif self.gameData[0] == "R":
                    logger.info("Start Right, Right Switch")

                else:
                    logger.info("Start Right, Go Forward")
                    self.autonForward = AutonForward(self)
                    self.autonForward.start()

            # Want to do the switch
            elif self.scoringElement == "Switch":
                if self.gameData[0] == "R":
                    logger.info("Start Right, Right Switch")

                elif self.gameData[0] == "L" and self.crossFieldEnable:
                    logger.info("Start Right, Left Switch")

                elif self.gameData[1] == "R":
                    logger.info("Start Right, Right Scale")

                else:
                    logger.info("Start Right, Go Forward")
                    self.autonForward = AutonForward(self)
                    self.autonForward.start()

        # Starting in the middle
        elif self.startingPosition == "Middle":
            if self.gameData[0] == "R":
                logger.info("Start Middle, Right Switch")

            elif self.gameData[0] == "L":
                logger.info("Start Middle, Left Switch")

    def autonomousPeriodic(self):
        """
        Periodic code for autonomous mode should go here.  This method will be called every 20ms.
        """
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
