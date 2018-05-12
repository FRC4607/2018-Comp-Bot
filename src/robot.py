from wpilib import Timer, TimedRobot, run, SendableChooser, CameraServer
from wpilib.command import Scheduler
from wpilib.driverstation import DriverStation
from wpilib.smartdashboard import SmartDashboard
from subsystems.intake_pneumatics import IntakePneumatics
from subsystems.intake_motors import IntakeMotors
from subsystems.drivetrain import DriveTrain
from subsystems.boom import Boom
from oi import OI
from autonomous.auton_forward import AutonForward
from autonomous.auton_left_start_left_scale import AutonLeftStartLeftScale
from autonomous.auton_right_start_right_scale import AutonRightStartRightScale
from autonomous.auton_left_start_left_switch import AutonLeftStartLeftSwitch
from autonomous.auton_right_start_right_switch import AutonRightStartRightSwitch
from autonomous.auton_middle_start_left_switch import AutonMiddleStartLeftSwitch
from autonomous.auton_middle_start_right_switch import AutonMiddleStartRightSwitch
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
        self.startSpotChooser = SendableChooser()
        self.startSpotChooser.addObject("Start Left", 'Left')
        self.startSpotChooser.addObject("Start Right", 'Right')
        self.startSpotChooser.addDefault("Start Middle", 'Middle')
        self.smartDashboard.putData("Starting Spot", self.startSpotChooser)

        self.scaleDisableChooser = SendableChooser()        
        self.scaleDisableChooser.addObject("Enable Scale", 'Scale')
        self.scaleDisableChooser.addDefault("Disable Scale", 'No Scale')
        self.smartDashboard.putData("Scale Enable", self.scaleDisableChooser)

        # Build up the autonomous dictionary.  Fist key is the starting position.  The second key is the switch.  The third key is the scale.
        self.chooserOptions = {"Left": {"R": {"R": {"No Scale": {'command': AutonForward},
                                                    "Scale": {'command': AutonForward}
                                                    },
                                              "L": {"No Scale": {'command': AutonForward},
                                                    "Scale": {'command': AutonForward}
                                                    },
                                              },
                                        "L": {"R": {"No Scale": {'command': AutonLeftStartLeftSwitch},
                                                    "Scale": {'command': AutonLeftStartLeftSwitch}
                                                    },
                                              "L": {"No Scale": {'command': AutonLeftStartLeftSwitch},
                                                    "Scale": {'command': AutonLeftStartLeftSwitch}
                                                    },
                                              },
                                        },
                               "Middle": {"R": {"R": {"No Scale": {'command': AutonMiddleStartRightSwitch},
                                                      "Scale": {'command': AutonMiddleStartRightSwitch}
                                                      },
                                                "L": {"No Scale": {'command': AutonMiddleStartRightSwitch},
                                                      "Scale": {'command': AutonMiddleStartRightSwitch}
                                                      },
                                                },
                                          "L": {"R": {"No Scale": {'command': AutonMiddleStartLeftSwitch},
                                                      "Scale": {'command': AutonMiddleStartLeftSwitch}
                                                      },
                                                "L": {"No Scale": {'command': AutonMiddleStartLeftSwitch},
                                                      "Scale": {'command': AutonMiddleStartLeftSwitch}
                                                      },
                                                },
                                          },
                               "Right": {"R": {"R": {"No Scale": {'command': AutonRightStartRightSwitch},
                                                     "Scale": {'command': AutonRightStartRightScale}
                                                     },
                                               "L": {"No Scale": {'command': AutonRightStartRightSwitch},
                                                     "Scale": {'command': AutonRightStartRightSwitch}
                                                     },
                                               },
                                         "L": {"R": {"No Scale": {'command': AutonForward}, 
                                                     "Scale": {'command': AutonRightStartRightScale}
                                                     },
                                               "L": {"No Scale": {'command': AutonForward},
                                                     "Scale": {'command': AutonForward}
                                                     },
                                               },
                                         },
                               }

        # Create a timer for data logging
        self.timer = Timer()

        # Create the camera server
        CameraServer.launch()

        # Boom state start at the scale
        self.boomState = BOOM_STATE.Scale

        #===========================================================================================
        # if LOGGER_LEVEL == logging.INFO:
        #     self.smartDashboard.putNumber("rVelocity",
        #                                   self.driveTrain.getRightVelocity())
        #     self.smartDashboard.putNumber("lVelocity",
        #                                   self.driveTrain.getLeftVelocity())
        #     self.smartDashboard.putNumber("rVoltage",
        #                                   self.driveTrain.getRightVoltage())
        #     self.smartDashboard.putNumber("lVoltage",
        #                                   self.driveTrain.getLeftVoltage())
        #     self.smartDashboard.putNumber("TimeStamp",
        #                                   self.timer.get())
        #===========================================================================================

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

        # The game specific data will be a 3-character string representing where the teams switch,
        # scale, switch are located.  For example, "LRR" means your teams closest switch is on the
        # left (as you look out onto the field from the drivers station).  The teams scale is on
        # the right, and the switch furthest away is also on the right.
        self.startingPosition = self.startSpotChooser.getSelected()
        self.scaleDisable = self.scaleDisableChooser.getSelected()
        self.gameData = DriverStation.getInstance().getGameSpecificMessage()

        logger.info("Game Data: %s" % (self.gameData))
        logger.info("Starting Position %s" % (self.startingPosition))
        logger.info("Scale Enable %s" % (self.scaleDisable))
        
        self.autonCommand = self.chooserOptions[self.startingPosition][self.gameData[0]][self.gameData[1]][self.scaleDisable]['command'](self)
        self.autonCommand.start()

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
