import math
import pickle
import os.path
from constants import *
import matplotlib.pyplot as plt
import numpy as np
import pathfinder as pf


def CalculateFeedForwardVoltage(leftSide, velocity, acceleration):
    """
    This function will take the velocity and acceration from the pathfinder generated trajectory and output an applied voltage.  The applied voltage
    can then be used as a feed-forward to the motion profile controller.  This is based off of prior work fround here =>
    https://www.chiefdelphi.com/media/papers/3402

    Vapp = kV * Velocity + kA * Acceleration + V-Intercept

    """
    if acceleration >= DRIVETRAIN_MAX_ACCELERATION:
        print("WARNING: The acceration is larger than the max!!")

    if velocity >= DRIVETRAIN_MAX_VELOCITY:
        print("WARNING: The velocity is larger than the max!!")

    if leftSide:
        kV = DRIVETRAIN_LEFT_KV
        kA = DRIVETRAIN_LEFT_KA
        VIntercept = DRIVETRAIN_LEFT_V_INTERCEPT
    else:
        kV = DRIVETRAIN_RIGHT_KV
        kA = DRIVETRAIN_RIGHT_KA
        VIntercept = DRIVETRAIN_RIGHT_V_INTERCEPT

    return kV * velocity + kA * acceleration + VIntercept


def GenerateTalonMotionProfileArcPath(path_name, file_name, waypoints, settings, reverse=False, heading_override=False, heading_value=0.0):
    """
    This function will take a set of pathfinder waypoints and create the trajectories to follow a path going through the waypoints.  This path is
    specific for the drivetrain controllers, so the position will use the CTRE quadrature encoders, the velocity will use the feed-forward in
    units of Volts, and the heading will use the Pigeon IMU.
    """
    # Generate the path using pathfinder.
    info, trajectory = pf.generate(waypoints, settings.order, settings.samples, settings.period,
                                   settings.maxVelocity, settings.maxAcceleration, settings.maxJerk)

    # Modify the path for the differential drive based on the calibrated wheelbase
    modifier = pf.modifiers.TankModifier(trajectory).modify(ROBOT_WHEELBASE_FT)

    # Get the left and right trajectories
    leftTrajectory = modifier.getLeftTrajectory()
    rightTrajectory = modifier.getRightTrajectory()

    # Grab the position, velocity + acceleration for feed-forward, heading, and duration.  Apply the proper conversions for the position,
    # feed-forward, and heading.  The headings from pathfinder will likely be fixed
    path = {"left": [], "right": []}
    headings = {"left": [], "right": []}
    for i in range(len(leftTrajectory)):
        heading = pf.r2d(leftTrajectory[i].heading)
        if heading_override:
            heading = heading_value
        else:
            if not reverse:
                if pf.r2d(leftTrajectory[i].heading) > 180:
                    heading = pf.r2d(leftTrajectory[i].heading) - 360
            else:
                if pf.r2d(leftTrajectory[i].heading) >= 180:
                    heading = -(pf.r2d(leftTrajectory[i].heading) - 180)
                else:
                    heading = -pf.r2d(leftTrajectory[i].heading) - 180
        headings["left"].append(heading)
        path["left"].append([leftTrajectory[i].position * 4096 /                            # Position: CTRE SRX Mag encoder: 4096 units per rotation
                             (ROBOT_WHEEL_DIAMETER_FT * math.pi),                           # Voltage / Feed-Forward
                             CalculateFeedForwardVoltage(True,
                                                         leftTrajectory[i].velocity,
                                                         leftTrajectory[i].acceleration),
                             3600 * heading / 360,                                          # Pigeon IMU setup for 3600 units per rotation
                             int(leftTrajectory[i].dt * 1000)])                             # Duration
        heading = pf.r2d(rightTrajectory[i].heading)
        if heading_override:
            heading = heading_value
        else:
            if not reverse:
                if pf.r2d(rightTrajectory[i].heading) > 180:
                    heading = pf.r2d(rightTrajectory[i].heading) - 360
            else:
                if pf.r2d(rightTrajectory[i].heading) >= 180:
                    heading = -(pf.r2d(rightTrajectory[i].heading) - 180)
                else:
                    heading = -pf.r2d(rightTrajectory[i].heading) - 180
                        
        headings["right"].append(heading)
        path["right"].append([rightTrajectory[i].position * 4096 /
                              (ROBOT_WHEEL_DIAMETER_FT * math.pi),
                              CalculateFeedForwardVoltage(False,
                                                          rightTrajectory[i].velocity,
                                                          rightTrajectory[i].acceleration),
                              3600 * heading / 360,
                              int(rightTrajectory[i].dt * 1000)])

    # Dump the path into a pickle file which will be read up later by the RoboRIO robot code
    with open(os.path.join(path_name, file_name+".pickle"), "wb") as fp:
        pickle.dump(path, fp)

    # Plot the X,Y points to see if the paths go where desired
    x = list(i * (settings.period) for i, _ in enumerate(leftTrajectory))
    plt.figure()
    plt.title("Trajectory")
    drawField(plt)
    # Pathfinder +X is forward and +Y is right, flip axis for easier viewing also flip the label of the trajectory sides.  The velocity and heading
    # plots are the gold standards for direction.
    plt.plot([-segment.y for segment in leftTrajectory],
             [segment.x for segment in leftTrajectory],
             marker='.', color='b')
    plt.plot([-segment.y for segment in rightTrajectory],
             [segment.x for segment in rightTrajectory],
             marker='.', color='r')
    plt.gca().set_yticks(np.arange(0, 30.1, 1.0), minor=True)
    plt.gca().set_yticks(np.arange(0, 30.1, 3))
    plt.gca().set_xticks(np.arange(0, 27.1, 1.0), minor=True)
    plt.gca().set_xticks(np.arange(0, 27.1, 3))
    plt.grid(which='minor', color='grey', linestyle='--', alpha=0.25)
    plt.grid(which='major', color='grey', linestyle='-', alpha=0.75)

    # Plot the heading data in degrees and look for any discontinuities
    plt.figure()
    plt.title("Heading")
    plt.plot(x, headings["left"], marker='.', color='b')
    plt.plot(x, headings["right"], marker='.', color='r')

    # Plot the velocity and acceleration and look for any discontinuities
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.title("Velocity")
    plt.plot(x, [segment.velocity for segment in leftTrajectory], marker='.', color='b')
    plt.plot(x, [segment.velocity for segment in rightTrajectory], marker='.', color='r')
    plt.yticks(np.arange(0, DRIVETRAIN_MAX_VELOCITY + 0.1, 1.0))
    plt.grid()
    plt.subplot(2, 1, 2)
    plt.title("Acceleration")
    plt.plot(x, [segment.acceleration for segment in leftTrajectory], marker='.', color='b')
    plt.plot(x, [segment.acceleration for segment in rightTrajectory], marker='.', color='r')
    plt.yticks(np.arange(-DRIVETRAIN_MAX_ACCELERATION, DRIVETRAIN_MAX_ACCELERATION + 1.1, 2.0))
    plt.grid()
    plt.tight_layout()
    plt.show()


def GenerateMotionProfile(motion_profile_name, file_name, trajectory, position_units, velocity_units):
    """
    This function will use the pathfinder to generate a single-axis motion profile.
    """
    # Grab the position, velocity, and duration
    path = []
    output = open(os.path.join(FILE_OUTPUT_PATH, file_name+".txt"), "w")
    output.write("position, velocity, acceration, dt\n")
    for i in range(len(trajectory)):
        path.append([trajectory[i].position * position_units,
                     trajectory[i].velocity * velocity_units,
                     0.0,  # No heading is used for single-axis
                     int(trajectory[i].dt * 1000)])

        output.write("%3.4f, %3.4f, %3.4f, %1.3f\n" %
                     (trajectory[i].position, trajectory[i].velocity,
                      trajectory[i].acceleration, trajectory[i].dt))

    output.close()

    # Dump the path into a pickle file which will be read up later by the RoboRIO robot code
    with open(os.path.join(motion_profile_name, file_name+".pickle"), "wb") as fp:
        pickle.dump(path, fp)

    # Plot the data for review
    plt.figure()
    plt.title("Trajectory(Native Units)")
    plt.plot([segment.y * position_units for segment in trajectory],
             [segment.x * position_units for segment in trajectory],
             marker='.', color='b')
    x = list(i * (trajectory[i].dt) for i, _ in enumerate(trajectory))

    # Plot the velocity and acceleration and look for any discontinuities
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.title("Velocity")
    plt.plot(x, [segment.velocity for segment in trajectory], marker='.', color='r',
             label='velocity')
    plt.grid()
    plt.subplot(2, 1, 2)
    plt.title("Acceleration")
    plt.plot(x, [segment.acceleration for segment in trajectory], marker='.', color='b',
             label='acceration')
    plt.grid()
    plt.tight_layout()
    plt.show()


#===============================================================================
# def PathAppender(first_path, second_path, reverse=False):
# 
#     if reverse:
#         for i in range(len(second_path['left'])):    
#             second_path['left'][i][0] = -second_path['left'][i][0]
#             second_path['left'][i][1] = -second_path['left'][i][1] 
#               
#         for i in range(len(second_path['right'])):    
#             second_path['right'][i][0] = -second_path['right'][i][0]    
#             second_path['right'][i][1] = -second_path['right'][i][1]    
#     
#     for i in range(len(first_path['left'])):    
#         lastPoint = first_path['left'][-1][0]      
#         lastHeading = first_path['left'][-1][2]
#      
#     for i in range(len(second_path['left'])):
#         firstPoint = second_path['left'][0][2]
#          
#     for i in range(len(second_path['left'])):    
#         second_path['left'][i][0] = second_path['left'][i][0] + lastPoint  
#         second_path['left'][i][2] = second_path['left'][i][2] - firstPoint + lastHeading
#          
#     for i in range(len(first_path['right'])):    
#         lastPoint = first_path['right'][-1][0]      
#         lastHeading = first_path['right'][-1][2]
#      
#     for i in range(len(second_path['right'])):
#         firstPoint = second_path['right'][0][2]
#          
#     for i in range(len(second_path['right'])):    
#         second_path['right'][i][0] = second_path['right'][i][0] + lastPoint  
#         second_path['right'][i][2] = second_path['right'][i][2] - firstPoint + lastHeading 
#          
#     for key in second_path:
#        for data in (second_path[key]):       
#            first_path[key].append(data)
#===============================================================================


def drawField(plt):
    # Add in the null zone lines
    plt.plot([0, Y_WALL_TO_NULL_ZONE],
             [X_WALL_TO_NULL_ZONE, X_WALL_TO_NULL_ZONE],
             linewidth=2, color='red')
    plt.plot([27, 27 - Y_WALL_TO_NULL_ZONE],
             [X_WALL_TO_NULL_ZONE, X_WALL_TO_NULL_ZONE],
             linewidth=2, color='red')

    # Draw the 10-foot autonomous line
    plt.plot([0, 27],
             [10, 10],
             linewidth=2, color='red')

    # Draw the exchange zone
    plt.plot([27 / 2 - 1, 27 / 2 - 1, 27 / 2 - 1 - 4, 27 / 2 - 1 - 4],
             [0,          3,          3,              0],
             linewidth=2, color='red')

    # Draw the platform
    plt.plot([Y_WALL_TO_NULL_ZONE,  Y_WALL_TO_NULL_ZONE, 27 - Y_WALL_TO_NULL_ZONE, 27 - Y_WALL_TO_NULL_ZONE],
             [X_WALL_TO_SCALE_NEAR, X_WALL_TO_PLATFORM,  X_WALL_TO_PLATFORM,       X_WALL_TO_SCALE_NEAR],
             linewidth=2, color='red')

    # Draw the wall
    plt.plot([Y_WALL_TO_START, 27 - Y_WALL_TO_START],
             [0,               0],
             linewidth=2, color='black')
    plt.plot([0,               0],
             [X_WALL_TO_START, 30],
             linewidth=2, color='black')
    plt.plot([27,              27],
             [X_WALL_TO_START, 30],
             linewidth=2, color='black')
    plt.plot([Y_WALL_TO_START, 0],
             [0,               X_WALL_TO_START],
             linewidth=2, color='black')
    plt.plot([27 - Y_WALL_TO_START, 27],
             [0,                    X_WALL_TO_START],
             linewidth=2, color='black')

    # Draw switch
    plt.plot([Y_WALL_TO_SWITCH_NEAR, Y_WALL_TO_SWITCH_NEAR, Y_WALL_TO_SWITCH_FAR, Y_WALL_TO_SWITCH_FAR,  Y_WALL_TO_SWITCH_NEAR],
             [X_WALL_TO_SWITCH_NEAR, X_WALL_TO_SWITCH_FAR,  X_WALL_TO_SWITCH_FAR, X_WALL_TO_SWITCH_NEAR, X_WALL_TO_SWITCH_NEAR],
             linewidth=2, color='black')

    # Draw the scale
    plt.plot([Y_WALL_TO_SCALE_NEAR, Y_WALL_TO_SCALE_NEAR,     Y_WALL_TO_SCALE_FAR,      Y_WALL_TO_SCALE_FAR,  Y_WALL_TO_SCALE_NEAR],
             [X_WALL_TO_SCALE_NEAR, X_WALL_TO_SCALE_NEAR + 4, X_WALL_TO_SCALE_NEAR + 4, X_WALL_TO_SCALE_NEAR, X_WALL_TO_SCALE_NEAR],
             linewidth=2, color='black')

    # Draw the switch cubes
    plt.plot([Y_WALL_TO_SWITCH_NEAR, Y_WALL_TO_SWITCH_NEAR,       Y_WALL_TO_SWITCH_NEAR + 1.33, Y_WALL_TO_SWITCH_NEAR + 1.33, Y_WALL_TO_SWITCH_NEAR],
             [X_WALL_TO_SWITCH_FAR,  X_WALL_TO_SWITCH_FAR + 1.33, X_WALL_TO_SWITCH_FAR + 1.33,  X_WALL_TO_SWITCH_FAR,         X_WALL_TO_SWITCH_FAR],
             linewidth=2, color='yellow')

    plt.plot([Y_WALL_TO_SWITCH_FAR, Y_WALL_TO_SWITCH_FAR,        Y_WALL_TO_SWITCH_FAR - 1.33, Y_WALL_TO_SWITCH_FAR - 1.33, Y_WALL_TO_SWITCH_FAR],
             [X_WALL_TO_SWITCH_FAR, X_WALL_TO_SWITCH_FAR + 1.33, X_WALL_TO_SWITCH_FAR + 1.33, X_WALL_TO_SWITCH_FAR,        X_WALL_TO_SWITCH_FAR],
             linewidth=2, color='yellow')

    # 4 cubes 5 gaps for Y_WALL_TO_SWITCH_FAR - Y_WALL_TO_SWITCH_NEAR - 2 cubes
    # gap width = 5 / (Y_WALL_TO_SWITCH_FAR - Y_WALL_TO_SWITCH_NEAR - 4 * 16 / 12
    gapWidth = 5 / (Y_WALL_TO_SWITCH_FAR - Y_WALL_TO_SWITCH_NEAR - 6 * 16 / 12)
    for i in range(1, 5, 1):
        startOffset = gapWidth * i + 16 / 12 * i
        plt.plot([Y_WALL_TO_SWITCH_NEAR + startOffset, Y_WALL_TO_SWITCH_NEAR + startOffset, Y_WALL_TO_SWITCH_NEAR + startOffset + 1.33, Y_WALL_TO_SWITCH_NEAR + startOffset + 1.33, Y_WALL_TO_SWITCH_NEAR + startOffset],
                 [X_WALL_TO_SWITCH_FAR,                X_WALL_TO_SWITCH_FAR + 1.33,         X_WALL_TO_SWITCH_FAR + 1.33,                X_WALL_TO_SWITCH_FAR,                       X_WALL_TO_SWITCH_FAR],
                 linewidth=2, color='yellow')

    # Draw the cube pile
    plt.plot([13.5 - 1.33 / 2,        13.5 + 1.33 / 2,      13.5 + 1.33 / 2,             13.5 - 1.33 / 2,             13.5 - 1.33 / 2],
             [X_WALL_TO_SWITCH_NEAR,  X_WALL_TO_SWITCH_NEAR, X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR],
             linewidth=2, color='yellow')
    plt.plot([13.5 - 1.33 - 1.33 / 2,        13.5 - 1.33 + 1.33 / 2,      13.5 - 1.33 + 1.33 / 2,             13.5 - 1.33 - 1.33 / 2,             13.5 - 1.33 - 1.33 / 2],
             [X_WALL_TO_SWITCH_NEAR,  X_WALL_TO_SWITCH_NEAR, X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR],
             linewidth=2, color='yellow')
    plt.plot([13.5 + 1.33 - 1.33 / 2,        13.5 + 1.33 + 1.33 / 2,      13.5 + 1.33 + 1.33 / 2,             13.5 + 1.33 - 1.33 / 2,             13.5 + 1.33 - 1.33 / 2],
             [X_WALL_TO_SWITCH_NEAR,  X_WALL_TO_SWITCH_NEAR, X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR],
             linewidth=2, color='yellow')
    plt.plot([13.5 - 1.33,            13.5,                  13.5,                         13.5 - 1.33,                  13.5 - 1.33],
             [X_WALL_TO_SWITCH_NEAR - 1.33,  X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33 - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33 - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33],
             linewidth=2, color='yellow')
    plt.plot([13.5,                   13.5 + 1.33,           13.5 + 1.33,                  13.5,                         13.5],
             [X_WALL_TO_SWITCH_NEAR - 1.33,  X_WALL_TO_SWITCH_NEAR - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33 - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33 - 1.33, X_WALL_TO_SWITCH_NEAR - 1.33],
             linewidth=2, color='yellow')
    plt.plot([13.5 - 1.33 / 2,        13.5 + 1.33 / 2,      13.5 + 1.33 / 2,             13.5 - 1.33 / 2,             13.5 - 1.33 / 2],
             [X_WALL_TO_SWITCH_NEAR - 2.66,  X_WALL_TO_SWITCH_NEAR - 2.66, X_WALL_TO_SWITCH_NEAR - 2.66 - 1.33, X_WALL_TO_SWITCH_NEAR - 2.66 - 1.33, X_WALL_TO_SWITCH_NEAR - 2.66],
             linewidth=2, color='yellow')
