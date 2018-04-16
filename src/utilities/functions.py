import pickle
import os.path
from constants import *
import matplotlib.pyplot as plt
import numpy as np
import pathfinder as pf


def CalculateFeedForwardVoltage(leftSide, velocity, acceleration):
    """
    This function will take the velocity and acceration from the pathfinder generated trajectory
    and output an applied voltage.  The applied voltage can then be used as a feed-forward to the
    motion profile controller.  This is based off of prior work fround here =>
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


def GeneratePath(path_name, file_name, waypoints):
    """
    This function will take a set of pathfinder waypoints and create the trajectories to follow
    a path going through the waypoints.
    """
    # Generate the path
    info, trajectory = pf.generate(waypoints, pf.FIT_HERMITE_QUINTIC, 1000000,
                                   PF_SAMPLE_PERIOD_MS / 1000, PF_MAX_VELOCITY,
                                   PF_MAX_ACCELERATION, PF_MAX_JERK)

    print(info)

    # Modify the path for the differential drive
    modifier = pf.modifiers.TankModifier(trajectory).modify(ROBOT_WHEELBASE_FT)

    # Ge the left and right trajectories
    leftTrajectory = modifier.getLeftTrajectory()
    rightTrajectory = modifier.getRightTrajectory()

    # Grab the position, velocity + acceleration for feed-forward, heading, and duration
    path = {"left": [], "right": []}
    output = open(os.path.join(FILE_OUTPUT_PATH, file_name+".txt"), "w")
    output.write("x, y, pos, vel, acc, heading, "
                 "L_x, L_y, L_pos, L_vel, L_acc, L_heading, "
                 "R_x, R_y, R_pos, R_vel, R_acc, R_heading\n")
    for i in range(len(leftTrajectory)):
        path["left"].append([leftTrajectory[i].position,
                             CalculateFeedForwardVoltage(True,
                                                         leftTrajectory[i].velocity,
                                                         leftTrajectory[i].acceleration),
                             pf.r2d(leftTrajectory[i].heading),
                             leftTrajectory[i].dt])
        path["right"].append([rightTrajectory[i].position,
                              CalculateFeedForwardVoltage(False,
                                                          rightTrajectory[i].velocity,
                                                          rightTrajectory[i].acceleration),
                              pf.r2d(rightTrajectory[i].heading),
                              rightTrajectory[i].dt])

        # It's easier to see the path when plotting the X data as the Y-axis and the Y data as the
        # X-axis in openoffice.
        output.write("%2.4f, %2.4f, %3.4f, %3.4f, %3.4f, %3.4f, "
                     "%2.4f, %2.4f, %3.4f, %3.4f, %3.4f, %3.4f, "
                     "%2.4f, %2.4f, %3.4f, %3.4f, %3.4f, %3.4f\n" %
                     (trajectory[i].x, trajectory[i].y,
                      trajectory[i].position, trajectory[i].velocity,
                      trajectory[i].acceleration, pf.r2d(trajectory[i].heading),
                      leftTrajectory[i].x, leftTrajectory[i].y,
                      leftTrajectory[i].position, leftTrajectory[i].velocity,
                      leftTrajectory[i].acceleration, pf.r2d(leftTrajectory[i].heading),
                      rightTrajectory[i].x, rightTrajectory[i].y,
                      rightTrajectory[i].position, rightTrajectory[i].velocity,
                      rightTrajectory[i].acceleration, pf.r2d(rightTrajectory[i].heading)))
    output.close()

    # Dump the path into a pickle file which will be read up later by the RoboRIO robot code
    with open(os.path.join(path_name, file_name+".pickle"), "wb") as fp:
        pickle.dump(path, fp)

    # Plot the data for review
    x = list(i * (PF_SAMPLE_PERIOD_MS / 1000) for i, _ in enumerate(leftTrajectory))

    plt.figure()
    # plt.plot(aspect=0.5)
    plt.title("Trajectory")
    drawField(plt)
    plt.plot([segment.y for segment in leftTrajectory],
             [segment.x for segment in leftTrajectory],
             marker='.', color='b')
    plt.plot([segment.y for segment in rightTrajectory],
             [segment.x for segment in rightTrajectory],
             marker='.', color='r')
    plt.gca().set_yticks(np.arange(0, 30.1, 1.0), minor=True)
    plt.gca().set_yticks(np.arange(0, 30.1, 3))
    plt.gca().set_xticks(np.arange(0, 27.1, 1.0), minor=True)
    plt.gca().set_xticks(np.arange(0, 27.1, 3))
    plt.grid(which='minor', color='grey', linestyle='--', alpha=0.25)
    plt.grid(which='major', color='grey', linestyle='-', alpha=0.75)

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


def GenerateMotionProfile(motion_profile_name, file_name, trajectory,
                          position_units, velocity_units):
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
                     trajectory[i].dt])

        output.write("%3.4f, %3.4f, %3.4f, %1.3f\n" %
                     (trajectory[i].position, trajectory[i].velocity,
                      trajectory[i].acceleration, trajectory[i].dt))

    output.close()

    # Dump the path into a pickle file which will be read up later by the RoboRIO robot code
    with open(os.path.join(motion_profile_name, file_name+".pickle"), "wb") as fp:
        pickle.dump(path, fp)

    # Plot the data for review
    x = list(i * (trajectory[i].dt) for i, _ in enumerate(trajectory))
    plt.figure()
    plt.title("Velocity")
    plt.plot(x, [segment.velocity for segment in trajectory],
             marker='.', color='r', label='velocity')
    plt.plot(x, [segment.acceleration for segment in trajectory],
             marker='.', color='b', label='acceration')
    plt.tight_layout()
    plt.show()


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
