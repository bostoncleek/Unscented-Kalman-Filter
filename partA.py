"""
All files written in python 3

This file runs the required components for
part A. It provides the figures for questions
2 and 3, then prints the results to question
6 to the terminal
"""

from motion import mobile_robot
from measure import*
from params import*
from utils import landmark_data


import matplotlib.pyplot as plt
import numpy as np


def question_two(noise):
    # control inputs
    u = np.array([[0.5, 0.0, 0.5, 0.0, 0.5],
                [0.0, -1/(2*np.pi), 0.0, 1/(2*np.pi), 0.0]])

    # each command applied for 1 second
    dt = 1

    # initialize pose
    pose = np.array([0, 0, 0])

    # store trajectory
    # columns are x, y, theta
    traj = np.array([pose])

    for cmd in range(u.shape[1]):
        # update pose
        pose = mobile_robot(u[:,cmd], pose, dt, noise)
        traj = np.append(traj, [pose], axis=0)

    #print(traj[:,0])
    #print(traj[:,1])

    plt.figure(dpi=110, facecolor='w')
    plt.plot(traj[:,0], traj[:,1], 'red', linewidth=2)
    plt.xlabel("Global X Positon")
    plt.ylabel("Global Y Position")
    plt.title("Motion of Robot Given Control Sequence")
    #plt.legend("Motion Model")
    plt.show()


def question_three(odom_file, ground_file, noise):
    # controls
    v = []  # velocity
    w = []  # angular velocity
    t = []  # time stamps

    # ground truth position of robot
    x_true = []  # global x position
    y_true = []  # global y position
    theta_true = [] # orientation

    # open odometry file and store controls
    file_odom = open(odom_file, "r")
    for line in file_odom:
        if not line.startswith("#"):
            values = line.split()
            v.append(float(values[1]))
            w.append(float(values[2]))
            t.append(float(values[0]))

    file_odom.close()

    # open ground truth file
    file_ground = open(ground_file, "r")
    for line in file_ground:
        if not line.startswith("#"):
            values = line.split()
            x_true.append(float(values[1]))
            y_true.append(float(values[2]))
            theta_true.append(float(values[3]))

    file_odom.close()


    # initialize pose with ground truth
    pose = [x_true[0], y_true[0], theta_true[0]]

    # init time
    curr_time = 0

    # store trajectory
    # lists are x, y, theta
    traj = [[pose[0]], [pose[1]], [pose[2]]]

    # write odometry data to file
    #file = open(motion_model_odom, "w")
    #file.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + "\n")

    # apply odometry controls to motion model
    for cmd in range(len(v)):
        # delta t
        dt = t[cmd] - curr_time
        curr_time = t[cmd]

        # update pose
        pose = mobile_robot([v[cmd], w[cmd]], pose, dt, noise)
        traj[0].append(pose[0])
        traj[1].append(pose[1])
        traj[2].append(pose[2])

        # write to file
        #file.write(str(pose[0]) + " " + str(pose[1]) + " " + str(pose[2]) + "\n")

    #file.close()

    plt.figure(dpi=110, facecolor='w')
    #plt.plot(traj_ground[0][0:5000], traj_ground[1][0:5000], 'red', linewidth=2)
    #plt.plot(traj[0][0:5000], traj[1][0:5000], 'blue', linewidth=2)
    plt.plot(x_true, y_true, 'red', linewidth=2)
    plt.plot(traj[0], traj[1], 'black', linewidth=2)
    plt.xlabel("Global X Positon")
    plt.ylabel("Global Y Position")
    plt.title("Odometry and Ground Truth Motion Model")
    plt.legend(("Ground Truth","Dead Reckoning"))
    plt.show()


def question_six(noise):
    lm_dict = {}
    landmark_data(lm_dict)
    #print(lm_dict)

    # landmark subjects
    lm = [6, 13, 17]

    # robot position assume theta = 0 rad
    pose_rob = [[2, 3, 0], [0, 3, 0], [1, -2, 0]]
    #pose_rob = [[1, 1, -3.14], [0, 3, 0], [1, -2, 0]]


    # global position of landmark
    meas_lm = []

    print("-----------------------------")
    print("Landmark Position Estimation")
    print("-----------------------------")

    for i in range(len(lm)):
        # range and bearing measurement
        meas_rb = sensor(lm_dict[lm[i]], pose_rob[i], noise)
        # global position of landmark
        meas_xy = landmark_position(meas_rb, pose_rob[i])
        meas_lm.append(meas_xy)

        print("The range and bearing of landmark ", lm[i], ": ", "range: ", meas_rb[0], "(m)", " bearing: ", meas_rb[1], " (rad)")
        print("The global position of landmark ", lm[i], ": ", "x: ", meas_xy[0], " (m)", " y: ", meas_xy[1], " (m)")
        print("Error in x: ", lm_dict[lm[i]][0] - meas_xy[0], " (m)", " Error in y: ", lm_dict[lm[i]][1] - meas_xy[1], " (m)")
        print("----------------------------------------------------------------------------------------------")

    #print(lm_dict[lm[i]][0])
