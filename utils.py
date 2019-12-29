"""
utility functions
"""

from params import*

def landmark_data(lm_dict):
    """ creates a dictionary of landmark ground truth data """
    file = open(landmark_truth, "r")
    for line in file:
        if not line.startswith("#"):
            values = line.split()
            # subject number, global x, globaly
            lm_dict.update({float(values[0]) : [float(values[1]), float(values[2])]})

    file.close()

def barcode_data(barcodes_dict):
    """ creates a dictionary mapping barcodes to subject numbers """
    file = open(barcodes_file, "r")
    for line in file:
        if not line.startswith("#"):
            values = line.split()

            key = int(values[1])
            subject = int(values[0])
            # landmarks have numbers 6 -> 20
            if subject >= 6:
                # key is the barcode number
                # element if the subject number
                barcodes_dict.update({key : subject})

    file.close()

def measurement_data(measurement_mat):
    """ creates a matrix for the measurements made by the robot """
    file = open(measure_data, "r")
    for line in file:
        if not line.startswith("#"):
            values = line.split()
            meas = [float(values[0]), int(values[1]), float(values[2]), float(values[3])]
            measurement_mat.append(meas)
    file.close()

def odometry_data(odometry_mat):
    """ creates a matrix for the odometry data """
    file = open(odom_path, "r")
    for line in file:
        if not line.startswith("#"):
            values = line.split()
            odom = [float(values[0]), float(values[1]), float(values[2])]
            odometry_mat.append(odom)
    file.close()

def ground_truth_data():
    """ creates a matrix of ground truth robot pose """
    x_true = []  # global x position
    y_true = []  # global y position
    theta_true = [] # orientation

    file_ground = open(ground_truth, "r")
    for line in file_ground:
        if not line.startswith("#"):
            values = line.split()
            x_true.append(float(values[1]))
            y_true.append(float(values[2]))
            theta_true.append(float(values[3]))
    ground = [x_true, y_true, theta_true]

    file_ground.close()
    return ground


def dead_reck_data():
    """ loads the Dead Reckoning data """
    x = []
    y = []
    theta = []

    file = open(motion_model_odom, "r")
    for line in file:
        values = line.split()
        x.append(float(values[0]))
        y.append(float(values[1]))
        theta.append(float(values[2]))
    dead_reck = [x, y, theta]

    file.close()
    return dead_reck

def filter_data():
    """ loads the pose for UKF """
    x = []
    y = []
    theta = []

    file = open(filter_output, "r")
    for line in file:
        values = line.split()
        x.append(float(values[0]))
        y.append(float(values[1]))
        theta.append(float(values[2]))
    filter = [x, y, theta]

    file.close()
    return filter














#
