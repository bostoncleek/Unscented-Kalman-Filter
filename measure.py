
"""
This file contains the sensor model that estimates the
range and bearing of landmarks and other robots
"""

from params import*
import numpy as np

def sensor(landmark, pose, noise=False):
    """ computes the expected range and bearing given a landmark and the robot pose
    with the option to add noise

    Args:
        landmark (array): global x and y position of a landmark
        pose (array): x, y, theta of robot

    Returns
     (array): range and bearing of landmark
    """

    # expected values
    range = np.sqrt((pose[0]-landmark[0])**2 + (pose[1]-landmark[1])**2)
    bearing = np.arctan2(landmark[1]-pose[1], landmark[0]-pose[0]) - pose[2]

    #### IMPORTANT ####
    # BUG:
    # wrap angle netween 0 to pi and 0 to -pi
    if bearing > np.pi:
        bearing -= 2*np.pi
    elif bearing < -np.pi:
        bearing += 2*np.pi


    #print("SENSOR: ",range, " ",bearing)

    if noise:
        n_r = np.random.normal(0, std_r**2)
        n_b = np.random.normal(0, std_b**2)

        return [range + n_r, bearing + n_b]

    else:
        return [range, bearing]


def landmark_position(meas_lm, pose):
    """ determines global position of landmark given range, bearing, and robots pose """

    # inputs:
    # meas_lm: range, bearing
    # pose: x, y, and theta

    # outputs:
    # global x and y of landmark

    r = meas_lm[0]
    b = meas_lm[1]

    x_lm = r*np.cos(b) + pose[0]
    y_lm = r*np.sin(b) + pose[1]

    return [x_lm, y_lm]
