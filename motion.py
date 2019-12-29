
"""
This file contains the motion model for a simple
differential drive mobile robot. In the robots frame
positive x is forward and positive y is to the left.
The control commands are forward velocity and angular
velocity. Positive angular velocity if considered counterclockwise.
"""

import numpy as np
from params import*


def mobile_robot(u, pose, dt, noise=False):
    """ updates pose of mobile robot with option to add noise

    Args:
        u (array): shape 2x1 velocity and angular velocity
        pose (array): shape 3x1 previous pose containing x, y, and theta

    Returns
        pose (array): shape 3x1 new pose containing x, y, and theta

    """

    v = u[0]
    w = u[1]

    x = pose[0]
    y = pose[1]
    theta = pose[2]

    # determine the change in pose
    dx = v*np.cos(theta)*dt
    dy = v*np.sin(theta)*dt
    dtheta = w*dt

    # wrap theta from 0 to 2pi
    theta = theta + dtheta

    num_rev = theta/(2*np.pi)
    rev_frac = num_rev - int(num_rev)
    theta = rev_frac*2*np.pi

    # wrap pi to -pi

    if theta > np.pi:
        theta -= 2*np.pi
    elif theta < -np.pi:
        theta += 2*np.pi


    if noise:
        n_dx = np.random.normal(0, std_dx**2)
        n_dy = np.random.normal(0, std_dy**2)
        n_dtheta = np.random.normal(0, std_dtheta**2)

        return [dx + x + n_dx, dy + y + n_dy, theta + n_dtheta]

    else:
        return [dx + x, dy + y, theta]



#
