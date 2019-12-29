"""
This file runs the required components for part B
"""
import localization
robot_local = localization.Localization()

def compare_sequence_controls(noise=False):
    """ compares the control sequence listed in question 2 between the
    motion model and the filter

    """
    robot_local.control_sequence()

def run_filter(iter):
    """ compares the result from running the UKF on the odometry and measurement
    data sets to dead reckoning and to the ground truth

    """
    robot_local.robot_localization(iter)


def filter_results():
    """ plots the filter results that were pre-ran """

    robot_local.plot_results()
    #robot_local.load_data()

def show_states():
    """ plots states of robot """

    robot_local.plot_states()
