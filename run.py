
"""
All files written in python 3. See ReadMe.txt for help.

This file runs the required components for
part A by calling partA.py and the required components for part B by calling
partB.py. It provides the figures for questions
2 and 3, then prints the results to question
6 to the terminalself.

"""

from partA import*
from partB import*


if __name__ == "__main__":

    #### PART B ####

    # compare the control sequence between dead reckoning and UKF
    # from question 8
    #compare_sequence_controls()

    # Specify how long to run the filter by setting iter.
    # The nubmer of iteration corresponds to the number of lines
    # in ds0_Odometry.dat: 0 -> 95818.
    # Set iter = None and it will run until the end
    # Takes aprrox. 40s to run the entire data set on i7 32GB RAM
    #iter = None
    iter = 5000
    run_filter(iter)

    # If you do not want to wait for the full data set to run
    # uncomment the line bellow to plot the results from the full filter
    # that were saved to filter_output.txt
    #filter_results()


    # plot the states x,y,theta of the robot versus time
    #show_states()

    #### PART A ####
    #
    # # Want to add noise to motion and sensor model?
    # noise = True
    #
    # # plot for question 2
    # question_two(noise)
    #
    # # plot for question 3 may take ~5 seconds
    # # arguements are the file paths defined in params.py
    # question_three(odom_path, ground_truth, noise)
    #
    # # print results for quesion 6
    # question_six(noise)



















#
