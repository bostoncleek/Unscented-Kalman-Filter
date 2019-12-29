"""
This class runs the robot localization pipeline. Determines robot
pose using UKF
"""

import filter
from motion import mobile_robot
from utils import*

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Localization(object):

    def __init__(self):

        # seed mu with the initial odom ground truth position
        self.mu = np.array([1.29812900, 1.88315210, 2.82870000])
        # init covariance matrix with small numbers
        self.cov_mat = np.array([[.10, 0, 0],
                                 [0, .10, 0],
                                 [0, 0, .20]])

        # dictionary containing robot landmark know global positions
        self.lm_dict = {}
        # dictionary containing to mapping btw barcodes and subject number
        self.lm_barcodes = {}

        # matrix containing the odometry data
        self.odometry = np.zeros((95818, 3), dtype=float)
        # matrix containing the measurement data
        self.measurement = np.zeros((7720, 4), dtype=float)

        # store iteration when lm detected
        self.lm_iter = []
        self.lm_num = []

        # ground truth pose
        self.ground_truth = []

        # dead reckoning data
        self.dead_reck = []

        # pre-ran UKF localization pose data
        self.filter_data = []

        # number of odometry commands
        self.length = 95818

        # the current measurement number
        self.num_z = 0

        # landmark detected
        self.lm_detected = False
        # use curren measurement
        self.use_meas = False

        # current barcode
        self.barcode = None

        # count which odometry and measurement reading we see
        self.odom_ctr1 = 0
        self.odom_ctr2 = 1
        self.m_ctr = 0

        # contains position of robot when a measurement is taken
        self.robot_lm = [[], []]

        # number of landmark scene
        self.num_lm = 0

        # parameter to skip n measurements
        self.skip = 1

        # number of landmarks used in load_measurements
        self.lm_used = 0

        # sets dt to zero for multiple measurements
        self.dtzero = False

    def load_landmarks(self):
        """ loads landmark global positions """
        landmark_data(self.lm_dict)

    def load_barcodes(self):
        """ loads mapping btw barcodes and subjects """
        barcode_data(self.lm_barcodes)

    def load_measurements(self):
        """ contains all the measrements made by the robot """
        meas = []
        measurement_data(meas)
        self.measurement = np.array(meas)
        #print(self.measurement[7719,0])

    def load_odometry(self):
        """ contain all odometry commands send to robot """
        odom = []
        odometry_data(odom)
        self.odometry = np.array(odom)
        #print(self.odometry[0,:])

    def load_ground_truth(self):
        """ load ground truth data """
        self.ground_truth = ground_truth_data()
        #print(self.ground_truth)

    def load_dead_data(self):
        """ loads the Dead Reckoning data """
        self.dead_reck = dead_reck_data()
        #print(self.dead_reck)

    def load_filter_data(self):
        """ load the pose from the localization function bellow """
        self.filter_data = filter_data()

    def load_data(self):
        """ loads all the data """
        self.load_landmarks()
        self.load_barcodes()
        self.load_measurements()
        self.load_odometry()
        self.load_ground_truth()
        self.load_dead_data()
        self.load_filter_data()

    def plot_states(self):
        """ plot the state of the robot """
        self.load_data()

        # number of points
        num_g = 87676
        num_dr = len(self.dead_reck[0])
        num_f = len(self.filter_data[0])

        # delta t
        #dt_g = 0.03

        # end time
        tf = 1387.28

        # time arrays
        tvec_gt = np.linspace(0, tf, num_g)
        tvec_dr = np.linspace(0, tf, num_dr)
        tvec_f = np.linspace(0, tf, num_f)

        # plot x position
        plt.figure(dpi=110, facecolor='w')
        plt.plot(tvec_f, self.filter_data[0], 'blue', linewidth=2)
        plt.plot(tvec_dr, self.dead_reck[0],  'black', linewidth=2)
        plt.plot(tvec_gt, self.ground_truth[0], 'red', linewidth=2)
        plt.xlabel("Time (s)")
        plt.ylabel("Global X Position (m)")
        plt.title("Comparing the Global X Position of Robot")
        plt.legend(("Filter","Dead Reckoning", "Ground Truth"))
        plt.xlim(0,1315)
        plt.show()

        # plot y position
        plt.figure(dpi=110, facecolor='w')
        plt.plot(tvec_f, self.filter_data[1], 'blue', linewidth=2)
        plt.plot(tvec_dr, self.dead_reck[1],  'black', linewidth=2)
        plt.plot(tvec_gt, self.ground_truth[1], 'red', linewidth=2)
        plt.xlabel("Time (s)")
        plt.ylabel("Global Y Position (m)")
        plt.title("Comparing the Global Y Position of Robot")
        plt.legend(("Filter","Dead Reckoning", "Ground Truth"))
        plt.xlim(0,1315)
        plt.show()

        # plot theta
        plt.figure(dpi=110, facecolor='w')
        plt.plot(tvec_f, self.filter_data[2], 'blue', linewidth=2)
        plt.plot(tvec_dr, self.dead_reck[2],  'black', linewidth=2)
        plt.plot(tvec_gt, self.ground_truth[2], 'red', linewidth=2)
        plt.xlabel("Time (s)")
        plt.ylabel("Angular Position (rad)")
        plt.title("Comparing the Global Angular Position of Robot")
        plt.legend(("Filter","Dead Reckoning", "Ground Truth"))
        plt.xlim(0,1315)
        plt.show()


    def plot_results(self):
        """ plots dead reckoning, ground truth, and filtered data """
        self.load_data()

        plt.figure(dpi=110, facecolor='w')
        plt.plot(self.filter_data[0], self.filter_data[1], 'blue', linewidth=2)
        plt.plot(self.dead_reck[0], self.dead_reck[1], 'black', linewidth=2)
        plt.plot(self.ground_truth[0], self.ground_truth[1], 'red', linewidth=2)
        plt.xlabel("Global X Positon")
        plt.ylabel("Global Y Position")
        plt.title("Trajectory of Robot Given Control Sequence")
        plt.legend(("Filter","Dead Reckoning", "Ground Truth"))
        plt.show()

    def control_sequence(self):
        """ compare dead reckoning to UKF given a control sequence """
        # filter
        ukf = filter.UKF()

        # init covariance
        cov_mat = np.array([[0.004**2, 0, 0],
                            [0, 0.004*2, 0],
                            [0, 0, .0085**2]])

        # control inputs
        u = np.array([[0.5, 0.0, 0.5, 0.0, 0.5],
                    [0.0, -1/(2*np.pi), 0.0, 1/(2*np.pi), 0.0]])

        # each command applied for 1 second
        dt = 1

        # initialize pose
        pose_dr = np.array([0, 0, 0])
        pose_ukf = pose_dr

        # dead reckoning trajectory
        traj_dr = np.array([pose_dr])

        # ukf trajectory
        traj_ukf = np.array([pose_ukf])

        for cmd in range(u.shape[1]):
            # update pose
            pose_dr = mobile_robot(u[:,cmd], pose_dr, dt, True)
            traj_dr = np.append(traj_dr, [pose_dr], axis=0)

            pose_ukf, cov_mat = ukf.unscented_kalman_filter(pose_ukf, cov_mat, u[:,cmd], None, dt)
            traj_ukf = np.append(traj_ukf, [pose_ukf], axis=0)

        #print(cov_mat)

        plt.figure(dpi=110, facecolor='w')
        plt.plot(traj_dr[:,0], traj_dr[:,1], 'red', linewidth=2)
        plt.plot(traj_ukf[:,0], traj_ukf[:,1], 'blue', linewidth=1)
        plt.xlabel("Global X Positon")
        plt.ylabel("Global Y Position")
        plt.title("Trajectory of Robot")
        plt.legend(("Dead Reckoning", "UKF"))
        plt.show()


    def animate(self, i, trajectory, line1):
        #line1.set.xdata(trajectory[0][i])
        #line1.set.ydata(trajectory[1][i])
        line1.set_data(trajectory[0][i], trajectory[1][i])
        return line1


    def robot_localization(self, num_iter=None):
        """ This is the main loop for running the UKF for localization with the
        odometry and measurment data

        """

        #### IMPORTANT #####
        # LOAD DATA FIRST
        self.load_data()
        #### IMPORTANT #####

        # filter
        ukf = filter.UKF()

        # init time
        curr_time = 0

        # init gaussian
        mu = self.mu
        cov_mat = self.cov_mat

        # store trajectory: x, y, theta
        trajectory = [[self.mu[0]], [self.mu[1]], [self.mu[2]]]

        # write the trajectory to file
        file = open(filter_output, "w")
        file.write(str(self.mu[0]) + " " + str(self.mu[1]) + " " + str(self.mu[2]) + "\n")

        # current measurement
        z = None


        # how long to run localization algorithm?
        if num_iter == None:
            # run until the end
            iter = self.length-1
        else:
            iter = num_iter

        # start reading in odometry commands
        while(self.odom_ctr1 != iter):

            #print("------------------------ ")
            #print("Odom index: ", self.odom_ctr1)
            #print("Measurement index: ", self.m_ctr)
            #print("------------------------ ")

            # assume no lonadmarks scene yet
            self.lm_detected = False
            self.use_meas = False

            # get odometry time stamps
            time_stamp_odom = self.odometry[self.odom_ctr1, 0] # index based on odometry
            time_stamp_odom_next = self.odometry[self.odom_ctr2, 0] # index based on odometry

            # check for last measurements
            if self.m_ctr < 7720:
                time_stamp_meas = self.measurement[self.m_ctr, 0]    # index based on measurements

            else:
                # ensure no measurements considered after end of file
                time_stamp_meas = 0


            # controls from odometry
            u = [self.odometry[self.odom_ctr1, 1], self.odometry[self.odom_ctr1, 2]]

            #### Check for measurements ####

            # the current measurement is btw the current and next odom commands
            if time_stamp_odom <= time_stamp_meas <= time_stamp_odom_next:
                code = self.measurement[self.m_ctr, 1]

                # detected a landmark
                if code in self.lm_barcodes.keys():
                    self.barcode = code
                    self.lm_detected = True

                # detected another robot
                else:
                    # increment measurement count
                    self.m_ctr += 1
                    self.lm_detected = False


            else:
                # no measurements yet
                self.lm_detected = False
                #self.use_meas = False

                dt = time_stamp_odom - curr_time
                curr_time = time_stamp_odom

                mu, cov_mat = ukf.unscented_kalman_filter(mu, cov_mat, u, None, dt)

                # increment odom index
                self.odom_ctr1 += 1
                self.odom_ctr2 += 1


            # landmark detected
            if self.lm_detected == True:

                # first land mark detected then apply this measurement
                if self.m_ctr == 0:
                    self.use_meas = True

                # current landmark time step does NOT equal previous
                # landmark time step then apply measurement
                elif self.measurement[self.m_ctr, 0] != self.measurement[self.m_ctr-1, 0]:
                    self.use_meas = True

                # current landmark time step equals previous landmark time step
                # then dont apply measurement
                # or apply it as set dt = 0
                elif self.measurement[self.m_ctr, 0] == self.measurement[self.m_ctr-1, 0]:
                    self.use_meas = False
                    self.m_ctr += 1
                    #self.use_meas = True
                    #self.dtzero = True

            # landmark detected and can apply this measurement
            if self.use_meas == True:

                # skip every measurement defined by skip
                if self.num_lm % self.skip != 0:
                    self.m_ctr += 1
                    self.num_lm += 1

                else:

                    # update number of landmarks scene
                    self.num_lm += 1

                    #print("Line in data file: ", self.odom_ctr1+5)
                    #print("landmark detected (barcode): ", self.barcode)
                    #print("Measurement number: ", self.m_ctr)

                    # store location of robot when measurment is taken
                    self.robot_lm[0].append(mu[0])
                    self.robot_lm[1].append(mu[1])
                    self.lm_iter.append(self.odom_ctr1)

                    # map the subject (stored as barcode number) -> subject number
                    subject = self.lm_barcodes[self.barcode]
                    #print("subject number: ", subject)
                    self.lm_num.append(subject)

                    # adjust dt according to when measurement was recorded
                    if self.dtzero == True:
                        dt = None
                        self.dtzero = False

                    else:
                        dt = time_stamp_meas - curr_time
                        curr_time = time_stamp_meas

                    # get global position of subjects landmark
                    lm_pos = self.lm_dict[subject]

                    # get range and bearing measurements at current index
                    r = self.measurement[self.m_ctr, 2]
                    b = self.measurement[self.m_ctr, 3]

                    # store results in measurement vector z
                    z = np.array([lm_pos[0], lm_pos[1], r, b])

                    # update measurement index
                    self.m_ctr += 1
                    self.lm_used += 1

                    mu, cov_mat = ukf.unscented_kalman_filter(mu, cov_mat, u, z, dt)


            #print(cov_mat)
            #print(self.m_ctr)

            # update trajectory
            trajectory[0].append(mu[0])
            trajectory[1].append(mu[1])
            trajectory[2].append(mu[2])

            file.write(str(mu[0]) + " " + str(mu[1]) + " " + str(mu[2]) + "\n")

        #print("Number of landmarks scene: ", self.num_lm)
        #print("Number of landmarks used: ", self.lm_used)
        print("Last time stamp", self.odometry[self.odom_ctr1, 0])

        # decide how much data to plot
        if num_iter == None:
            # plot all data
            plt.figure(dpi=110, facecolor='w')
            plt.plot(trajectory[0], trajectory[1], 'blue', linewidth=2)
            plt.plot(self.dead_reck[0], self.dead_reck[1], 'black', linewidth=2)
            plt.plot(self.ground_truth[0], self.ground_truth[1], 'red', linewidth=2)
            plt.xlabel("Global X Positon")
            plt.ylabel("Global Y Position")
            plt.title("Trajectory of Robot")
            plt.legend(("UKF","Dead Reckoning", "Ground Truth"))
            plt.show()

        else:
            plt.figure(1, dpi=110, facecolor='w')
            plt.plot(trajectory[0], trajectory[1], 'blue', linewidth=2)
            plt.plot(self.dead_reck[0][0:iter], self.dead_reck[1][0:iter], 'black', linewidth=2)
            plt.plot(self.ground_truth[0][0:iter], self.ground_truth[1][0:iter], 'red', linewidth=2)
            #plt.scatter(self.robot_lm[0], self.robot_lm[1], c='green', alpha=1, s=15)
            plt.xlabel("Global X Positon")
            plt.ylabel("Global Y Position")
            plt.title("Trajectory of Robot")
            plt.legend(("UKF","Dead Reckoning", "Ground Truth", "Measurement"))
            plt.show()

            # fig, ax = plt.subplots(figsize=(5, 5))
            # ax.set(xlim=(0.5, 2.5), ylim=(-0.5, 3.0))
            #
            # #line1 = ax.plot(trajectory[0][0:iter], trajectory[1][0:iter], 'blue', linewidth=2)
            # #line1, = ax.plot([], [], 'blue', linewidth=2)
            # line1, = ax.plot([], [], lw=2)
            #
            # anim = FuncAnimation(fig, self.animate, frames=iter, fargs=(trajectory, line1),
            #                        interval=50, blit=True)
            #
            # plt.show()

            # plt.xlim((0.5, 2.5))
            # plt.ylim((-0.5, 3.0))
            # plt.pause(0.5)
            # print("Start recording")
            # plt.pause(0.5)
            #
            #
            # lm_indx = 0
            # for i in range(iter):
            #
            #
            #     plt.scatter(trajectory[0][i], trajectory[1][i], c='blue', alpha=1, s=15)
            #     #plt.plot(trajectory[0][0:i], trajectory[1][0:i], 'blue', linewidth=2)
            #     plt.scatter(self.dead_reck[0][i], self.dead_reck[1][i], c='black', alpha=1, s=15)
            #     #plt.plot(self.dead_reck[0][0:i], self.dead_reck[1][0:i], 'black', linewidth=2)
            #     plt.scatter(self.ground_truth[0][i], self.ground_truth[1][i], c='red', alpha=1, s=15)
            #     #plt.plot(self.ground_truth[0][0:i], self.ground_truth[1][0:i], 'red', linewidth=2)
            #
            #     if i % 100 == 0:
            #         print("Iteration", i)
            #
            #     # print(i)
            #     # #plt.scatter(1, 1, c='green', alpha=1, s=30)
            #     #
            #     # # hack to get lm to plot
            #     # plt.scatter(trajectory[0][0], trajectory[1][0], c='green', alpha=1, s=15)
            #     #
            #     # if i == self.lm_iter[lm_indx]:
            #     #     plt.scatter(self.robot_lm[0][lm_indx], self.robot_lm[1][lm_indx], c='green', alpha=1, s=15)
            #     #     plt.text(self.robot_lm[0][lm_indx]+0.01, self.robot_lm[1][lm_indx]+0.02, self.lm_num[lm_indx], fontsize=9)
            #     #
            #     #     print("lm found", self.lm_iter[lm_indx]," subject# ", self.lm_num[lm_indx])
            #     #     lm_indx += 1
            #
            #
            #     plt.xlabel("Global X Positon")
            #     plt.ylabel("Global Y Position")
            #     plt.title("Trajectory of Robot")
            #     plt.legend(("UKF","Dead Reckoning", "Ground Truth"))
            #     #plt.legend(("UKF","Dead Reckoning", "Ground Truth", "Landmark"))
            #
            #     plt.pause(0.00001)
            # print("Stop recording")
            #
            # plt.show()









#
