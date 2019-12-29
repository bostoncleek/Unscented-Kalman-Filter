
"""
UKF implementation for differential drive robot with landmark sensor

"""

import numpy as np
from scipy import linalg

from motion import mobile_robot
from measure import sensor
from params import*


class UKF(object):

    def __init__(self):

        # sigma weights
        # first weight for mean
        self.wm = lamda/(n+lamda)
        #print("Weight 0 mean, ", self.wm)
        # first weight for covarince matrix
        self.wc = lamda/(n+lamda) + (1 - (alpha)**2 + beta)
        #print("Weight 0 cov, ", self.wc)
        # remaining weights are equal
        self.w = 1/(2*(n+lamda))
        #print("Weight, ", 12*self.w + self.wm + self.wc)

        # model noise
        # motion model noise
        self.R = np.array([[std_dx**2, 0, 0],
                            [0, std_dy**2, 0],
                            [0, 0, std_dtheta**2]])
        # self.R = np.array([[std_dx**2,         std_dx*std_dy,     std_dx*std_dtheta],
        #                    [std_dx*std_dy,     std_dy**2,         std_dy*std_dtheta],
        #                    [std_dx*std_dtheta, std_dy*std_dtheta, std_dtheta**2]])

        # sensor model noise
        self.Q = np.array([[std_r**2, 0],
                          [0, std_b**2]])

        # self.Q = np.array([[std_r**2,    std_r*std_b],
        #                    [std_r*std_b, std_b**2]])


    def wrap_two_pi(self, angle):
        """ wraps and angle between 0 and 2pi """

        num_rev = angle/(2*np.pi)
        rev_frac = num_rev - int(num_rev)
        angle = rev_frac*2*np.pi

        return angle

    def wrap_pi(self, angle):
        """ wraps angle pi to -pi """
        angle = self.wrap_two_pi(angle)

        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi

        return angle


    def compute_sigma_points(self, mu, cov_mat):
        """ computes sigma points

        Arg:
            mu (np.array): shape 3x1 contains averages x, y, and theta
            cov_mat (np.array): shape 3x3 covariance matrix for mu

        Returns:
            sigma_mat (np.array): shape 7x3 contains 7 sigma points, each row
            coressponds to the robots pose
        """

        # take square root of cov_mat
        sqrt_cov_mat = linalg.sqrtm(cov_mat)
        #sqrt_cov_mat = np.real(complex(sqrt_cov_mat))
        sqrt_cov_mat = sqrt_cov_mat.real


        if np.any(np.iscomplex(sqrt_cov_mat)):
            print("ERROR")
            print("The square root of covariance matrix has complex numbers")
            print("ERROR")
            print(sqrt_cov_mat)

        # first row in sigma_mat is mu
        sigma_mat = np.array([mu])

        #print(np.sqrt(n+lamda))

        # compute next three sigma points
        # where i is a row of the covariance matrix square root
        for i in range(0, n):
            sigma_pt = mu + np.sqrt(n+lamda)*sqrt_cov_mat[i,:]
            sigma_mat = np.append(sigma_mat, [sigma_pt], axis=0)

        # compute next three sigma points
        # the difference here is the subtraction of the covariance matrix square root
        for i in range(0, n):
            sigma_pt = mu - np.sqrt(n+lamda)*sqrt_cov_mat[i,:]   # subtract
            sigma_mat = np.append(sigma_mat, [sigma_pt], axis=0)

        return sigma_mat


    def propagate_sigma_points(self, sigma_mat, u, dt):
        """ passes sigma_points through the motion model

        Args:
            sigma_mat (np.array): shape 7x3 of sigma points based on robots pose
            u (np.array): shape 2x1 velocity and angular velocity
            dt (float): change in time between measurements

        Returns:
            sigma_mat_star (np.array): shape 7x3 of new points based on motion model
        """

        # array of propagated points
        sigma_prime = []

        # pass each row through motion model at a time
        for i in range(0, pts):
            # turn noise off based on PR Table 3.4
            sigma_new = mobile_robot(u, sigma_mat[0,:], dt)
            sigma_prime.append(sigma_new)

        sigma_mat_star = np.array(sigma_prime)
        return sigma_mat_star


    def predict_mean(self, sigma_mat_star):
        """ computes the predicted mean vector

        Args:
            sigma_mat_star (np.array): shape 7x3 containt the sigma points
            that were propagated through the motion model

        Returns:
            mu_bar (np.array): shape 3x1 array of means for pose
        """

        # init empty array
        mu_bar = np.array([0, 0, 0])

        for i in range(0, pts):
            # apply first weight for the mean
            if i == 0:
                w_m =  self.wm
            else:
                w_m = self.w

            # update the predicted mean
            mu_bar = mu_bar + w_m * sigma_mat_star[i,:]

        return mu_bar


    def predict_covariance(self, mu_bar, sigma_mat_star):
        """ computes the predicted covariance matrix for mean vector

        Args:
            mu_bar (np.array): shape 3x1 array of means for pose
            sigma_mat_star (np.array): shape 7x3 of new points based on motion model


        Returns:
            cov_mat_bar (np.array): shape 3x3 covarince matrix for vector of means mu_bar
        """

        # note: to multiply two 1D arrays in numpy the outer must be
        # np.array([[e1,e2,e3]]), need extra pair of brackets

        # init empty covariance bar matrix
        cov_mat_bar = np.zeros((n,n))

        # update based on contribution from each propagated sigma point
        for i in range(0, pts):
            # apply first weight for covariance
            if i == 0:
                w_c = self.wc
            else:
                w_c = self.w

            # difference between propagated sigma and mu bar
            delta1 = sigma_mat_star[i,:] - mu_bar
            delta1 = delta1[np.newaxis]  # 1x3
            delta2 = delta1.T   # 3x1
            #print("shape1: ", delta1.shape)
            #print("shape2: ", delta2.shape)
            #print("weight ", w_c)

            # add motion model noise here ---> PR stable 3.4
            cov_mat_bar = np.add(cov_mat_bar, w_c * np.dot(delta2, delta1))

        cov_mat_bar = np.add(cov_mat_bar, self.R)

        return cov_mat_bar


    def observation_sigma(self, sigma_mat_new, landmark):
        """ passes each sigma point through sensor model

        Args:
            sigma_mat_new (np.array): shape 7x3 contains the sigma points based
            on mu_bar

            landmark (np.array): shape 2x1 contains the global x and y position
            of a landmark

        Returns:
            obs_mat (np.array): 7x2 observation matrix each row contains a
            observed range and bearing corresponding to each sigma point
        """

        # init empty array for observations
        obs = []

        # pass each sigma point through sensor model
        for i in range (0, pts):
            meas = sensor(landmark, sigma_mat_new[i,:])
            obs.append(meas)

        obs_mat = np.array(obs)
        return obs_mat


    def predicted_observation(self, obs_mat):
        """ the resulting observation sigma points are used to compute the
        predicted observation

        Args:
            obs_mat (np.array): 7x2 observation matrix each row contains a
            observed range and bearing corresponding to each sigma point

        Returns:
            z_hat (np.array): 2x1 this is the predicted observation containing
            predicted rand and bearing
        """
        # init empty z_hat
        z_hat = np.array([0, 0])

        # update based on contribution from each row of the observation matrix
        for i in range(0, pts):
            # apply first weight for mean
            if i == 0:
                w_m = self.wm
            else:
                w_m = self.w

            z_hat = z_hat + w_m * obs_mat[i,:]


        # bound bearing between 0 -> pi and 0 -> -pi
        z_hat[1] = self.wrap_pi(z_hat[1])

        return z_hat


    def uncertainty(self, obs_mat, z_hat):
        """ computes the uncertainty in the measurement
        Args:
            obs_mat (np.array): 7x2 observation matrix each row contains a
            z_hat (np.array): 2x1 containing predicted rand and bearing

        Returns:
            uncert_mat (np.array): shape 2x2 uncertainty in measurement
        """

        # init empy uncertainty array ---> PR St
        uncert_mat = np.zeros((2,2))

        for i in range(0, pts):
            # apply first weight
            if i == 0:
                w_c = self.wc
            else:
                w_c = self.w

            # difference in observation and predicted observation
            delta1 = obs_mat[i,:] - z_hat
            delta1 = delta1[np.newaxis] # 1x2
            delta2 = delta1.T   #2x1

            # add measurement noise here ---> PR table 3.4
            uncert_mat = np.add(uncert_mat, w_c * np.dot(delta2, delta1))

        uncert_mat = uncert_mat + w_c*np.dot(delta2, delta1) + self.Q
        #uncert_mat = np.add(uncert_mat, self.Q)

        return uncert_mat


    def cross_covariance(self, sigma_mat_new, mu_bar, obs_mat, z_hat):
        """

        Args:
            sigma_mat_new (np.array): shape 7x3 contains the sigma points based
            on mu_bar

            mu_bar (np.array): shape 3x1 array of means for pose

            obs_mat (np.array): 7x2 observation matrix each row contains a
            observed range and bearing corresponding to each sigma point

            z_hat (np.array): 2x1 containing predicted rand and bearing

        Returns:
            cross_cov_mat (np.array): shape 3x2
        """

        # init empty cross covariance matrix
        cross_cov_mat = np.zeros((3,2))

        for i in range(0, pts):
            # apply first weight
            if i == 0:
                w_c = self.wc
            else:
                w_c = self.w

            # difference btw new sigma points and the predicted mean vector
            delta_states = sigma_mat_new[i,:] - mu_bar
            delta_states = delta_states[np.newaxis] # 1x3
            delta_states = delta_states.T # 3x1

            # difference btw observation and predicted observation
            delta_obs = obs_mat[i,:] - z_hat
            delta_obs = delta_obs[np.newaxis] # 1x2

            cross_cov_mat = cross_cov_mat + w_c * np.dot(delta_states, delta_obs)

        return cross_cov_mat


    def kalman_gain(self, cross_cov_mat, uncert_mat):
        """ computes the kalman gain

        Args:
            cross_cov_mat (np.array): shape 3x2
            uncert_mat (np.array): shape 2x2 uncertainty in measurement

        Returns:
            kal_gain (np.array): shape 3x2 kalman gain
        """

        # check if uncertainty matrix is invertible by checking the determinant
        if np.linalg.det(uncert_mat) == 0:
            print("ERROR")
            print("The uncertainty matrix in not invertible ")
            print("ERROR")

        # invert the uncertainty matrix
        uncert_inv = np.linalg.inv(uncert_mat)
        # kalman gain
        kal_gain = np.dot(cross_cov_mat, uncert_inv)

        return kal_gain


    def update_mean(self, mu_bar, kal_gain, z, z_hat):
        """ estimates the posterior by update the mean based on the kalman gain
        and difference in measurements and predicted observation

        Args:
            mu_bar (np.array): shape 3x1 array of means for pose
            kal_gain (np.array): shape 3x2 kalman gain
            z (np.array): shape 2x1 contains range and bearing of landmark
            z_hat (np.array): 2x1 containing predicted rand and bearing

        Returns:
            new_mean (np.array): shape 3x1 update mean of the robots pose
        """

        # difference btw measurements and predicted observation
        delta_meas = z - z_hat
        delta_meas = delta_meas[np.newaxis] # 1x2
        delta_meas = delta_meas.T # 2x1

        #print("mu shape", mu_bar.shape)
        #print("delta shape", delta_meas.shape)
        #influ = np.dot(kal_gain, delta_meas)
        #print("influ", influ.shape)

        new_mean = mu_bar + np.dot(kal_gain, delta_meas).T

        # remove extra brackets because it causes indexing error
        # in motion model
        new_mean = new_mean.ravel()

        return new_mean


    def update_covariance(self, cov_mat_bar, kal_gain, uncert_mat):
        """ estimates the posterior by updating the covariance matrix

        Args:
            cov_mat_bar (np.array): shape 3x3 covarince matrix for vector of means mu_bar
            kal_gain (np.array): shape 3x2 kalman gain
            uncert_mat (np.array): shape 2x2 uncertainty in measurement

        Returns:
            new_cov_mat (np.array): shape 3x3 update covariance matrix for
            mean of robots pose
        """
        # kalman gain times the uncertainty matrix
        #mat = np.dot(kal_gain, uncert_mat)
        # (3x2) * (2x3)
        new_cov_mat = cov_mat_bar - np.dot(np.dot(kal_gain, uncert_mat), kal_gain.T)
        #new_cov_mat = new_cov_mat.ravel()

        return new_cov_mat


    def unscented_kalman_filter(self, mu, cov_mat, u, meas, dt):
        """ updates the guassian of the states

        Args:
            mu (np.array): shape 3x1 contains averages x, y, and theta
            cov_mat (np.array): shape 3x3 covariance matrix for mu

            u (np.array): shape 2x1 control input linear and angular velocity
            meas (np.array): shape 4x1 contains:
                                                landmark global position x,
                                                landmark global position y,
                                                range (m),
                                                bearing (rad)
            dt (float): change in time (s)

        Returns:
            mu (np.array): shape 3x1 contains averages x, y, and theta
            cov_mat (np.array): shape 3x3 covariance matrix for mu

        """

        # This is when we want to consider the controls update
        # The alternative if when there are multiple measurements at the
        # same time step the controls are ignored. dt must be a real number
        # to propagate the controls and ge the next state.
        if dt != None:

            # sample sigma points ---> PR step 2
            sigma_mat = self.compute_sigma_points(mu, cov_mat)

            # pass sigma points through motion model ---> PR ste 3
            sigma_mat_star = self.propagate_sigma_points(sigma_mat, u, dt)

            ### compute predicted belief ###

            # determine mu bar ---> PR step 4
            mu_bar = self.predict_mean(sigma_mat_star)

            # determine covariance matrix bar ---> PR step 5
            cov_mat_bar = self.predict_covariance(mu_bar, sigma_mat_star)

            # if no new measurements then controls have been applied
            # and the algorithm terminates here
            if np.all(meas) == None:
                #print("No measurement included")
                return mu_bar, cov_mat_bar

        else:
            mu_bar = mu
            cov_mat_bar = cov_mat

        #print("Measurement included")

        # from the measurement array
        # global x and y position of landmark
        landmark = [meas[0], meas[1]]
        # range and bearing of landmark
        z = [meas[2], meas[3]]


        # sample new sigma points ---> PR step 6
        sigma_mat_new = self.compute_sigma_points(mu_bar, cov_mat_bar)

        # from +x-axis NOT counterclockwise like sensor
        # pass new sigma points through measurement model ---> PR step 7
        obs_mat = self.observation_sigma(sigma_mat_new, landmark)
        #print("bugger: ", obs_mat)

        # predicted observation --> PR step 8
        z_hat = self.predicted_observation(obs_mat)

        # predict uncertainty ---> PR step 9
        uncert_mat = self.uncertainty(obs_mat, z_hat)

        # compute cross-covariance between state and observation ---> PR step 10
        cross_cov_mat = self.cross_covariance(sigma_mat_new, mu_bar, obs_mat, z_hat)

        # kalman gain ---> PR step 11
        kal_gain = self.kalman_gain(cross_cov_mat, uncert_mat)

        ### compute desired belief ###

        # determine new mean ---> PR step 12
        new_mean = self.update_mean(mu_bar, kal_gain, z, z_hat)

        # detemine the new covariance matrix ---> PR step 13
        new_cov_mat = self.update_covariance(cov_mat_bar, kal_gain, uncert_mat)


        return new_mean, new_cov_mat












#
