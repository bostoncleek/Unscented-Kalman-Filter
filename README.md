# Unscented Kalman Filter

<p align="center">
  <img src="/media/ukflm.jpg" width="400" height="400"/>
</p>

# Overview
This project focuses on implementing a UFK using the [UTIAS Multi-Robot Cooperative Localization and Mapping Dataset](http://asrl.utias.utoronto.ca/datasets/mrclam/) from the Autonomous Space Robotics Lab at the University of Toronto.

The plot at the top compares the results of the UKF to dead reckoning and the ground truth. When a landmark is observed (represented as a green dot) the trajectory approximated using the UKF tends toward the ground truth path. Localization algorithms such as the UKF are used for SLAM in autonomous vehicle applications such as driver-less cars and extraterrestrial exploration robots.

See this [document](https://github.com/bostoncleek/Unscented-Kalman-Filter/blob/master/doc/ukf.pdf). for more details and theory behind the implementation.

# How to run
Simply run:
```
python3 run.py
```

The file run.py displays results to questions: 2,3,6,8, and 9. The file run.py will first display the two plots for question 8. You have the option to specify how long to run the full trajectory. It took 40s on my computer to run the trajectory. After the plots for part B run the plots for Part A will run. You must close a plot for the next one to appear.

The plot to question 2 will appear then after it is closed
the plot to question 3 will appear. Finally after closure of the last plot
the range and bearing results to 6 will print to the terminal. The error in
the global position of landmarks will be non zero if noise is add.

# Dependencies:
- numpy
- matplotlib
- scipy

Notes:
Change the data file input paths at the top of params.py to
specify a different file path. Specify the noise to add to the sensor and motion models in params.py

# Implementation
## Alogrithm
The UKF is accomplished by representing the belief as a Gaussian and extracting sigma points from it. The sigma points are located at the mean and along the axis of the covariance matrix. The equation for the sigma points is shown in line 2 bellow. The sigma points are passed through the nonlinear state transition function.

After a Gaussian is passed through a nonlinear function it is no longer a Gaussian. However, the predicted Gaussian is recovered by applying weights to each of the sigma points.  Each sigma point has two weights. The predicted mean and covariance are shown in lines 4 and 5 respectively. They key point is the UKF avoids linearizing around the mean using a Taylor series like the Extended Kalman Filter.

Note the motion model noise covariance matrix is applied to the predicted covariance matrix in line 5 and the measurement model noise covariance matrix is applied in line 9 to the uncertainty matrix.

Alogrithm from Probabilistic Robotics by Sebastian Thrun.

<p align="center">
  <img src="/media/ukfalgo.jpg" width="250" height="350"/>
</p>

The sigma points are sampled again after the Gaussian is recovered in lines 4 and 5. The new sigma points are passed through the measurement model in line 7. The cross-covariance matrix in line 10 determines the relationship between state and observation. The Kalman gain is computed in line 11 based on the cross-covariance and the uncertainty matrix. In line 12 the difference between the actual measurement and the predicted observation is used to determine the affect the Kalman gain will have upon updating the mean. The Kalman gain and the uncertainty matrix are used to update the covariance matrix in line 13.

## Motion Model
The robot is modeled as a kinematic unicycle, where the controls (u = [u1 u2]) are the linear and angular velocities. In the frame of the robot, the positive x-direction is forward and the positive y-direction is left. Positive angular velocity and angular position is considered counter clockwise.

## Measurement Model
The measurement model outputs the range and bearing of an observed landmark relative to the robot. The range is defined as the distance from the robot to the landmark. The bearing is angular position of the landmark relative to the robots x-axis frame. 
