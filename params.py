
#### data file paths from UTIAS ####
odom_path = "ds0/ds0_Odometry.dat"
ground_truth = "ds0/ds0_Groundtruth.dat"
landmark_truth = "ds0/ds0_Landmark_Groundtruth.dat"
measure_data = "ds0/ds0_Measurement.dat"
barcodes_file = "ds0/ds0_Barcodes.dat"

#### data file paths I wrote to #####
motion_model_odom = "ds0/motion_model_odom.txt"
filter_output = "ds0/filter_output.txt"

#### UKF parameters ####
n = 3   # state dimension
pts = 2*n+1 # number of sigma points

beta = 2
alpha = 10**-5
lamda = (alpha**2)*n-n

#### motion model noise ####
# standard deviation in pose
std_dx = 0.004 # mm
std_dy = 0.004 # mm
std_dtheta = 0.0085 # 0.0085 rad ~ 0.5 deg
# std_dx = 4e-3 # mm
# std_dy = 4e-3 # mm
# std_dtheta = 8.5e-2 # rad

#### sensor model noise ####
# standard deviation in range and bearing
std_r = 0.002 # 4 mm
std_b  = 0.0085 # 0.0085 rad ~ 0.5 deg
# std_r = 4e-2 #  mm
# std_b  = 8.5e-3 # rad


#
