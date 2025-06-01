import numpy as np
import matplotlib.pyplot as plt
import argparse
from train_crane_data import *

imu_data_filenames = []
gt_data_filenames = []
class argument:
    dataset = "my_imu" 
    output = "/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_model_simple_motion_ros_rate_all_win_100"
    imu_data_filenames.append('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_imu_simple_motion_ros_rate_1.csv')
    imu_data_filenames.append('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_imu_simple_motion_ros_rate_1.5.csv')
    imu_data_filenames.append('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_imu_simple_motion_ros_rate_2.csv')


    imu_data_filenames = imu_data_filenames


    gt_data_filenames.append('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_gt_simple_motion_ros_rate_1.csv')
    gt_data_filenames.append('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_gt_simple_motion_ros_rate_1.5.csv')
    gt_data_filenames.append('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_gt_simple_motion_ros_rate_2.csv')

    gt_data_filenames = gt_data_filenames
   
   
    window_size = 100
    stride = 10
    epoch = 500

args = argument()


history = train_crane_data(args)
model_loss = [history.history['loss'], history.history['val_loss']]
model_loss =  np.array(model_loss)
model_loss = np.transpose(model_loss)
np.savetxt('/mah/AI/6-DOF-Inertial-Odometry_orignal/crane simple motion data/4_15_data_model_loss_simple_motion_ros_rate_all_win_100', model_loss, delimiter=',', fmt = '%.9f')





plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['Train', 'Validation'], loc='upper left')
plt.show()