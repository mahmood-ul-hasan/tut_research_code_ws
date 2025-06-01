import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from dataset import *
from util import *

from plot_fig import *

def plot_raw_data(title, imu, gt):
        gyro_data, acc_data, pos_data, ori_data = load_euroc_mav_dataset(imu, gt)

        # n1 = 8000+6000+500 +n
        n2 = 160000

        n1 = 0
        # n2 = len(gyro_data)

        plot_fig_gazebo("raw "+ str(title), gyro_data,acc_data, pos_data, ori_data, n1, n2)
        # plot_compare_pos("predict", "gt",  pos_data, ori_data, pos_data, ori_data)
        # plot_pos_error("predict error" ,  pos_data, ori_data, pos_data, ori_data)

        [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)

    # plot_fig("gt", x_gyro,x_acc, y_delta_p, y_delta_q)
        # plot_compare_pos("predict", "gt",  yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q)
        plot_pos_error_gazebo("predict error "+ str(title) ,  y_delta_p, y_delta_q, y_delta_p, y_delta_q, n1, n2)
        # plot_compare_imu("given data", "change in data",  acc_data, gyro_data, x_acc, x_gyro)
        # plot_fig("perdict", x_gyro,x_acc, yhat_delta_p, yhat_delta_q)
        gt_trajectory = generate_trajectory_6d_quat(init_p, init_q, y_delta_p, y_delta_q)



   

if __name__ == '__main__':
    
    class argument:
    
        dataset = "my_imu"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.002_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.002_gt_data_filter.csv'
    args = argument()

    window_size = 26
    stride = 26
    limit = 100*600*2

        
    # plot_raw_data("x",args.input, args.gt)



    class argument:
    
        dataset = "my_imu"
        input = '/mah/AI/imu_pose/4_8_two_motion_win_50_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/imu_pose/4_8_two_motion_win_50_dt_0.01_gt_data_filter.csv'
    args = argument()

    window_size = 26
    stride = 26
    limit = 100*600*2

        
    plot_raw_data("online", args.input, args.gt)

    
    
    
    plt.show()