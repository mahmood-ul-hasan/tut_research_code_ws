#!/usr/bin/env python3
# license removed for brevity
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from tensorflow.keras.models import load_model

from dataset import *
from util import *
from plot_fig import *

import csv
import numpy as np


def main():
    # parser = argparse.ArgumentParser()
    # #parser.add_argument('dataset', choices=['oxiod', 'euroc', 'my_imu'], help='Training dataset name (\'oxiod\' or \'euroc\' or \'my_imu\')')
    # parser.add_argument('dataset', choices=['oxiod', 'euroc', 'my_imu'], help='Training dataset name (\'oxiod\' or \'euroc\' or \'my_imu\')')
    # parser.add_argument('model', help='Model path')
    # parser.add_argument('input', help='Input sequence path (e.g. \"Oxford Inertial Odometry Dataset/handheld/data4/syn/imu1.csv\" for OxIOD, \"MH_02_easy/mav0/imu0/data.csv\" for EuRoC, "/mah/AI/6-DOF-Inertial-Odometry/6-DOF-Inertial-Odometry-master/data_imu_accel_rotate.csv" for my_imu)')
    # parser.add_argument('gt', help='Ground truth path (e.g. \"Oxford Inertial Odometry Dataset/handheld/data4/syn/vi1.csv\" for OxIOD, \"MH_02_easy/mav0/state_groundtruth_estimate0/data.csv\" for EuRoC, "/mah/AI/6-DOF-Inertial-Odometry/6-DOF-Inertial-Odometry-master/data_gt_accel_rotate.csv" for my_imu)')
    # args = parser.parse_args()

    class argument:
        # dataset = "euroc"
        # model = "6dofio_euroc.hdf5"
        # input = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/imu0/data.csv"
        # gt = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv"

        # dataset = "my_imu"
        # model = "/mah/AI/my_6-DOF-Inertial-Odometry/my_imu_accel_rotate_absolute.hdf5"
        # input = "/mah/AI/my_6-DOF-Inertial-Odometry/data_imu_accel_rotate2.csv"
        # gt = "/mah/AI/my_6-DOF-Inertial-Odometry/data_gt_accel_rotate2.csv"

        dataset = "my_imu"
        model = "/media/aisl2/aisl_data/AI_imu/my_6-DOF-Inertial-Odometry/my_imu_accel_rotate_absolute.hdf5"
        input = "/media/aisl2/aisl_data/AI_imu/my_6-DOF-Inertial-Odometry/random_1_imu_data.csv"
        gt = "/media/aisl2/aisl_data/AI_imu/my_6-DOF-Inertial-Odometry/random_1_gt.csv"

    args = argument()

    window_size = 200
    stride = 10

    model = load_model(args.model, compile=False)

    if args.dataset == 'oxiod':
        gyro_data, acc_data, pos_data, ori_data = load_oxiod_dataset(args.input, args.gt)
    elif args.dataset == 'euroc' or args.dataset == 'my_imu':
        gyro_data, acc_data, pos_data, ori_data, time_stamp = load_euroc_mav_dataset(args.input, args.gt)

    # plot_fig("gt",gyro_data,acc_data, pos_data, ori_data)

    [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)

    plot_fig("gt", x_gyro, x_acc, y_delta_p, y_delta_q)

    [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = modified_load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)

    plot_fig("modified", x_gyro, x_acc, y_delta_p, y_delta_q)

    # plt.show()

    print("init_p", init_p)
    print("pos_data", np.shape(pos_data))
    print("y_delta_p", np.shape(y_delta_p))
    print("ori_data", np.shape(ori_data))
    print("y_delta_q", np.shape(y_delta_q))
    print("gyro_data", np.shape(gyro_data))
    print("x_gyro", np.shape(x_gyro))
    print("acc_data", np.shape(acc_data))
    print("x_acc", np.shape(x_acc))

    # #plot_compare("compare", x_gyro, x_acc, gyro_data, acc_data)
    # print("===============================================================")
    # plot_compare_pos("given data", "change in data",  pos_data, ori_data, y_delta_p, y_delta_q)
    # print("===============================================================")
    # plot_compare_imu("given data", "change in data",  acc_data, gyro_data, x_acc, x_gyro)

    x_gyro_min = np.amin(x_gyro, axis=0)
    x_gyro_max = np.amax(x_gyro, axis=0)
    x_gyro = (x_gyro - x_gyro_min) / (x_gyro_max - x_gyro_min)

    x_acc_min = np.amin(x_acc, axis=0)
    x_acc_max = np.amax(x_acc, axis=0)
    x_acc = (x_acc - x_acc_min) / (x_acc_max - x_acc_min)

    if args.dataset == 'oxiod':
        [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro[0:200, :, :], x_acc[0:200, :, :]], batch_size=1, verbose=1)
    elif args.dataset == 'euroc' or args.dataset == 'my_imu':
        [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)

        print(yhat_delta_p)

    #plot_fig("predict", x_gyro,x_acc, yhat_delta_p, yhat_delta_q)

    plot_compare_pos("predict", "gt", yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q)
    plot_pos_error("predict error", yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q)

    gt_trajectory = generate_trajectory_6d_quat(init_p, init_q, y_delta_p, y_delta_q)
    # pred_trajectory = generate_trajectory_6d_quat(init_p, init_q, yhat_delta_p, yhat_delta_q)
    pred_trajectory, pred_quat = my_generate_trajectory_6d_quat(init_p, init_q, yhat_delta_p, yhat_delta_q)

    if args.dataset == 'oxiod':
        gt_trajectory = gt_trajectory[0:200, :]

    print("pred_trajectory", np.shape(pred_trajectory))
    print("pred_quat", np.shape(pred_quat))

# save test data in csv
# =======================================
    # Define the CSV file path
    csv_file_path = "odometry_data.csv"

    # Combine the position and quaternion data horizontally
    combined_data = np.hstack((pred_trajectory, pred_quat))

    # Write the data to a CSV file
    with open(csv_file_path, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write a header row (optional)
        csvwriter.writerow(['pos_x', 'pos_y', 'pos_z', 'quat_w', 'quat_x', 'quat_y', 'quat_z'])
        # Write the data rows
        csvwriter.writerows(combined_data)

    matplotlib.rcParams.update({'font.size': 18})
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2])
    ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    # min_x = np.minimum(np.amin(gt_trajectory[:, 0]), np.amin(pred_trajectory[:, 0]))
    # min_y = np.minimum(np.amin(gt_trajectory[:, 1]), np.amin(pred_trajectory[:, 1]))
    # min_z = np.minimum(np.amin(gt_trajectory[:, 2]), np.amin(pred_trajectory[:, 2]))
    # max_x = np.maximum(np.amax(gt_trajectory[:, 0]), np.amax(pred_trajectory[:, 0]))
    # max_y = np.maximum(np.amax(gt_trajectory[:, 1]), np.amax(pred_trajectory[:, 1]))
    # max_z = np.maximum(np.amax(gt_trajectory[:, 2]), np.amax(pred_trajectory[:, 2]))
    # range_x = np.absolute(max_x - min_x)
    # range_y = np.absolute(max_y - min_y)
    # range_z = np.absolute(max_z - min_z)
    # max_range = np.maximum(np.maximum(range_x, range_y), range_z)
    # ax.set_xlim(min_x, min_x + max_range)
    # ax.set_ylim(min_y, min_y + max_range)
    # ax.set_zlim(min_z, min_z + max_range)
    ax.legend(['ground truth', 'predicted'], loc='upper right')

    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('gt_trajectory Plot')

    plt.show()


if __name__ == '__main__':
    main()
