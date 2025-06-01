import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from keras.models import load_model

from dataset import *
from util import *





def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('dataset', choices=['oxiod', 'euroc'], help='Training dataset name (\'oxiod\' or \'euroc\')')
    # parser.add_argument('model', help='Model path')
    # parser.add_argument('input', help='Input sequence path (e.g. \"Oxford Inertial Odometry Dataset/handheld/data4/syn/imu1.csv\" for OxIOD, \"MH_02_easy/mav0/imu0/data.csv\" for EuRoC)')
    # parser.add_argument('gt', help='Ground truth path (e.g. \"Oxford Inertial Odometry Dataset/handheld/data4/syn/vi1.csv\" for OxIOD, \"MH_02_easy/mav0/state_groundtruth_estimate0/data.csv\" for EuRoC)')
    # args = parser.parse_args()


    class argument:
        dataset = "euroc"
        model = "6dofio_euroc.hdf5"
        input = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/imu0/data.csv"
        gt = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv"

    args = argument()
    print(args.dataset)
    print(args.model)
    print(args.input)
    print(args.gt)




    window_size = 200
    stride = 10

    model = load_model(args.model)

    if args.dataset == 'oxiod':
        gyro_data, acc_data, pos_data, ori_data = load_oxiod_dataset(args.input, args.gt)
    elif args.dataset == 'euroc':
        gyro_data, acc_data, pos_data, ori_data = load_euroc_mav_dataset(args.input, args.gt)


    plt.figure
    plt.subplot(311);  plt.plot(pos_data[:,0])
    plt.subplot(312);  plt.plot(pos_data[:,1])
    plt.subplot(313);  plt.plot(pos_data[:,2])
 


    [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)

    if args.dataset == 'oxiod':
        [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro[0:200, :, :], x_acc[0:200, :, :]], batch_size=1, verbose=1)
    elif args.dataset == 'euroc':
        [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)

    gt_trajectory = generate_trajectory_6d_quat(init_p, init_q, y_delta_p, y_delta_q)
    pred_trajectory = generate_trajectory_6d_quat(init_p, init_q, yhat_delta_p, yhat_delta_q)

    if args.dataset == 'oxiod':
        gt_trajectory = gt_trajectory[0:200, :]

    matplotlib.rcParams.update({'font.size': 18})
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2])
    ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    min_x = np.minimum(np.amin(gt_trajectory[:, 0]), np.amin(pred_trajectory[:, 0]))
    min_y = np.minimum(np.amin(gt_trajectory[:, 1]), np.amin(pred_trajectory[:, 1]))
    min_z = np.minimum(np.amin(gt_trajectory[:, 2]), np.amin(pred_trajectory[:, 2]))
    max_x = np.maximum(np.amax(gt_trajectory[:, 0]), np.amax(pred_trajectory[:, 0]))
    max_y = np.maximum(np.amax(gt_trajectory[:, 1]), np.amax(pred_trajectory[:, 1]))
    max_z = np.maximum(np.amax(gt_trajectory[:, 2]), np.amax(pred_trajectory[:, 2]))
    range_x = np.absolute(max_x - min_x)
    range_y = np.absolute(max_y - min_y)
    range_z = np.absolute(max_z - min_z)
    max_range = np.maximum(np.maximum(range_x, range_y), range_z)
    ax.set_xlim(min_x, min_x + max_range)
    ax.set_ylim(min_y, min_y + max_range)
    ax.set_zlim(min_z, min_z + max_range)
    ax.legend(['ground truth', 'predicted'], loc='upper right')


    fig = plt.figure("Plot of x y z part of tracjectory"); plt.suptitle("Plot of x y z part of tracjectory")
    plt.subplot(311); plt.plot(gt_trajectory[:,0], label = "x"); plt.grid(True); plt.title("x")
    plt.subplot(312); plt.plot(gt_trajectory[:,1], label = "y"); plt.grid(True); plt.title("y")
    plt.subplot(313); plt.plot(gt_trajectory[:,2], label = "z"); plt.grid(True); plt.title("z")

    plt.subplot(311); plt.plot(pred_trajectory[:,0]); plt.grid(True);  
    plt.subplot(312); plt.plot(pred_trajectory[:,1]); plt.grid(True); 
    plt.subplot(313); plt.plot(pred_trajectory[:,2]); plt.grid(True); 
    plt.legend(['ground truth', 'predicted'])


    fig = plt.figure("Plot of x y z part of y_delta_p"); plt.suptitle("Plot of x y z part of tracjectory")
    plt.subplot(311); plt.plot(y_delta_p[:,0], label = "x"); plt.grid(True); plt.title("x")
    plt.subplot(312); plt.plot(y_delta_p[:,1], label = "y"); plt.grid(True); plt.title("y")
    plt.subplot(313); plt.plot(y_delta_p[:,2], label = "z"); plt.grid(True); plt.title("z")

    plt.subplot(311); plt.plot(yhat_delta_p[:,0], label = "x"); plt.grid(True); plt.title("x")
    plt.subplot(312); plt.plot(yhat_delta_p[:,1], label = "y"); plt.grid(True); plt.title("y")
    plt.subplot(313); plt.plot(yhat_delta_p[:,2], label = "z"); plt.grid(True); plt.title("z")


    plt.show()

if __name__ == '__main__':
    main()