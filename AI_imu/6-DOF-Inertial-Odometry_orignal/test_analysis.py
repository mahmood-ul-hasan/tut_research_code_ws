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




def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('dataset', choices=['oxiod', 'euroc'], help='Training dataset name (\'oxiod\' or \'euroc\')')
    # parser.add_argument('model', help='Model path')
    # parser.add_argument('input', help='Input sequence path (e.g. \"Oxford Inertial Odometry Dataset/handheld/data4/syn/imu1.csv\" for OxIOD, \"MH_02_easy/mav0/imu0/data.csv\" for EuRoC)')
    # parser.add_argument('gt', help='Ground truth path (e.g. \"Oxford Inertial Odometry Dataset/handheld/data4/syn/vi1.csv\" for OxIOD, \"MH_02_easy/mav0/state_groundtruth_estimate0/data.csv\" for EuRoC)')
    # args = parser.parse_args()

    class argument:
        # dataset = "euroc"
        # model = "6dofio_euroc.hdf5"
        # input = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/imu0/data.csv"
        # gt = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv"

        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/data_crane_filter.hdf5"
        input = "/mah/AI/6-DOF-Inertial-Odometry_orignal/data_imu_filter_short2.csv"
        gt = "/mah/AI/6-DOF-Inertial-Odometry_orignal/data_gt_filter_short2.csv"

        # input = "/home/aisl/catkin_ws/src/latest_crane/imu_pose/data_imu_end.csv"
        # gt = "/home/aisl/catkin_ws/src/latest_crane/imu_pose/data_gt_end.csv"
    
    args = argument()

    window_size = 200
    stride = 10
    ii = 0
    y_delta_p_array = np.zeros((1,3))
    y_delta_q_array = np.array([1,0,0,0])   
    yhat_delta_p_array = np.zeros((1,3))
    yhat_delta_q_array = np.array([1,0,0,0]) 
    first_loop_flag = False

    model = load_model(args.model)

    if args.dataset == 'oxiod':
        gyro_data, acc_data, pos_data, ori_data = load_oxiod_dataset(args.input, args.gt)
    elif args.dataset == 'euroc' or args.dataset == 'my_imu':
        gyro_data, acc_data, pos_data, ori_data = load_euroc_mav_dataset(args.input, args.gt)

    plot_fig("raw", gyro_data,acc_data, pos_data, ori_data)

    print("gyro_data = ", np.shape(gyro_data))
    print("acc_data = ", np.shape(acc_data))
    print("pos_data = ", np.shape(pos_data))
    print("ori_data = ", np.shape(ori_data))

    limit = len(gyro_data)-window_size-200
    

   
    for ii in range(0, limit, stride):
        gyro_data1 = gyro_data[ii:ii + window_size+2, :]
        acc_data1 = acc_data[ii :ii + window_size+2, :]
        pos_data1 = pos_data[ii :ii + window_size+2, :]
        ori_data1 = ori_data[ii :ii + window_size+2, :]
        # ii = ii+ stride

        print("gyro_data = ", np.shape(gyro_data1), "  ii ", ii, " limit ", limit)
        # print("acc_data = ", np.shape(acc_data))
        # print("pos_data = ", np.shape(pos_data))
        # print("ori_data = ", np.shape(ori_data))


        # plot_compare_pos("predict", "gt",  pos_data, ori_data, pos_data, ori_data)
        # plot_pos_error("predict error" ,  pos_data, ori_data, pos_data, ori_data)

        [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data1, acc_data1, pos_data1, ori_data1, window_size, stride)

        if (first_loop_flag == False):
            init_p_first = init_p
            init_q_first = init_q
        first_loop_flag = True

        # plot_fig("gt", x_gyro,x_acc, y_delta_p, y_delta_q)


        if args.dataset == 'oxiod':
            [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro[0:200, :, :], x_acc[0:200, :, :]], batch_size=1, verbose=1)
        elif args.dataset == 'euroc' or args.dataset == 'my_imu':
            [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)

        y_delta_p_array = np.vstack((y_delta_p_array,np.array(y_delta_p)))
        y_delta_q_array = np.vstack((y_delta_q_array, np.array(y_delta_q))) 
        yhat_delta_p_array = np.vstack((yhat_delta_p_array,np.array(yhat_delta_p)))
        yhat_delta_q_array = np.vstack((yhat_delta_q_array, np.array(yhat_delta_q)))


    n1 = 0
    n2 = len(yhat_delta_p)
    
    print("y_delta_p_array = ", np.shape(y_delta_p_array))
    print("y_delta_q_array = ", np.shape(y_delta_q_array))
    print("yhat_delta_p_array = ", np.shape(yhat_delta_p_array))
    print("yhat_delta_q_array = ", np.shape(yhat_delta_q_array))

    print("gyro_data = ", np.shape(gyro_data))
    print("acc_data = ", np.shape(acc_data))
    print("pos_data = ", np.shape(pos_data))
    print("ori_data = ", np.shape(ori_data))

    y_delta_p = y_delta_p_array
    y_delta_q = y_delta_q_array
    yhat_delta_p = yhat_delta_p_array
    yhat_delta_q = yhat_delta_q_array


    # plot_compare_pos("predict", "gt",  yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q)
    plot_pos_error("predict error" ,  yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q, n1, n2)
    # plot_compare_imu("given data", "change in data",  acc_data, gyro_data, x_acc, x_gyro)
    # plot_fig("perdict", x_gyro,x_acc, yhat_delta_p, yhat_delta_q)




    gt_trajectory = generate_trajectory_6d_quat(init_p_first, init_q_first, y_delta_p, y_delta_q)
    pred_trajectory = generate_trajectory_6d_quat(init_p_first, init_q_first, yhat_delta_p, yhat_delta_q)
    error_traj = gt_trajectory - pred_trajectory

    print("gt_trajectory = ", np.shape(gt_trajectory))
    print("pred_trajectory = ", np.shape(pred_trajectory))





    fig = plt.figure("Error in Traj xyz axes")
    plt.subplot(311); plt.plot(error_traj[:,0]); plt.grid(True); plt.title(" pos x");  
    plt.subplot(312); plt.plot(error_traj[:,1]); plt.grid(True); plt.title(" pos y"); 
    plt.subplot(313); plt.plot(error_traj[:,2]); plt.grid(True); plt.title(" pos z");   
    
  

    fig = plt.figure("pos and ori11")
    # plt.subplot(321); plt.plot(publish_y_delta_p_array[:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(321); plt.plot(gt_trajectory[1:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(323); plt.plot(gt_trajectory[1:,1], label='delta gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    plt.subplot(325); plt.plot(gt_trajectory[1:,2], label='delta gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

    plt.subplot(322); plt.plot(pos_data[1:,0], label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(324); plt.plot(pos_data[1:,1], label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    plt.subplot(326); plt.plot(pos_data[1:,2], label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()
   
    # x1 = np.linspace(0, 100, len(pos_data)-1)
    # x2 = np.linspace(0, 100, len(gt_trajectory)-1)
   
    # fig = plt.figure("pos and ori1122")
    # # plt.subplot(321); plt.plot(publish_y_delta_p_array[:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    # plt.subplot(311); plt.plot(x2, gt_trajectory[1:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    # plt.subplot(312); plt.plot(x2, gt_trajectory[1:,1], label='delta gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    # plt.subplot(313); plt.plot(x2, gt_trajectory[1:,2], label='delta gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

    # plt.subplot(311); plt.plot(x1, pos_data[1:,0], label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    # plt.subplot(312); plt.plot(x1, pos_data[1:,1], label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    # plt.subplot(313); plt.plot(x1, pos_data[1:,2], label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

    pos_data_slice = pos_data[1:len(pos_data):10]
    pos_data_slice = pos_data_slice[0:len(gt_trajectory),:]


    print("shape of pos_data_slice = ", np.shape(pos_data_slice))
    print("shape of gt_trajectory = ", np.shape(gt_trajectory))
    errr = pos_data_slice - gt_trajectory

    fig = plt.figure("pos and ori")
    # plt.subplot(321); plt.plot(publish_y_delta_p_array[:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(321); plt.plot(gt_trajectory[1:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(323); plt.plot(gt_trajectory[1:,1], label='delta gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    plt.subplot(325); plt.plot(gt_trajectory[1:,2], label='delta gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

    plt.subplot(321); plt.plot(pos_data_slice[1:,0], label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(323); plt.plot(pos_data_slice[1:,1], label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    plt.subplot(325); plt.plot(pos_data_slice[1:,2], label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()


    plt.subplot(322); plt.plot(errr[1:,0], label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
    plt.subplot(324); plt.plot(errr[1:,1], label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
    plt.subplot(326); plt.plot(errr[1:,2], label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()


    # fig = plt.figure("gt trajectory"); 
    fig = plt.figure(figsize=[14.4, 10.8]); #plt.title("gt trajectory")
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend(['gt trajectory'], loc='upper right')

    # fig = plt.figure("pred trajectory"); plt.title("pred trajectory")
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend(['pred trajectory'], loc='upper right')



    # fig = plt.figure("Error in Traj"); plt.title("Error in Traj")
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(error_traj[:, 0], error_traj[:, 1], error_traj[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend(['Error in Traj'], loc='upper right')

    

    







    # gt_trajectory = generate_trajectory_6d_quat_sep("gt",init_p, init_q, y_delta_p, y_delta_q)
    # pred_trajectory = generate_trajectory_6d_quat_sep("pr", init_p, init_q, yhat_delta_p, yhat_delta_q)

    if args.dataset == 'oxiod':
        gt_trajectory = gt_trajectory[0:200, :]

    matplotlib.rcParams.update({'font.size': 18})
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2])
    ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2])
    ax.plot(pos_data[:, 0], pos_data[:, 1], pos_data[:, 2])
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
    ax.legend(['ground truth', 'predicted', "pos gt"], loc='upper right')
    plt.show()

if __name__ == '__main__':
    main()