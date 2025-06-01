import numpy as np
import matplotlib.pyplot as plt
import argparse
from test_crane_data import *
from sklearn.metrics import mean_squared_error
from math import sqrt



def plot_trajectory_first(title, legend, gt_trajectory,pred_trajectory):

    fig = plt.figure("Trjectory " + str(title))
    ax = fig.gca(projection='3d')
    # matplotlib.rcParams.update({'font.size': 18})
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2], label='ground truth')
    ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2], label= 'predicted ('+str(legend)+ ")")
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
    ax.legend()
    # ax.legend(['ground truth', 'predicted ('+str(legend)+ ")"], loc='upper right')


def plot_trajectory(title, legend, gt_trajectory,pred_trajectory):

    fig = plt.figure("Trjectory " + str(title))
    ax = fig.gca(projection='3d')
    # matplotlib.rcParams.update({'font.size': 18})
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2], label='ground truth')
    # ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2], label= 'predicted ('+str(legend)+ ")")
    ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2], label= 'predicted ('+str(legend)+ ")")
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
    ax.legend()
    # ax.legend(['ground truth', 'predicted ('+str(legend)+ ")"], loc='upper right')


def plot_gt_trajectory_3d(title, legend, gt_trajectory):

    fig = plt.figure("gt Trjectory " + str(title))
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2], label='ground truth ' + str(legend))
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()



def plot_trj_error_vector(title, legend , error_traj):
    fig = plt.figure("Error in Trajectory xyz axes "  + str(title))
    plt.subplot(311); plt.plot(error_traj[:,0], label=  str(legend)); plt.grid(True);  plt.ylabel("X (m)");  plt.title("Error in xyz axes of Trajectory");  plt.legend()
    plt.subplot(312); plt.plot(error_traj[:,1], label=  str(legend)); plt.grid(True);  plt.ylabel("Y (m)");    plt.legend()
    plt.subplot(313); plt.plot(error_traj[:,2], label=  str(legend)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Z (m)");    plt.legend()

def plot_gt_trj(title, legend , traj):
    fig = plt.figure("gt rajectory xyz axes "  + str(title))
    plt.subplot(311); plt.plot(traj[:,0], label=  str(legend)); plt.grid(True);  plt.ylabel("X (m)");  plt.title("gt rajectory xyz axes");  plt.legend()
    plt.subplot(312); plt.plot(traj[:,1], label=  str(legend)); plt.grid(True);  plt.ylabel("Y (m)");    plt.legend()
    plt.subplot(313); plt.plot(traj[:,2], label=  str(legend)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Z (m)");    plt.legend()




def plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q):
    plot_delta_position_perdict_vs_gt(title, legend , yhat_delta_p, y_delta_p)
    plot_delta_rotation_angle_perdict_vs_gt(title, legend , yhat_delta_q, y_delta_q)
    plot_delta_gt(title, legend, y_delta_p, y_delta_q )

    gt_trajectory = generate_trajectory_6d_quat(init_p, init_q, y_delta_p, y_delta_q)
    pred_trajectory = generate_trajectory_6d_quat(init_p, init_q, yhat_delta_p, yhat_delta_q)
    error_traj = gt_trajectory - pred_trajectory

    result = sqrt(mean_squared_error(gt_trajectory,pred_trajectory))
# Print the result
    print("RMSE: " + str(legend), result)


    plot_trj_error_vector(title, legend , error_traj)

    plot_trajectory(title, legend , gt_trajectory,pred_trajectory)
    plot_gt_trajectory_3d(title, legend ,gt_trajectory)
    plot_gt_trj(title, legend ,gt_trajectory)


def plot_graphs_first(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q):
    plot_delta_position_perdict_vs_gt(title, legend , yhat_delta_p, y_delta_p)
    plot_delta_rotation_angle_perdict_vs_gt(title, legend , yhat_delta_q, y_delta_q)

    gt_trajectory = generate_trajectory_6d_quat(init_p, init_q, y_delta_p, y_delta_q)
    pred_trajectory = generate_trajectory_6d_quat(init_p, init_q, yhat_delta_p, yhat_delta_q)
    error_traj = gt_trajectory - pred_trajectory
    result = sqrt(mean_squared_error(gt_trajectory,pred_trajectory))
# Print the result
    print("RMSE: " + str(title), result)
    plot_trj_error_vector(title, legend , error_traj)

    plot_trajectory_first(title, legend , gt_trajectory,pred_trajectory)



    # if first_loop_flag == False:
    
    gt_trj_p = init_p
    gt_trj_q = quaternion.from_float_array(init_q)
    pred_trj_p = init_p
    pred_trj_q = quaternion.from_float_array(init_q)

    for i in range(len(y_delta_p)):
        gt_trj_p, gt_trj_q = generate_trajectory_6d_quat_thread(gt_trj_p, gt_trj_q, y_delta_p, y_delta_q[0,:])
        pred_trj_p, pred_trj_q = generate_trajectory_6d_quat_thread(pred_trj_p, pred_trj_q, yhat_delta_p, yhat_delta_q[0,:])




    
    # plot_pos_error("dgt vs gt error " ,  pred_trj_p,  pred_trj_q, gt_trj_p, gt_trj_q, 0, len(y_delta_q))


    # plot_trajectory(title, legend , gt_trajectory,pred_trajectory)


def plot_sampling_rate_01():

    class win_26_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_gt_data_filter.csv'
        window_size = 26
        stride = 26
        limit = 100*60
    args = win_26_dt_01_filter()


    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):8,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):8,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):8,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):8,:]
    title = "sampling rate = 0.01"
    legend = "win_26_filter"
    plot_graphs(title, legend,init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))




    class win_26_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_26_dt_0.01_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_gt_data.csv'
        window_size = 26
        stride = 26
        limit = 100*60
    args = win_26_dt_01_unfilter()

    title = "sampling rate = 0.01"
    legend = "win_26_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):8,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):8,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):8,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):8,:]

    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))





    class win_200_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_gt_data.csv'
        window_size = 200
        stride = 200
        limit = 100*60
    args = win_200_dt_01_unfilter()

    title = "sampling rate = 0.01"
    legend = "win_200_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))


    class win_200_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_gt_data_filter.csv'
        window_size = 200
        stride = 200
        limit = 100*60
    args = win_200_dt_01_filter()

    title = "sampling rate = 0.01"
    legend = "win_200_filtered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))



    class win_50_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_gt_data_filter.csv'
        window_size = 50
        stride = 50
        limit = 100*60
    args = win_50_dt_01_filter()

    title = "sampling rate = 0.01"
    legend = "win_50_filtered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):4,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):4,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):4,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):4,:]
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))



    class win_50_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_50_dt_0.01_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_gt_data.csv'
        window_size = 50
        stride = 50
        limit = 100*60
    args = win_50_dt_01_unfilter()

    title = "sampling rate = 0.01"
    legend = "win_50_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):4,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):4,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):4,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):4,:]
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))




def plot_sampling_rate_01_filter():

    class win_26_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_gt_data_filter.csv'
        window_size = 26
        stride = 26
        limit = 100*60
    args = win_26_dt_01_filter()


    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):8,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):8,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):8,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):8,:]
    title = "sampling rate = 0.01"
    legend = "win_200"
    plot_graphs(title, legend,init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))



    class win_50_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_gt_data_filter.csv'
        window_size = 50
        stride = 50
        limit = 100*60
    args = win_50_dt_01_filter()

    title = "sampling rate = 0.01"
    legend = "win_50"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):4,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):4,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):4,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):4,:]
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))





    class win_200_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_gt_data_filter.csv'
        window_size = 200
        stride = 200
        limit = 100*60
    args = win_200_dt_01_filter()

    title = "sampling rate = 0.01"
    legend = "win_26"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))


    class win_26_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_26_dt_0.01_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_gt_data.csv'
        window_size = 26
        stride = 26
        limit = 100*60
    args = win_26_dt_01_unfilter()

    title = "sampling rate = 0.01"
    legend = "win_26_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):8,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):8,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):8,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):8,:]

    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))





    class win_200_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_gt_data.csv'
        window_size = 200
        stride = 200
        limit = 100*60
    args = win_200_dt_01_unfilter()

    title = "sampling rate = 0.01"
    legend = "win_200_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    print("y_delta_p = ",np.shape(y_delta_p))
    print("y_delta_q = ",np.shape(y_delta_q))
    print("yhat_delta_p = ",np.shape(yhat_delta_p))
    print("yhat_delta_q = ",np.shape(yhat_delta_q))





####################################################################

def plot_sampling_rate_002(title):

    class win_26_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.002_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.002_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.002_gt_data_filter.csv'
        window_size = 26
        stride = 26
        limit = 100*60
    args = win_26_dt_01_filter()


    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    legend = "win_26_filter"
    y_delta_p = y_delta_p[0:len(y_delta_p):8,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):8,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):8,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):8,:]
    plot_graphs(title, legend,init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)




    class win_26_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_26_dt_0.002_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_gt_data.csv'
        window_size = 26
        stride = 26
        limit = 100*60
    args = win_26_dt_01_unfilter()

    legend = "win_26_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):8,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):8,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):8,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):8,:]
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)





    class win_200_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_gt_data.csv'
        window_size = 200
        stride = 200
        limit = 100*60
    args = win_200_dt_01_unfilter()

    legend = "win_200_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)


    class win_200_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.002_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.002_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.002_gt_data_filter.csv'
        window_size = 200
        stride = 200
        limit = 100*60
    args = win_200_dt_01_filter()

    legend = "win_200_filtered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)
    print(legend)
    



    class win_50_dt_01_filter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.002_model_ep_800_data_filter.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.002_imu_data_filter.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.002_gt_data_filter.csv'
        window_size = 50
        stride = 50
        limit = 100*60
    args = win_50_dt_01_filter()

    legend = "win_50_filtered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):4,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):4,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):4,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):4,:]
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)



    class win_50_dt_01_unfilter:
        dataset = "my_imu"
        model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_50_dt_0.002_model_ep_800_data.hdf5"
        input = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_imu_data.csv'
        gt = '/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_gt_data.csv'
        window_size = 50
        stride = 50
        limit = 100*60
    args = win_50_dt_01_unfilter()

    legend = "win_50_unfiltered"

    init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q] = main_test(args)
    y_delta_p = y_delta_p[0:len(y_delta_p):4,:]
    y_delta_q = y_delta_q[0:len(y_delta_q):4,:]
    yhat_delta_p = yhat_delta_p[0:len(yhat_delta_p):4,:]
    yhat_delta_q = yhat_delta_q[0:len(yhat_delta_q):4,:]
    plot_graphs(title, legend, init_p, init_q, y_delta_p, y_delta_q,yhat_delta_p, yhat_delta_q)



title = "sampling rate = 0.002"
# plot_sampling_rate_002(title)
plot_sampling_rate_01()
# plot_sampling_rate_01_filter()




plt.show()

