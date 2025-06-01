import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def plot(data,x):
    plt.plot(data[:,0], label= " Training Loss  " + str(x))
    plt.plot(data[:,1], label= "Validation Loss"+  str(x))
    plt.ylabel('Loss')
    plt.xlabel('Epoch')

def plot_loss_for_window(win, dt_01_filter, dt_002_filter, dt_01_unfiltered, dt_002_unfiltered):
    dt_01_filter = pd.read_csv(dt_01_filter).values
    dt_002_filter = pd.read_csv(dt_002_filter).values
    dt_01_unfiltered = pd.read_csv(dt_01_unfiltered).values
    dt_002_unfiltered = pd.read_csv(dt_002_unfiltered).values


    fig = plt.figure("Modelloss for Window size " + str(win))
    plt.title('Modelloss for Window size '+ str(win))
    plot(dt_01_filter, "(sampling = 0.01,  Filter data)")
    plot(dt_01_unfiltered,"(sampling = 0.01,  Unfiltered data)")
    plot(dt_002_filter,"(sampling = 0.002, Filter data)")
    plot(dt_002_unfiltered,"(sampling = 0.002, Unfiltered data)")
    plt.legend()


def plot_loss_for_window_01(win, dt_01_filter, dt_01_unfiltered):
    dt_01_filter = pd.read_csv(dt_01_filter).values
    # dt_002_filter = pd.read_csv(dt_002_filter).values
    dt_01_unfiltered = pd.read_csv(dt_01_unfiltered).values
    # dt_002_unfiltered = pd.read_csv(dt_002_unfiltered).values


    fig = plt.figure("Modelloss for Window size " + str(win))
    plt.title('Modelloss for Window size '+ str(win))
    plot(dt_01_filter, "(sampling = 0.01,  Filter data)")
    plot(dt_01_unfiltered,"(sampling = 0.01,  Unfiltered data)")
    # plot(dt_002_filter,"(sampling = 0.002, Filter data)")
    # plot(dt_002_unfiltered,"(sampling = 0.002, Unfiltered data)")
    plt.legend()

def plot_loss_for_window_all(win_25, win_50,  win_200):
    
    win_25 = pd.read_csv(win_25).values
    win_50 = pd.read_csv(win_50).values
    win_200 = pd.read_csv(win_200).values


    fig = plt.figure("Modelloss for Different Window size ")
    # plt.title('Modelloss for Window size')
    plot(win_25, "(window size = 26)")
    plot(win_50, "(window size = 50)")
    plot(win_200, "(window size = 200)")
  
    plt.legend()

win_50_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_modelloss_ep_800_data_filter.csv"
win_50_dt_002_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.002_modelloss_ep_800_data_filter.csv"
win_50_dt_01_unfiltered = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_50_dt_0.01_modelloss_ep_800_data.csv"
win_50_dt_002_unfiltered = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_50_dt_0.002_modelloss_ep_800_data.csv"

# plot_loss_for_window_01("50", win_50_dt_01_filter,  win_50_dt_01_unfiltered)



win_26_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.01_modelloss_ep_800_data_filter.csv"
win_26_dt_002_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win26/4_1_simple_motion_win_26_dt_0.002_modelloss_ep_800_data_filter.csv"
win_26_dt_01_unfiltered = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_26_dt_0.01_modelloss_ep_800_data.csv"
win_26_dt_002_unfiltered = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_26_dt_0.002_modelloss_ep_800_data.csv"

# plot_loss_for_window_01("26",win_26_dt_01_filter, win_26_dt_01_unfiltered)


win_200_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.01_modelloss_ep_800_data_filter.csv"
win_200_dt_002_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win200/4_1_simple_motion_win_200_dt_0.002_modelloss_ep_800_data_filter.csv"
win_200_dt_01_unfiltered = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.01_modelloss_ep_800_data.csv"
win_200_dt_002_unfiltered = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter/4_1_simple_motion_win_200_dt_0.002_modelloss_ep_800_data.csv"


# win_26_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter_stride/4_1_simple_motion_win_26_dt_0.01_stride_model_loss"
# win_50_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter_stride/4_1_simple_motion_win_50_dt_0.01_stride_model_loss"
# win_100_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter_stride/4_1_simple_motion_win_100_dt_0.01_stride_model_loss"
# win_200_dt_01_filter = "/mah/AI/6-DOF-Inertial-Odometry_orignal/unfilter_stride/4_1_simple_motion_win_200_dt_0.01_stride_model_loss"

# plot_loss_for_window_01("200",win_200_dt_01_filter, win_200_dt_01_unfiltered)
# plot_loss_for_window("200",win_200_dt_01_filter, win_200_dt_002_filter, win_200_dt_01_unfiltered, win_200_dt_002_unfiltered)

plot_loss_for_window_all(win_26_dt_01_filter, win_50_dt_01_filter, win_200_dt_01_filter )


plt.show()