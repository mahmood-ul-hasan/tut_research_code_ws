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


def main_test(args):
   
    window_size = args.window_size
    stride =  args.stride

    model = load_model(args.model)

    if args.dataset == 'my_imu':
        gyro_data, acc_data, pos_data, ori_data = load_euroc_mav_dataset(args.input, args.gt)
    
    gyro_data = gyro_data[0:args.limit,:]
    acc_data = acc_data[0:args.limit,:]
    pos_data = pos_data[0:args.limit,:]
    ori_data = ori_data[0:args.limit,:] 

    print("gyro_data = ",np.shape(gyro_data))
    print("acc_data = ",np.shape(acc_data))
    print("pos_data = ",np.shape(pos_data))
    print("ori_data = ",np.shape(ori_data))

    # plot_fig("raw", gyro_data,acc_data, pos_data, ori_data)





    # plot_compare_pos("predict", "gt",  pos_data, ori_data, pos_data, ori_data)
    # plot_pos_error("predict error" ,  pos_data, ori_data, pos_data, ori_data)

    [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)



    # plot_fig("gt", x_gyro,x_acc, y_delta_p, y_delta_q)

    if args.dataset == 'my_imu':
        [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)

    return   init_p, init_q, [y_delta_p, y_delta_q],[yhat_delta_p, yhat_delta_q]

    