
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from matplotlib.animation import FuncAnimation
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math

def plot_fig(cur_gyro_data,cur_acc_data, cur_pos_data, cur_ori_data):

    euler_ori_data = np.zeros((len(cur_ori_data),3))
    euler_y_delta_q = np.zeros((len(cur_ori_data),3))
    for i in range(len(cur_ori_data)):
      euler_ori_data[i,:]= euler_from_quaternion(cur_ori_data[i,:], axes='szyx')
     # euler_y_delta_q[i,:]= euler_from_quaternion(y_delta_q[i,:])

    fig = plt.figure("cur_gyro_data and cur_acc_data")
    plt.subplot(321); plt.plot(cur_gyro_data[:,0]*180/math.pi); plt.grid(True); plt.title("gyro x")
    plt.subplot(323); plt.plot(cur_gyro_data[:,1]*180/math.pi); plt.grid(True); plt.title("gyro y")
    plt.subplot(325); plt.plot(cur_gyro_data[:,2]*180/math.pi); plt.grid(True); plt.title("gyro z")

    plt.subplot(322); plt.plot(cur_acc_data[:,0]); plt.grid(True); plt.title("acc x")
    plt.subplot(324); plt.plot(cur_acc_data[:,1]); plt.grid(True); plt.title("acc y")
    plt.subplot(326); plt.plot(cur_acc_data[:,2]); plt.grid(True); plt.title("acc z")


    fig = plt.figure("pos and ori")
    plt.subplot(321); plt.plot(euler_ori_data[:,0]*180/math.pi); plt.grid(True); plt.title("ori x")
    plt.subplot(323); plt.plot(euler_ori_data[:,1]*180/math.pi); plt.grid(True); plt.title("ori y")
    plt.subplot(325); plt.plot(euler_ori_data[:,2]*180/math.pi); plt.grid(True); plt.title("ori z")

    plt.subplot(322); plt.plot(cur_pos_data[:,0]*180/math.pi); plt.grid(True); plt.title("pos x")
    plt.subplot(324); plt.plot(cur_pos_data[:,1]*180/math.pi); plt.grid(True); plt.title("pos y")
    plt.subplot(326); plt.plot(cur_pos_data[:,2]); plt.grid(True); plt.title("pos z")


    fig = plt.figure("quat")
    plt.subplot(411); plt.plot(cur_ori_data[:,0]); plt.grid(True); plt.title("ori x")
    plt.subplot(412); plt.plot(cur_ori_data[:,1]); plt.grid(True); plt.title("ori x")
    plt.subplot(413); plt.plot(cur_ori_data[:,2]); plt.grid(True); plt.title("ori x")
    plt.subplot(414); plt.plot(cur_ori_data[:,3]); plt.grid(True); plt.title("ori x")
    plt.show()



    
    # fig = plt.figure("x_gyro and x_acc")
    # plt.subplot(321); plt.plot(x_gyro[0:220,0,0]); plt.grid(True); plt.title("gyro x")
    # plt.subplot(323); plt.plot(x_gyro[0:220,0,1]); plt.grid(True); plt.title("gyro y")
    # plt.subplot(325); plt.plot(x_gyro[0:220,0,2]); plt.grid(True); plt.title("gyro z")

    # plt.subplot(322); plt.plot(x_acc[0:220,0,0]); plt.grid(True); plt.title("acc x")
    # plt.subplot(324); plt.plot(x_acc[0:220,0,1]); plt.grid(True); plt.title("acc y")
    # plt.subplot(326); plt.plot(x_acc[0:220,0,2]); plt.grid(True); plt.title("acc z")

    # plt.subplot(321); plt.plot(x_gyro[0:220,2,0]); plt.grid(True); plt.title("gyro x")
    # plt.subplot(323); plt.plot(x_gyro[0:220,2,1]); plt.grid(True); plt.title("gyro y")
    # plt.subplot(325); plt.plot(x_gyro[0:220,2,2]); plt.grid(True); plt.title("gyro z")

    # plt.subplot(322); plt.plot(x_acc[0:220,2,0]); plt.grid(True); plt.title("acc x")
    # plt.subplot(324); plt.plot(x_acc[0:220,2,1]); plt.grid(True); plt.title("acc y")
    # plt.subplot(326); plt.plot(x_acc[0:220,2,2]); plt.grid(True); plt.title("acc z")



   

    # fig = plt.figure("y_delta_p and y_delta_q")
    # plt.subplot(321); plt.plot(euler_y_delta_q[0:220,0]); plt.grid(True); plt.title("ori x")
    # plt.subplot(323); plt.plot(euler_y_delta_q[0:220,1]); plt.grid(True); plt.title("ori y")
    # plt.subplot(325); plt.plot(euler_y_delta_q[0:220,2]); plt.grid(True); plt.title("ori z")

    # plt.subplot(322); plt.plot(y_delta_p[0:220,0]); plt.grid(True); plt.title("pos x")
    # plt.subplot(324); plt.plot(y_delta_p[0:220,1]); plt.grid(True); plt.title("pos y")
    # plt.subplot(326); plt.plot(y_delta_p[0:220,2]); plt.grid(True); plt.title("pos z")