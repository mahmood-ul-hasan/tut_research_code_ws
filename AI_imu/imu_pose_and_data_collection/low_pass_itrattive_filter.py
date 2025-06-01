import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


from low_pass_filter_function import *



import rospy

from numpy import savetxt
from sensor_msgs.msg import Imu
import time
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import tf2_ros
imu_data_filter_array = np.zeros((1,7))
gt_data_filter_array = np.zeros((1,8))







def thread_filter_NN(gt_data,imu_data ) :
   
    global  imu_data_filter_array
    global gt_data_filter_array
    global time_interval

    time_array = imu_data[:,0]
    noisy_wx = imu_data[:,1]
    noisy_wy = imu_data[:,2]
    noisy_wz = imu_data[:,3]
    noisy_ax = imu_data[:,4]
    noisy_ay = imu_data[:,5]
    noisy_az = imu_data[:,6]


    filtered_wx, wx_noise_freq, wx_noise_amp  = apply_filter(time_interval, time_array, noisy_wx, "wx")
    filtered_wy, wy_noise_freq, wy_noise_amp  = apply_filter(time_interval, time_array, noisy_wy,"wy")
    filtered_wz, wz_noise_freq, wz_noise_amp  = apply_filter(time_interval, time_array, noisy_wz,"wz")

    filtered_ax, ax_noise_freq, ax_noise_amp  = apply_filter(time_interval, time_array, noisy_ax,"ax")
    filtered_ay, ay_noise_freq, ay_noise_amp  = apply_filter(time_interval, time_array, noisy_ay,"ay")
    filtered_az, az_noise_freq, az_noise_amp  = apply_filter(time_interval, time_array, noisy_az,"az")

    imu_data_filter =  np.vstack((time_array, filtered_wx, filtered_wy, filtered_wz, filtered_ax, filtered_ay, filtered_az))
    imu_data_filter = np.array(imu_data_filter)
    imu_data_filter = np.transpose(imu_data_filter)
    imu_data_filter_array = np.vstack((imu_data_filter_array,imu_data_filter))   
    gt_data_filter_array =  np.vstack((gt_data_filter_array,gt_data))
    # print("imu_data_filter_array = ", np.shape(imu_data_filter_array))   
    # print("imu_data_filter =       ", np.shape(imu_data_filter))   

                



  


   
if __name__ == '__main__':

    

    imu_data_filename = "/mah/AI/imu_pose/4_1_simple_motion_win_200_dt_0.002_imu_data.csv"
    gt_data_filename = "/mah/AI/imu_pose/4_1_simple_motion_win_200_dt_0.002_gt_data.csv"

    imu_data = pd.read_csv(imu_data_filename).values
    gt_data = pd.read_csv(gt_data_filename).values

    print("imu_data =", np.shape(imu_data))
    print("gt_data =", np.shape(gt_data))

    n1 = 0
    n1 = int(n1)
    # n2 = 100* 10
    n2 = len(imu_data)
    imu_data = imu_data[n1:n2,:]
    gt_data = gt_data[n1:n2,:]
    time_array = imu_data[:,0]
    imu_data = np.array(imu_data)
    gt_data = np.array(gt_data)

    ii =0
    window_size = 200
    stride = 200
    time_interval = 0.002
    delta_time = 0
    num =1



    while  (ii+ window_size+2 <= len(imu_data)) or rospy.is_shutdown():
     try:
        
        imu_data1 = imu_data[ii :ii + window_size, :]
        gt_data1 = gt_data[ii :ii + window_size, :]
        # print("imu_data[ii, :] = ", imu_data[ii, 0])  
        # print("imu_data[ii + window_size+2, :] = ", imu_data[ii + window_size+2, 0])  
        a = imu_data[0:ii + window_size, :]
        b = imu_data[ii:ii + window_size, :]
        # print("imu_data[ii + window_size+2, :] = ", np.shape(a))  
        # print("imu_data[ii + window_size+2, :] = ", np.shape(b))  

        thread_filter_NN(gt_data1,imu_data1 )     
        ii = ii+ stride

        print("===============================================ii = ", ii)  

       
     except KeyboardInterrupt:
        break



    # thread1.join()


    imu_data = imu_data[0:ii + window_size-stride,:]
    gt_data = gt_data[0:ii + window_size-stride,:]
    imu_time_array = imu_data[:,0]
    gt_time_array = gt_data[:,0]
    noisy_wx = imu_data[:,1]
    noisy_wy = imu_data[:,2]
    noisy_wz = imu_data[:,3]
    noisy_ax = imu_data[:,4]
    noisy_ay = imu_data[:,5]
    noisy_az = imu_data[:,6]
    print("imu_data =", np.shape(imu_data))
    print("gt_data =", np.shape(gt_data))
 

    imu_data_filter = imu_data_filter_array[1:,:]
    gt_data_filter = gt_data_filter_array[1:,:]
    print("imu_data_filter =", np.shape(imu_data_filter))
    print("gt_data_filter =", np.shape(gt_data_filter))


    imu_filtered_time_array = imu_data_filter[:,0]
    gt_filtered_time_array = gt_data_filter[:,0]
    filtered_wx = imu_data_filter[:,1]
    filtered_wy = imu_data_filter[:,2]
    filtered_wz = imu_data_filter[:,3]
    filtered_ax = imu_data_filter[:,4]
    filtered_ay = imu_data_filter[:,5]
    filtered_az = imu_data_filter[:,6]

    time_array= imu_time_array

    print("imu -imu_filter time axes check = ", imu_time_array-imu_filtered_time_array)
    print("gt - gt_filter  time axes check = ", gt_time_array-gt_filtered_time_array)
    print("gt - imu_filter time axes check = ", gt_time_array-imu_filtered_time_array)
    print("imu - gt_filter time axes check = ", imu_time_array-gt_filtered_time_array)
    print("imu - gt        time axes check = ", imu_time_array-gt_time_array)
    print("imu_fil-gt_fil  time axes check = ", imu_filtered_time_array-gt_filtered_time_array)

    
    
    plt.figure("time ")
    plt.plot(imu_time_array-imu_filtered_time_array, label='imu -imu_filter '); plt.grid(True);    plt.legend(); 
    plt.plot(gt_time_array-gt_filtered_time_array, label='gt - gt_filter'); plt.grid(True);    plt.legend(); 
    plt.plot(gt_time_array-imu_filtered_time_array, label='gt - imu_filter '); plt.grid(True);    plt.legend(); 
    plt.plot(imu_time_array-gt_filtered_time_array, label='imu - gt_filter time '); plt.grid(True);    plt.legend(); 
    plt.plot(imu_time_array-gt_time_array, label='imu - gt'); plt.grid(True);    plt.legend(); 
    plt.plot( imu_filtered_time_array-gt_filtered_time_array, label='imu_fil-gt_fil'); plt.grid(True);    plt.legend(); 



       
     

    plt.figure("Gyroscope Noisy Signal vs Filtered Signal")
    plt.subplot(311); plt.plot(time_array, noisy_wx, label='noisy_wx'); plt.grid(True);    plt.legend(); plt.title("Gyroscope Noisy Signal vs Filtered Signal");
    plt.subplot(312); plt.plot(time_array, noisy_wy, label='noisy_wy'); plt.grid(True);    plt.legend()
    plt.subplot(313); plt.plot(time_array, noisy_wz, label='noisy_wz'); plt.grid(True);    plt.legend()

    plt.subplot(311); plt.plot(time_array, filtered_wx, label='filtered_wx'); plt.grid(True);   plt.legend()
    plt.subplot(312); plt.plot(time_array, filtered_wy, label='filtered_wy'); plt.grid(True);   plt.legend()
    plt.subplot(313); plt.plot(time_array, filtered_wz, label='filtered_wz'); plt.grid(True);   plt.legend(); plt.xlabel('time')

    plt.figure("Accelerometer Noisy Signal vs Filtered Signal")
    plt.subplot(311); plt.plot(time_array, noisy_ax, label='noisy_ax'); plt.grid(True);    plt.legend(); plt.title("Accelerometer Noisy Signal vs Filtered Signal");
    plt.subplot(312); plt.plot(time_array, noisy_ay, label='noisy_ay'); plt.grid(True);    plt.legend()
    plt.subplot(313); plt.plot(time_array, noisy_az, label='noisy_az'); plt.grid(True);    plt.legend()

    plt.subplot(311); plt.plot(time_array, filtered_ax, label='filtered_ax'); plt.grid(True);   plt.legend()
    plt.subplot(312); plt.plot(time_array, filtered_ay, label='filtered_ay'); plt.grid(True);   plt.legend()
    plt.subplot(313); plt.plot(time_array, filtered_az, label='filtered_az'); plt.grid(True);   plt.legend(); plt.xlabel('time')

   
   

    print("==================================================================")
    print("imu_data_filter =", np.shape(imu_data_filter))
    print("gt_data_filter =", np.shape(gt_data_filter))



    np.savetxt('/mah/AI/imu_pose/4_1_simple_motion_win_200_dt_0.002_imu_data_filter.csv', imu_data_filter, delimiter=',', fmt = '%.9f')
    np.savetxt('/mah/AI/imu_pose/4_1_simple_motion_win_200_dt_0.002_gt_data_filter.csv', gt_data_filter, delimiter=',', fmt = '%.9f')

    plt.legend()
    plt.show()



