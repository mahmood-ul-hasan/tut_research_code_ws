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
from low_pass_filter_function import *


import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
from numpy import savetxt
from sensor_msgs.msg import Imu
import time
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

import tf2_ros
import geometry_msgs.msg
import threading


array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
i =0


class GazeboLinkPose:
  # line name to check its currect state
  link_name = '/gazebo/link_states'
  link_pose = Pose()
  imu = Imu()
  link_pose_rpy = Float32MultiArray()
  array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
  i= 0

  
  def __init__(self, link_name):
    self.link_name = link_name
    self.link_name_rectified = link_name.replace("::", "_")

    if not self.link_name:
      raise ValueError("'link_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)
    self.imu_sub = rospy.Subscriber("/imu_boom_link/data", Imu, self.imu_callback)
    self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)
    self.pose_rpy_pub = rospy.Publisher("/gazebo/k_crane_front_laser_link/rpy", Float32MultiArray, queue_size=1)


  def imu_callback(self, data):
   try:
     self.imu = data
   except ValueError:
     pass

  def callback(self, data):
    try:
      global i
      global array
      ind = data.name.index(self.link_name)
      self.link_pose = data.pose[ind]



      # Convert quat to rpy
      quat_upper = (
      self.link_pose.orientation.x,
      self.link_pose.orientation.y,
      self.link_pose.orientation.z,
      self.link_pose.orientation.w)
      euler_upper = euler_from_quaternion(quat_upper)
      # publish rpy
      array[0]= euler_upper[0]*180/math.pi
      array[1]= euler_upper[1]*180/math.pi
      array[2]= euler_upper[2]*180/math.pi
      array[3]= i
      self.link_pose_rpy = Float32MultiArray(data=array)
      # print("self.link_pose_rpy = ", self.link_pose_rpy)
      i = i+1
    except ValueError:
      pass




def publish_lidar_tf(yhat_delta_p, yhat_delta_q):

  # yhat_delta_q = yhat_delta_q / np.linalg.norm(yhat_delta_q)
  yhat_delta_q = quaternion.as_float_array(yhat_delta_q)
  br = tf2_ros.TransformBroadcaster()
  t = geometry_msgs.msg.TransformStamped()
  

  t.header.stamp = rospy.Time.now()
  t.header.frame_id = "upper_link"
  t.child_frame_id = "boom_link"
  t.transform.translation.x =  yhat_delta_p[0]
  t.transform.translation.y =  yhat_delta_p[1]
  t.transform.translation.z =  yhat_delta_p[2]
  
  t.transform.rotation.w = yhat_delta_q[0]
  t.transform.rotation.x = yhat_delta_q[1]
  t.transform.rotation.y = yhat_delta_q[2]
  t.transform.rotation.z = yhat_delta_q[3]
  br.sendTransform(t)


def static_transform_publisher(header, child,x,y,z):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = header
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)


def timer_get_imu_data():
    global pervious_time_1
   

    now_time_stamp1 = time.time()
    delta_time1 = now_time_stamp1 - init_time_stamp
     # to publish the state with new topic name
    gp.pose_pub.publish(gp.link_pose) 
    # rospy.loginfo(gp.link_pose_rpy)
    gp.pose_rpy_pub.publish(gp.link_pose_rpy)
    
   


    # if now_time_stamp1 >=  pervious_time_1 +  (0.01):
    pos = [delta_time1, gp.link_pose.position.x,gp.link_pose.position.y,gp.link_pose.position.z,gp.link_pose.orientation.w, gp.link_pose.orientation.x, gp.link_pose.orientation.y, gp.link_pose.orientation.z]
    imu_pose.append(pos)
    w = gp.imu.angular_velocity
    acc =gp.imu.linear_acceleration
    sensor_data = [delta_time1,w.x, w.y, w.z , acc.x, acc.y, acc.z ]
    imu_sensor_data.append(sensor_data)
    pervious_time_1 = now_time_stamp1 
    print("delta_time", delta_time1)
    print("================================================================")
    # if(delta_time >= duration):
      # timer1.shutdown

def thread_filter_NN(gt_data, imu_data):

    

    global publish_yhat_delta_p
    global publish_yhat_delta_q
    global yhat_delta_p
    global yhat_delta_q
    global first_loop_flag
    global first_loop_flag2
    global gt_trajectory_array
    global pred_trajectory_array
    global gt_data_array
    global imu_data_array
    global y_delta_p_array
    global y_delta_q_array
    global yhat_delta_p_array
    global yhat_delta_q_array
    global init_p_first 
    global init_q_first

    # print("gt_data = ", np.shape(gt_data))
    # print("imu_data = ", np.shape(imu_data))

    time_array = imu_data[:,0]
    noisy_wx = imu_data[:,1]
    noisy_wy = imu_data[:,2]
    noisy_wz = imu_data[:,3]
    noisy_ax = imu_data[:,4]
    noisy_ay = imu_data[:,5]
    noisy_az = imu_data[:,6]


    filtered_wx, wx_noise_freq, wx_noise_amp  = apply_filter(time_array, noisy_wx,"wx")
    filtered_wy, wy_noise_freq, wy_noise_amp  = apply_filter(time_array, noisy_wy,"wy")
    filtered_wz, wz_noise_freq, wz_noise_amp  = apply_filter(time_array, noisy_wz,"wz")

    filtered_ax, ax_noise_freq, ax_noise_amp  = apply_filter(time_array, noisy_ax,"ax")
    filtered_ay, ay_noise_freq, ay_noise_amp  = apply_filter(time_array, noisy_ay,"ay")
    filtered_az, az_noise_freq, az_noise_amp  = apply_filter(time_array, noisy_az,"az")

    imu_data_filter =  np.vstack((time_array, filtered_wx, filtered_wy, filtered_wz, filtered_ax, filtered_ay, filtered_az))
    imu_data_filter = np.array(imu_data_filter)
    imu_data_filter = np.transpose(imu_data_filter)
                

    gyro_data = interpolate_3dvector_linear(imu_data_filter[:, 1:4], imu_data_filter[:, 0], gt_data[:, 0])
    acc_data = interpolate_3dvector_linear(imu_data_filter[:, 4:7], imu_data_filter[:, 0], gt_data[:, 0])
    pos_data = gt_data[:, 1:4]
    ori_data = gt_data[:, 4:8]
    

    print("gyro_data = ", np.shape(gyro_data))
    print("acc_data = ", np.shape(acc_data))
    print("pos_data = ", np.shape(pos_data))
    print("ori_data = ", np.shape(ori_data))

    [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)
            
    if (first_loop_flag2 == False):
      publish_yhat_delta_p = init_p
      publish_yhat_delta_q = quaternion.from_float_array(init_q)
      init_p_first = init_p
      init_q_first = init_q

    first_loop_flag2 = True


    [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)
    first_loop_flag = True
    publish_yhat_delta_p = publish_yhat_delta_p + yhat_delta_p[0,:]
    publish_yhat_delta_q = publish_yhat_delta_q * quaternion.from_float_array(yhat_delta_q[0,:]).normalized()



    # print("gt_data = ", np.shape(gt_data))
    # print("imu_data = ", np.shape(imu_data))
    # print("y_delta_p = ", np.shape(y_delta_p))
    # print("y_delta_q = ", np.shape(y_delta_q))
    # print("yhat_delta_p = ", np.shape(yhat_delta_p))
    # print("yhat_delta_q = ", np.shape(yhat_delta_q))

    # print("---------------------------------------------")

    # print("gt_data_array = ", np.shape(gt_data_array))
    # print("imu_data_array = ", np.shape(imu_data_array))
    # print("y_delta_p_array = ", np.shape(y_delta_p_array))
    # print("y_delta_q_array = ", np.shape(y_delta_q_array))
    # print("yhat_delta_p_array = ", np.shape(yhat_delta_p_array))
    # print("yhat_delta_q_array = ", np.shape(yhat_delta_q_array))



    gt_data_array = np.vstack((gt_data_array,gt_data))
    imu_data_array = np.vstack((imu_data_array,imu_data))        
    y_delta_p_array = np.vstack((y_delta_p_array,np.array(y_delta_p)))
    y_delta_q_array = np.vstack((y_delta_q_array, np.array(y_delta_q))) 
    yhat_delta_p_array = np.vstack((yhat_delta_p_array,np.array(yhat_delta_p)))
    yhat_delta_q_array = np.vstack((yhat_delta_q_array, np.array(yhat_delta_q)))


   
if __name__ == '__main__':
    imu_pose = []
    imu_sensor_data = []
    gt_data_array = np.zeros((1,8))
    imu_data_array = np.zeros((1,7))
    y_delta_p_array = np.zeros((1,3))
    y_delta_q_array = np.array([1,0,0,0])   
    yhat_delta_p_array = np.zeros((1,3))
    pred_trajectory_array = np.zeros((1,3))
    gt_trajectory_array = np.zeros((1,3))
    yhat_delta_q_array = np.array([1,0,0,0]) 
    gt_pose_array = np.zeros((1,7))
    # init_p_first =  np.zeros((1,3))
    # init_q_first = np.array([1,0,0,0]) 

    first_loop_flag = False
    first_loop_flag2 = False


    rospy.init_node('gazebo_link_pose', anonymous=True)
    gp = GazeboLinkPose("k_crane::front_laser_link")


    window_size = 200
    stride = 10
    model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/data_crane_filter.hdf5"
    model = load_model(model)


    duration=3
    init_time_stamp = time.time()
    delta_time = 0

    pervious_time_1 = 0
    pervious_time_2 = 0
   
    ii =0
    
    # timer1 = rospy.Timer(rospy.Duration.from_sec(0.01), timer_get_imu_data)
    # if(delta_time >= duration):
    #   timer1.shutdown

  


    while  delta_time <= duration or rospy.is_shutdown():
     try:
        
        # # to publish the state with new topic name
        # gp.pose_pub.publish(gp.link_pose) 
        # # rospy.loginfo(gp.link_pose_rpy)
        # gp.pose_rpy_pub.publish(gp.link_pose_rpy)
        
        # current_time = time.time()
        now_time_stamp = time.time()
        delta_time = now_time_stamp - init_time_stamp


        if now_time_stamp >=  pervious_time_1 +  (0.01):
            pos = [delta_time, gp.link_pose.position.x,gp.link_pose.position.y,gp.link_pose.position.z,gp.link_pose.orientation.w, gp.link_pose.orientation.x, gp.link_pose.orientation.y, gp.link_pose.orientation.z]
            imu_pose.append(pos)
            w = gp.imu.angular_velocity
            acc =gp.imu.linear_acceleration
            sensor_data = [delta_time,w.x, w.y, w.z , acc.x, acc.y, acc.z ]
            imu_sensor_data.append(sensor_data)
            # print("duration = ", duration)
            print("delta_time", delta_time)
            print("================================================================", i)
            
        

            if (first_loop_flag2 == True):
                # print("publish_yhat_delta_p = ", publish_yhat_delta_p)
                publish_lidar_tf(publish_yhat_delta_p, publish_yhat_delta_q)
                static_transform_publisher("world", "base_link", 0,0,0)
                static_transform_publisher("base_link", "upper_link",0,0,0)
                static_transform_publisher("boom_link", "front_laser_link",0,0,0)
            pervious_time_1 = now_time_stamp 
            
        



        if now_time_stamp >=  pervious_time_2 +  (0.1) and (len(imu_pose) >= ii+ 202):

            gt_data = np.array(imu_pose)
            imu_data = np.array(imu_sensor_data)

            print("gt_data = ", np.shape(gt_data))
            print("imu_data = ", np.shape(imu_data))
            print("i  = ", ii)

            gt_data = gt_data[ii:ii+202, :]
            imu_data = imu_data[ii :ii+202, :]
            ii = ii+10

            # thread1 = threading.Thread(target = thread_filter_NN, args=(gt_data,imu_data ))
            # thread1.start()
            # thread1.join()

            print("gt_data = ", np.shape(gt_data))
            print("imu_data = ", np.shape(imu_data))


            time_array = imu_data[:,0]
            noisy_wx = imu_data[:,1]
            noisy_wy = imu_data[:,2]
            noisy_wz = imu_data[:,3]
            noisy_ax = imu_data[:,4]
            noisy_ay = imu_data[:,5]
            noisy_az = imu_data[:,6]


            filtered_wx, wx_noise_freq, wx_noise_amp  = apply_filter(time_array, noisy_wx,"wx")
            filtered_wy, wy_noise_freq, wy_noise_amp  = apply_filter(time_array, noisy_wy,"wy")
            filtered_wz, wz_noise_freq, wz_noise_amp  = apply_filter(time_array, noisy_wz,"wz")

            filtered_ax, ax_noise_freq, ax_noise_amp  = apply_filter(time_array, noisy_ax,"ax")
            filtered_ay, ay_noise_freq, ay_noise_amp  = apply_filter(time_array, noisy_ay,"ay")
            filtered_az, az_noise_freq, az_noise_amp  = apply_filter(time_array, noisy_az,"az")

            imu_data_filter =  np.vstack((time_array, filtered_wx, filtered_wy, filtered_wz, filtered_ax, filtered_ay, filtered_az))
            imu_data_filter = np.array(imu_data_filter)
            imu_data_filter = np.transpose(imu_data_filter)
                        

            gyro_data = interpolate_3dvector_linear(imu_data_filter[:, 1:4], imu_data_filter[:, 0], gt_data[:, 0])
            acc_data = interpolate_3dvector_linear(imu_data_filter[:, 4:7], imu_data_filter[:, 0], gt_data[:, 0])
            pos_data = gt_data[:, 1:4]
            ori_data = gt_data[:, 4:8]
            

            print("gyro_data = ", np.shape(gyro_data))
            print("acc_data = ", np.shape(acc_data))
            print("pos_data = ", np.shape(pos_data))
            print("ori_data = ", np.shape(ori_data))

          

    


            [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)
            
            if (first_loop_flag2 == False):
              publish_yhat_delta_p = init_p
              publish_yhat_delta_q = quaternion.from_float_array(init_q)
              init_p_first = init_p
              init_q_first = init_q

            first_loop_flag2 = True


            [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)
            first_loop_flag = True
            publish_yhat_delta_p = publish_yhat_delta_p + yhat_delta_p[0,:]
            publish_yhat_delta_q = publish_yhat_delta_q * quaternion.from_float_array(yhat_delta_q[0,:]).normalized()



            print("gt_data = ", np.shape(gt_data))
            print("imu_data = ", np.shape(imu_data))
            print("y_delta_p = ", np.shape(y_delta_p))
            print("y_delta_q = ", np.shape(y_delta_q))
            print("yhat_delta_p = ", np.shape(yhat_delta_p))
            print("yhat_delta_q = ", np.shape(yhat_delta_q))

            gt_data_array = np.vstack((gt_data_array,gt_data))
            imu_data_array = np.vstack((imu_data_array,imu_data))        
            y_delta_p_array = np.vstack((y_delta_p_array,np.array(y_delta_p)))
            y_delta_q_array = np.vstack((y_delta_q_array, np.array(y_delta_q))) 
            yhat_delta_p_array = np.vstack((yhat_delta_p_array,np.array(yhat_delta_p)))
            yhat_delta_q_array = np.vstack((yhat_delta_q_array, np.array(yhat_delta_q)))

            # print("---------------------------------------------")

            # print("gt_data_array = ", np.shape(gt_data_array))
            # print("imu_data_array = ", np.shape(imu_data_array))
            # print("y_delta_p_array = ", np.shape(y_delta_p_array))
            # print("y_delta_q_array = ", np.shape(y_delta_q_array))
            # print("yhat_delta_p_array = ", np.shape(yhat_delta_p_array))
            # print("yhat_delta_q_array = ", np.shape(yhat_delta_q_array))
     
            
                

            pervious_time_2 = now_time_stamp 
            # if(delta_time >= duration):
            #     timer1.shutdown



     except KeyboardInterrupt:
        break

    # print(gt_data_array)
    # print(imu_data_array)

    # timer1.shutdown

    y_delta_p = y_delta_p_array
    y_delta_q = y_delta_q_array
    yhat_delta_p = yhat_delta_p_array
    yhat_delta_q = yhat_delta_q_array

    gt_data_array = np.array(gt_data_array)
    imu_data_array = np.array(imu_data_array)
    np.savetxt('/home/aisl/catkin_ws/src/latest_crane/imu_pose/data_gt_end_AI.csv', gt_data_array, delimiter=',', fmt = '%.9f')
    np.savetxt('/home/aisl/catkin_ws/src/latest_crane/imu_pose/data_imu_end_AI.csv', imu_data_array, delimiter=',', fmt = '%.9f')

    print("gt_data_array = ", np.shape(gt_data_array))
    print("imu_data_array = ", np.shape(imu_data_array))
    print("y_delta_p_array = ", np.shape(y_delta_p_array))
    print("y_delta_q_array = ", np.shape(y_delta_q_array))
    print("yhat_delta_p_array = ", np.shape(yhat_delta_p_array))
    print("yhat_delta_q_array = ", np.shape(yhat_delta_q_array))

    # plot_fig("raw", gyro_data,acc_data, pos_data, ori_data)

    # plot_compare_pos("predict", "gt",  pos_data, ori_data, pos_data, ori_data)
    # plot_pos_error("predict error" ,  pos_data, ori_data, pos_data, ori_data)

    # plot_fig("gt", x_gyro,x_acc, y_delta_p, y_delta_q)


    n1 = 0
    n2 = len(yhat_delta_p)


    # plot_compare_pos("predict", "gt",  yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q)
    plot_pos_error("predict error" ,  yhat_delta_p, yhat_delta_q, y_delta_p, y_delta_q, n1, n2)
    # plot_compare_imu("given data", "change in data",  acc_data, gyro_data, x_acc, x_gyro)
    # plot_fig("perdict", x_gyro,x_acc, yhat_delta_p, yhat_delta_q)

    gt_trajectory_end  = generate_trajectory_6d_quat(init_p_first, init_q_first, y_delta_p, y_delta_q)
    pred_trajectory_end  = generate_trajectory_6d_quat(init_p_first, init_q_first, yhat_delta_p, yhat_delta_q)

    error_traj = gt_trajectory_end - pred_trajectory_end


    fig = plt.figure("Error in Traj xyz axes")
    plt.subplot(311); plt.plot(error_traj[:,0]); plt.grid(True); plt.title(" pos x");  
    plt.subplot(312); plt.plot(error_traj[:,1]); plt.grid(True); plt.title(" pos y"); 
    plt.subplot(313); plt.plot(error_traj[:,2]); plt.grid(True); plt.title(" pos z");   



    # # fig = plt.figure("gt trajectory"); 
    # fig = plt.figure(figsize=[14.4, 10.8]); #plt.title("gt trajectory")
    # ax = fig.gca(projection='3d')
    # ax.plot(gt_trajectory[:, 0], gt_trajectory[:, 1], gt_trajectory[:, 2])
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
    # ax.legend(['gt trajectory'], loc='upper right')

    # # fig = plt.figure("pred trajectory"); plt.title("pred trajectory")
    # fig = plt.figure(figsize=[14.4, 10.8])
    # ax = fig.gca(projection='3d')
    # ax.plot(pred_trajectory[:, 0], pred_trajectory[:, 1], pred_trajectory[:, 2])
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
    # ax.legend(['pred trajectory'], loc='upper right')



    # # fig = plt.figure("Error in Traj"); plt.title("Error in Traj")
    # fig = plt.figure(figsize=[14.4, 10.8])
    # ax = fig.gca(projection='3d')
    # ax.plot(error_traj[:, 0], error_traj[:, 1], error_traj[:, 2])
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
    # ax.legend(['Error in Traj'], loc='upper right')


    fig = plt.figure("pred trajectory end"); plt.title("pred trajectory end")
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(gt_trajectory_end[:, 0], gt_trajectory_end[:, 1], gt_trajectory_end[:, 2])
    ax.plot(pred_trajectory_end[:, 0], pred_trajectory_end[:, 1], pred_trajectory_end[:, 2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    min_x = np.minimum(np.amin(gt_trajectory_end[:, 0]), np.amin(pred_trajectory_end[:, 0]))
    min_y = np.minimum(np.amin(gt_trajectory_end[:, 1]), np.amin(pred_trajectory_end[:, 1]))
    min_z = np.minimum(np.amin(gt_trajectory_end[:, 2]), np.amin(pred_trajectory_end[:, 2]))
    max_x = np.maximum(np.amax(gt_trajectory_end[:, 0]), np.amax(pred_trajectory_end[:, 0]))
    max_y = np.maximum(np.amax(gt_trajectory_end[:, 1]), np.amax(pred_trajectory_end[:, 1]))
    max_z = np.maximum(np.amax(gt_trajectory_end[:, 2]), np.amax(pred_trajectory_end[:, 2]))
    range_x = np.absolute(max_x - min_x)
    range_y = np.absolute(max_y - min_y)
    range_z = np.absolute(max_z - min_z)
    max_range = np.maximum(np.maximum(range_x, range_y), range_z)
    ax.set_xlim(min_x, min_x + max_range)
    ax.set_ylim(min_y, min_y + max_range)
    ax.set_zlim(min_z, min_z + max_range)
    ax.legend(['ground truth', 'predicted'], loc='upper right')










    # matplotlib.rcParams.update({'font.size': 18})
    # fig = plt.figure(figsize=[14.4, 10.8])
    # ax = fig.gca(projection='3d')
    # ax.plot(gt_trajectory[1:, 0], gt_trajectory[1:, 1], gt_trajectory[1:, 2])
    # ax.plot(pred_trajectory[1:, 0], pred_trajectory[1:, 1], pred_trajectory[1:, 2])
    # ax.set_xlabel('X (m)')
    # ax.set_ylabel('Y (m)')
    # ax.set_zlabel('Z (m)')
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
    # ax.legend(['ground truth', 'predicted'], loc='upper right')
    plt.show()
