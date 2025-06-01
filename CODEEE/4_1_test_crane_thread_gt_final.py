import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from tensorflow.keras.models import load_model

# from dataset_online import *
from dataset import *
from util import *
from plot_fig import *
from low_pass_filter_function import *
np.set_printoptions(formatter={'float': '{: 0.3f}'.format})



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
first_loop_flag = False
imu_data_array = np.zeros((1,7))
gt_trj_p_array = np.zeros((1,4))
gt_trj_q_array = np.array([1,0,0,0]) 
pred_trj_p_array = np.zeros((1,4))
pred_trj_q_array = np.array([1,0,0,0]) 
gt_slice_array =  np.zeros((1,8))
y_delta_p_array = np.zeros((1,3))
y_delta_q_array = np.array([1,0,0,0]) 
yhat_delta_p_array = np.zeros((1,3))
yhat_delta_q_array = np.array([1,0,0,0]) 

# publish_gt_pos = np.zeros((1,3))
# publish_gt_ori = np.array([1,0,0,0]) 


window_size = 50
stride = 50
delta_time = 0
sampling_time = 0.01
model = "/mah/AI/6-DOF-Inertial-Odometry_orignal/win50/4_1_simple_motion_win_50_dt_0.01_model_ep_800_data_filter.hdf5"
model = load_model(model)




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
    # self.pose_pub = rospy.Publisher("/gazebo/" + self.link_name_rectified, Pose, queue_size = 10)
    # self.pose_rpy_pub = rospy.Publisher("/gazebo/k_crane_front_laser_link/rpy", Float32MultiArray, queue_size=1)


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




def publish_lidar_tf_pred(yhat_delta_p, yhat_delta_q):

  # yhat_delta_q = yhat_delta_q / np.linalg.norm(yhat_delta_q)
  yhat_delta_q = quaternion.as_float_array(yhat_delta_q)
  br = tf2_ros.TransformBroadcaster()
  t = geometry_msgs.msg.TransformStamped()
  

  t.header.stamp = rospy.Time.now()
  t.header.frame_id = "base_link"
  t.child_frame_id = "front_laser_link_pd"
  t.transform.translation.x =  yhat_delta_p[0]
  t.transform.translation.y =  yhat_delta_p[1]
  t.transform.translation.z =  yhat_delta_p[2]
  
  t.transform.rotation.w = yhat_delta_q[0]
  t.transform.rotation.x = yhat_delta_q[1]
  t.transform.rotation.y = yhat_delta_q[2]
  t.transform.rotation.z = yhat_delta_q[3]
  br.sendTransform(t)


def publish_lidar_tf_gazebo_real(yhat_delta_p, yhat_delta_q):

  # yhat_delta_q = yhat_delta_q / np.linalg.norm(yhat_delta_q)
#   yhat_delta_q = quaternion.as_float_array(yhat_delta_q)
  br = tf2_ros.TransformBroadcaster()
  t = geometry_msgs.msg.TransformStamped()
  

  t.header.stamp = rospy.Time.now()
  t.header.frame_id = "base_link"
  t.child_frame_id = "front_laser_link_gt"
  t.transform.translation.x =  yhat_delta_p.x
  t.transform.translation.y =  yhat_delta_p.y
  t.transform.translation.z =  yhat_delta_p.z
  
  t.transform.rotation.w = yhat_delta_q.w
  t.transform.rotation.x = yhat_delta_q.x
  t.transform.rotation.y = yhat_delta_q.y
  t.transform.rotation.z = yhat_delta_q.z
  br.sendTransform(t)


def publish_lidar_tf_real(yhat_delta_p, yhat_delta_q):

  # yhat_delta_q = yhat_delta_q / np.linalg.norm(yhat_delta_q)
  yhat_delta_q = quaternion.as_float_array(yhat_delta_q)
  br = tf2_ros.TransformBroadcaster()
  t = geometry_msgs.msg.TransformStamped()
  

  t.header.stamp = rospy.Time.now()
  t.header.frame_id = "base_link"
  t.child_frame_id = "front_laser_link"
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


def get_imu_data():

    # global pervious_time_1
    
    now_time_stamp1 = time.time()
    init_time_stamp = time.time()
    delta_time1 = now_time_stamp1 - init_time_stamp
    pervious_time_1 = 0.0
    global imu_pose
    global imu_sensor_data
    global publish_gt_pos
    global publish_gt_ori
     # to publish the state with new topic name
    # gp.pose_pub.publish(gp.link_pose) 
    # rospy.loginfo(gp.link_pose_rpy)
    # gp.pose_rpy_pub.publish(gp.link_pose_rpy)

    while  delta_time1 <= duration and not rospy.is_shutdown():
     try:
        now_time_stamp1 = time.time()
        delta_time1 = time.time() - init_time_stamp

        if now_time_stamp1 >=  pervious_time_1 +  sampling_time:
            pos = [delta_time1, gp.link_pose.position.x,gp.link_pose.position.y,gp.link_pose.position.z,gp.link_pose.orientation.w, gp.link_pose.orientation.x, gp.link_pose.orientation.y, gp.link_pose.orientation.z]
            publish_gt_pos = gp.link_pose.position
            publish_gt_ori = gp.link_pose.orientation
            imu_pose.append(pos)
            w = gp.imu.angular_velocity
            acc =gp.imu.linear_acceleration
            sensor_data = [delta_time1,w.x, w.y, w.z , acc.x, acc.y, acc.z ]
            imu_sensor_data.append(sensor_data)
            pervious_time_1 = now_time_stamp1 
            # print("delta_time1", delta_time1)
            # print("================================================================")
            # print("fun len(imu_pose) = ", len(imu_pose))


     except KeyboardInterrupt:
        break


def thread_filter_NN(gt_data, imu_data):
    global publish_yhat_delta_p
    global publish_yhat_delta_q
    global publish_y_delta_p
    global publish_y_delta_q
    global init_p_first 
    global init_q_first
    global first_loop_flag
    global imu_data_array
    global gt_trj_p_array
    global gt_trj_q_array
    global pred_trj_p_array
    global pred_trj_q_array
    global gt_trj_p
    global gt_trj_q  
    global pred_trj_p
    global pred_trj_q
    global yhat_delta_p_array
    global yhat_delta_q_array
    global y_delta_p_array
    global y_delta_q_array
 
  

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
                

    # gyro_data = interpolate_3dvector_linear(imu_data_filter[:, 1:4], imu_data_filter[:, 0], gt_data[:, 0])
    # acc_data = interpolate_3dvector_linear(imu_data_filter[:, 4:7], imu_data_filter[:, 0], gt_data[:, 0])
    gyro_data = interpolate_3dvector_linear(imu_data_filter[:, 1:4], imu_data_filter[:, 0], gt_data[:, 0])
    acc_data = interpolate_3dvector_linear(imu_data_filter[:, 4:7], imu_data_filter[:, 0], gt_data[:, 0])
    pos_data = gt_data[:, 1:4]
    ori_data = gt_data[:, 4:8]
    

    [x_gyro, x_acc], [y_delta_p, y_delta_q], init_p, init_q = load_dataset_6d_quat(gyro_data, acc_data, pos_data, ori_data, window_size, stride)
   
    if first_loop_flag == False:
        init_p_first = init_p
        init_q_first = init_q
        gt_trj_p = init_p
        gt_trj_q = quaternion.from_float_array(init_q)
        pred_trj_p = init_p
        pred_trj_q = quaternion.from_float_array(init_q)

    gt_trj_p, gt_trj_q = generate_trajectory_6d_quat_thread(gt_trj_p, gt_trj_q, y_delta_p, y_delta_q[0,:])
    publish_y_delta_p = gt_trj_p[0,:]
    publish_y_delta_q = gt_trj_q
    delta_time = time.time() - init_time_stamp
    gt_trj_p_time[0] = delta_time 
    gt_trj_p_time[1:] = np.array( gt_trj_p[0,:])
    # print("cur_p_gt  ", np.array(gt_trj_p_time))

    dgt_q = quaternion.as_float_array(gt_trj_q)
    dgt_q = ( dgt_q[1], dgt_q[2], dgt_q[3], dgt_q[0])
    gt_trj_q_euler = euler_from_quaternion(dgt_q,  axes='sxyz')
    # y_delta_q_euler = euler_from_quaternion(y_delta_q[0,:])
    # y_delta_q_euler = y_delta_q[0,:]
    # print("cur_p_gt  ", np.array(gt_trj_p_time), (quaternion.as_euler_angles(quaternion.from_float_array(y_delta_q_euler))), np.array(y_delta_q_euler))
    print("cur_p_gt  ", np.array(gt_trj_p_time), np.array(gt_trj_q_euler)*180/math.pi , np.array(dgt_q))

       
    # yhat_delta_p =  y_delta_p
    # yhat_delta_q = y_delta_q
    [yhat_delta_p, yhat_delta_q] = model.predict([x_gyro, x_acc], batch_size=1, verbose=1)
    pred_trj_p, pred_trj_q = generate_trajectory_6d_quat_thread(pred_trj_p, pred_trj_q, yhat_delta_p, yhat_delta_q[0,:])
    publish_yhat_delta_p = pred_trj_p[0,:]
    publish_yhat_delta_q = pred_trj_q
    delta_time = time.time() - init_time_stamp
    pred_trj_p_time[0] = delta_time 
    pred_trj_p_time[1:] = np.array( pred_trj_p[0,:])
    # print("cur_p_prd ", np.array(pred_trj_p_time))

    first_loop_flag = True


    

    # imu_data_array = np.vstack((imu_data_array,imu_data))        
    gt_trj_p_array = np.vstack((gt_trj_p_array, gt_trj_p_time))
    gt_trj_q_array = np.vstack((gt_trj_q_array,  quaternion.as_float_array(gt_trj_q)))
    pred_trj_p_array = np.vstack((pred_trj_p_array, pred_trj_p_time))
    pred_trj_q_array = np.vstack((pred_trj_q_array,  quaternion.as_float_array(pred_trj_q)))
    yhat_delta_p_array = np.vstack((yhat_delta_p_array, yhat_delta_p))
    yhat_delta_q_array = np.vstack((yhat_delta_q_array, yhat_delta_q))
    y_delta_p_array = np.vstack((y_delta_p_array, y_delta_p))
    y_delta_q_array = np.vstack((y_delta_q_array, y_delta_q))
    



  


   
if __name__ == '__main__':

    global publish_yhat_delta_p
    global publish_yhat_delta_q
    
    global publish_y_delta_p
    global publish_y_delta_q
    global gt_trj_p
    global gt_trj_q

    # rospy.init_node('online_crane_mapping_node', anonymous=True)
    rospy.init_node('online_crane_mapping_node')

    gp = GazeboLinkPose("k_crane::front_laser_link")
    rospy.sleep(2)
  
  
    pervious_time = 0
    pervious_time_2 = 0
    ii =0
   
    gt_trj_p_time = [0,0,0,0]
    pred_trj_p_time = [0,0,0,0]
    init_time_stamp = time.time()
    imu_pose = []
    imu_sensor_data = []



    duration=30

    # get_imu_data_thread = threading.Thread(target = get_imu_data)
    # get_imu_data_thread.start()


    # init_time_stamp = time.time()

    while  delta_time <= duration and not rospy.is_shutdown():
     try:
        
        # # to publish the state with new topic name
        # gp.pose_pub.publish(gp.link_pose) 
        # # rospy.loginfo(gp.link_pose_rpy)
        # gp.pose_rpy_pub.publish(gp.link_pose_rpy)
        
        # current_time = time.time()
        now_time_stamp = time.time()
        # delta_time = now_time_stamp - init_time_stamp
        # print("len(imu_pose) = ", len(imu_pose))

        

        if now_time_stamp >=  pervious_time + sampling_time:
            

            delta_time = time.time()- init_time_stamp
            pos = [delta_time, gp.link_pose.position.x,gp.link_pose.position.y,gp.link_pose.position.z,gp.link_pose.orientation.w, gp.link_pose.orientation.x, gp.link_pose.orientation.y, gp.link_pose.orientation.z]
            imu_pose.append(pos)
            publish_gt_pos = gp.link_pose.position
            publish_gt_ori = gp.link_pose.orientation
            w = gp.imu.angular_velocity
            acc =gp.imu.linear_acceleration
            sensor_data = [delta_time,w.x, w.y, w.z , acc.x, acc.y, acc.z ]
            imu_sensor_data.append(sensor_data)

            pervious_time = now_time_stamp 



            # print("duration = ", duration)
            # print("delta_time", delta_time)
            # print("================================================================", i)

            if first_loop_flag == True:

                # publish_lidar_tf_real(publish_y_delta_p, publish_y_delta_q)         
                # publish_lidar_tf_gazebo_real(publish_gt_pos,publish_gt_ori )
                # publish_lidar_tf_pred(publish_yhat_delta_p, publish_yhat_delta_q)
                # static_transform_publisher("world", "base_link", 0,0,0)

                publish_tf_thread1 = publish_thread = threading.Thread(target = static_transform_publisher, args=("world", "base_link", 0,0,0))
                publish_tf_thread2 = publish_thread = threading.Thread(target = publish_lidar_tf_pred, args=(publish_yhat_delta_p, publish_yhat_delta_q))
                publish_tf_thread3 = publish_thread = threading.Thread(target = publish_lidar_tf_gazebo_real, args=(publish_gt_pos,publish_gt_ori))
                publish_tf_thread4 = publish_thread = threading.Thread(target = publish_lidar_tf_real, args=(publish_y_delta_p, publish_y_delta_q))
                publish_tf_thread1.start()
                publish_tf_thread2.start()
                publish_tf_thread3.start()
                publish_tf_thread4.start()




        if (len(imu_pose) > ii+ window_size+2):
        # if (len(imu_pose) > ii+ window_size):
       
            gt_data = np.array(imu_pose)
            imu_data = np.array(imu_sensor_data)


            print("===========================================================", ii)
          
            # gt_slice = gt_data[ii+100, :]
            gt_slice = gt_data[ii+window_size, :]
            gt_slice_array = np.vstack((gt_slice_array,np.array(gt_slice)))
            # print("gt_slice  ", gt_slice[:4])

            gt_slice1 = (gt_slice[5], gt_slice[6], gt_slice[7], gt_slice[4])
            gt_slice_euler = euler_from_quaternion(gt_slice1, axes='sxyz')
            # gt_euler = (gt_slice[4:])
            # print("gt_slice  ", gt_slice[:4], quaternion.as_euler_angles(quaternion.from_float_array(gt_euler).normalized()), np.array(gt_euler))
            print("gt_slice  ", gt_slice[:4], np.array(gt_slice_euler)*180/math.pi , np.array(gt_slice1))



            gt_data1 = gt_data[ii:ii + window_size+2, :]
            imu_data = imu_data[ii :ii + window_size+2, :]
            ii = ii+ stride


            # if (first_loop_flag == True):
            #   thread1.join()
            # thread1 = threading.Thread(target = thread_filter_NN, args=(gt_data1,imu_data ))
            # thread1.start()
            # thread1.join()

            thread_filter_NN(gt_data1,imu_data )     
            # pervious_time_2 = now_time_stamp 
       
     except KeyboardInterrupt:
        break



    # thread1.join()

    if (delta_time > duration) :

        imu_data_array = np.array(imu_data_array)
        gt_trj_p = gt_trj_p_array
        gt_trj_q = gt_trj_q_array
        pred_trj_p = pred_trj_p_array
        pred_trj_q = pred_trj_q_array
        pred_trajectory = pred_trj_p_array
        gt_trajectory = gt_trj_p_array
        yhat_delta_p = yhat_delta_p_array
        yhat_delta_q = yhat_delta_q_array
        y_delta_p = y_delta_p_array
        y_delta_q = y_delta_q_array




        # print("gt_trj_p = ", np.shape(gt_trj_p))
        # print("gt_data = ", np.shape(gt_data))

        # if(len(gt_slice_array) > len(gt_trj_p)):
        #     gt_slice_array = gt_slice_array[0:len(gt_trj_p),:]
        # elif(len(gt_slice_array) < len(gt_trj_p)):
        #     gt_trj_p = gt_trj_p[0:len(gt_slice_array),:]
                    
        # print("gt_trj_p = ", np.shape(gt_trj_p))
        # print("gt_slice_array = ", np.shape(gt_slice_array))

        # gt_slice_array = gt_data[1:len(gt_data):10]

        # print("gt_trj_p = ", np.shape(gt_trj_p))
        # print("gt_slice_array = ", np.shape(gt_slice_array))

        imu_data = np.array(imu_sensor_data)

     
        np.savetxt('/mah/AI/imu_pose/gt_data.csv', gt_data, delimiter=',', fmt = '%.9f')
        np.savetxt('/mah/AI/imu_pose/imu_data.csv', imu_data, delimiter=',', fmt = '%.9f')



        gt_trajectory_end  = generate_trajectory_6d_quat(init_p_first, init_q_first, y_delta_p, y_delta_q)
        pred_trajectory_end = generate_trajectory_6d_quat(init_p_first, init_q_first, yhat_delta_p, yhat_delta_q)




        gyro_data = interpolate_3dvector_linear(imu_data[:, 1:4], imu_data[:, 0], gt_data[:, 0])
        acc_data = interpolate_3dvector_linear(imu_data[:, 4:7], imu_data[:, 0], gt_data[:, 0])
        pos_data = np.array(gt_data[:, 1:4])
        ori_data = np.array(gt_data[:, 4:8])
             


        # plot_compare_pos("predict", "gt",  pos_data, ori_data, pos_data, ori_data)
        # plot_pos_error("predict error" ,  pos_data, ori_data, pos_data, ori_data)

        # plot_fig("gt", x_gyro,x_acc, y_delta_p, y_delta_q)


        


        # plot_compare_pos("predict trj", "gt trj",  pred_trj_p, pred_trj_q, gt_trj_p, gt_trj_q)
        # plot_compare_pos("predict", "gt",  y_delta_p, y_delta_q, yhat_delta_p, yhat_delta_q)

        n1 = 0
        n2 = len(gt_trj_p)

        if len(gt_slice_array)>= len(gt_trj_q):
         nn = len(gt_trj_q)
        elif len(gt_slice_array)<= len(gt_trj_q):
          nn = len(gt_slice_array)

        plot_pos_error_gazebo("dgt vs gt error " ,  gt_trj_p[1:nn,1:], gt_trj_q[1:nn,:], gt_slice_array[1:nn,1:4], gt_slice_array[1:nn,4:], n1, n2)

        plot_fig_gazebo("raw", gyro_data,acc_data, pos_data, ori_data, n1, len(ori_data))
        plot_fig_gazebo("raw 2 ", gyro_data,acc_data, gt_slice_array[1:nn,1:4], gt_slice_array[1:nn,4:], n1, n2)


        # #down
        # plot_pos_error("predict trj error delta" ,  pred_trj_p[:,1:], pred_trj_q, gt_trj_p[:,1:], gt_trj_q, n1, n2)
        # n2 = len(y_delta_p)
        # plot_pos_error("predict Yhat error delta" ,  y_delta_p, y_delta_q, yhat_delta_p, yhat_delta_q, n1, n2)


        # plot_pos_error("predict error " ,  publish_y_delta_p_array, publish_y_delta_q_array, gt_data_array[1:3,:], gt_data_array[4:,:], n1, n2)
        # plot_compare_imu("given data", "change in data",  acc_data, gyro_data, x_acc, x_gyro)
        # plot_fig("perdict", x_gyro,x_acc, yhat_delta_p, yhat_delta_q)
        
        
 

        

        Err_gt_dgt =  gt_slice_array[:,1:4] -gt_trj_p[:,1:]
        Err_gt_pred = gt_slice_array[:,1:4] - pred_trj_p[:,1:] 
        Err_gt_pred_time_array = np.zeros(3)

        print("gt_slice_array = ", np.shape(gt_slice_array))
        print("pred_trj_p = ", np.shape(pred_trj_p))
        print("gt_trj_p = ", np.shape(gt_trj_p))
        # Err_gt_dgt_end = delta_gt_trajectory_end[1:,:] - gt_slice_array[:,1:4]
        jj = 0

        # for kk in range(0, len(gt_slice_array)):
        #     if round(pred_trj_p[jj,0], 1) == round(gt_slice_array[kk,0],2):
        #         Err_gt_pred_time = pred_trj_p[jj,1:] - gt_slice_array[kk,1:4]
        #         Err_gt_pred_time_array = np.vstack((Err_gt_pred_time_array,Err_gt_pred_time))
        #         jj =jj+1


        print(" gt_slice_array[:,1:4] = ",  np.shape(gt_slice_array[:,1:4]))
        print("gt_slice_array[:,5:] = ", np.shape(gt_slice_array[:,4:]))

        # plot_pos_error("gt vs delta gt" ,  gt_slice_array[:,1:4], gt_slice_array[:,4:], gt_trj_p[:,1:4], gt_trj_q, n1, n2)


        fig = plt.figure("Time error")
        plt.subplot(321); plt.plot(gt_trj_p[1:,0], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(323); plt.plot(gt_trj_p[1:,0], label='delta gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(325); plt.plot(gt_trj_p[1:,0], label='delta gt'); plt.grid(True); plt.title( " pos z");   plt.legend()


        plt.subplot(321); plt.plot(gt_slice_array[1:,0],  label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(323); plt.plot(gt_slice_array[1:,0],  label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(325); plt.plot(gt_slice_array[1:,0],  label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(321); plt.plot(pred_trj_p[1:,0],  label='pred'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(323); plt.plot(pred_trj_p[1:,0],  label='pred'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(325); plt.plot(pred_trj_p[1:,0],  label='pred'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(322); plt.plot(gt_trj_p[1:,0]- gt_slice_array[1:,0],  label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(324); plt.plot(gt_trj_p[1:,0]- gt_slice_array[1:,0],  label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(326); plt.plot(gt_trj_p[1:,0]- gt_slice_array[1:,0],  label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(322); plt.plot(pred_trj_p[1:,0]- gt_slice_array[1:,0],  label='pred'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(324); plt.plot(pred_trj_p[1:,0]- gt_slice_array[1:,0],  label='pred'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(326); plt.plot(pred_trj_p[1:,0]- gt_slice_array[1:,0],  label='pred'); plt.grid(True); plt.title( " pos z");   plt.legend()


        fig = plt.figure("pos and ori one")
        plt.subplot(321); plt.plot(gt_trj_p[1:,0], gt_trj_p[1:,1], label='delta gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(323); plt.plot(gt_trj_p[1:,0], gt_trj_p[1:,2], label='delta gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(325); plt.plot(gt_trj_p[1:,0], gt_trj_p[1:,3], label='delta gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(321); plt.plot(gt_slice_array[1:,0], gt_slice_array[1:,1], label='gt'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(323); plt.plot(gt_slice_array[1:,0], gt_slice_array[1:,2], label='gt'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(325); plt.plot(gt_slice_array[1:,0], gt_slice_array[1:,3], label='gt'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(321); plt.plot(pred_trj_p[1:,0], pred_trj_p[1:,1], label='pred'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(323); plt.plot(pred_trj_p[1:,0], pred_trj_p[1:,2], label='pred'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(325); plt.plot(pred_trj_p[1:,0], pred_trj_p[1:,3], label='pred'); plt.grid(True); plt.title( " pos z");   plt.legend()

        # plt.subplot(321); plt.plot(delta_gt_trajectory_end[1:,0], label='delta gt end'); plt.grid(True); plt.title(" pos x");   plt.legend()
        # plt.subplot(323); plt.plot(delta_gt_trajectory_end[1:,1], label='delta gt end'); plt.grid(True); plt.title( " pos y");   plt.legend()
        # plt.subplot(325); plt.plot(delta_gt_trajectory_end[1:,2], label='delta gt end'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(322); plt.plot(gt_slice_array[1:,0],Err_gt_dgt[1:,0], label='Err gt-dgt'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(324); plt.plot(gt_slice_array[1:,0],Err_gt_dgt[1:,1], label='Err gt-dgt'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(326); plt.plot(gt_slice_array[1:,0],Err_gt_dgt[1:,2], label='Err gt-dgt'); plt.grid(True); plt.title( " pos z");   plt.legend()

        plt.subplot(322); plt.plot(gt_slice_array[1:,0],Err_gt_pred[1:,0], label='Err gt-pred'); plt.grid(True); plt.title(" pos x");   plt.legend()
        plt.subplot(324); plt.plot(gt_slice_array[1:,0],Err_gt_pred[1:,1], label='Err gt-pred'); plt.grid(True); plt.title( " pos y");   plt.legend()
        plt.subplot(326); plt.plot(gt_slice_array[1:,0],Err_gt_pred[1:,2], label='Err gt-pred'); plt.grid(True); plt.title( " pos z");   plt.legend()

        # plt.subplot(322); plt.plot(Err_gt_dgt_end[1:,0], label='Err gt-dgt-end'); plt.grid(True); plt.title(" pos x");   plt.legend()
        # plt.subplot(324); plt.plot(Err_gt_dgt_end[1:,1], label='Err gt-dgt-end'); plt.grid(True); plt.title( " pos y");   plt.legend()
        # plt.subplot(326); plt.plot(Err_gt_dgt_end[1:,2], label='Err gt-dgt-end'); plt.grid(True); plt.title( " pos z");   plt.legend()

        


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
        

        
        fig = plt.figure(figsize=[14.4, 10.8]); 
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
        fig = plt.figure(figsize=[14.4, 10.8])
        ax = fig.gca(projection='3d')
        ax.plot(gt_trajectory[1:, 1], gt_trajectory[1:, 2], gt_trajectory[1:, 3])
        ax.plot(pred_trajectory[1:, 1], pred_trajectory[1:, 2], pred_trajectory[1:, 3])
        ax.plot(gt_trajectory_end[:, 0], gt_trajectory_end[:, 1], gt_trajectory_end[:, 2])
        ax.plot(pred_trajectory_end[:, 0], pred_trajectory_end[:, 1], pred_trajectory_end[:, 2])
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
        ax.legend(['ground truth', 'predicted', 'ground truth end', 'predicted end'], loc='upper right')

        plt.ticklabel_format(style='plain')    # to prevent scientific notation.
        plt.show()
