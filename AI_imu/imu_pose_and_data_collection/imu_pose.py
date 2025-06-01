#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose
import numpy as np
from numpy import savetxt
from sensor_msgs.msg import Imu
import time
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])






class GazeboLinkPose:
  # line name to check its currect state
  link_name = '/gazebo/link_states'
  link_pose = Pose()
  imu = Imu()
  link_pose_rpy = Float32MultiArray()

  
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

if __name__ == '__main__':
  # try:
  rospy.init_node('gazebo_link_pose_node')
  gp = GazeboLinkPose("k_crane::front_laser_link")
  boom_state = GazeboLinkPose("k_crane::boom_link")

  duration=60*60*2
  # duration=6
  frequency_save_pose=1 # 100hz
  rate = rospy.Rate(frequency_save_pose)
  delta_time =0      

  # there are 3 posibilities to get current time. one is from ros (rospy.get_time()), 2nd is imu time (imu.header.stamp), third is python time time.time()
  # the python time is correct other time are not working.

  # init_time_stamp = rospy.get_time()
  # now_time_stamp = rospy.get_time()
  # init_time_stamp = gp.imu.header.stamp.to_sec()
  # now_time_stamp = gp.imu.header.stamp.to_sec()
  
  
  i = 0
  imu_pose = []
  imu_sensor_data = []
  pervious_time = 0



  init_time_stamp = time.time()
  now_time_stamp = time.time()
  delta_time = now_time_stamp - init_time_stamp





  while  delta_time <= duration  and not rospy.is_shutdown():
    try:
      
      # to publish the state with new topic name
      gp.pose_pub.publish(gp.link_pose) 
      # rospy.loginfo(gp.link_pose_rpy)

      gp.pose_rpy_pub.publish(gp.link_pose_rpy)
      
      # current_time = time.time()
      now_time_stamp = time.time()
      delta_time = now_time_stamp - init_time_stamp


      if now_time_stamp >=  pervious_time +  (0.01):
        pos = [delta_time, gp.link_pose.position.x,gp.link_pose.position.y,gp.link_pose.position.z,gp.link_pose.orientation.w, gp.link_pose.orientation.x, gp.link_pose.orientation.y, gp.link_pose.orientation.z]
        imu_pose.append(pos)
        w = gp.imu.angular_velocity
        acc =gp.imu.linear_acceleration
        sensor_data = [delta_time,w.x, w.y, w.z , acc.x, acc.y, acc.z ]
        imu_sensor_data.append(sensor_data)
        print("duration = ", duration)
        print("delta_time", delta_time)
        print("================================================================", i)
        i = i+1
        pervious_time = now_time_stamp 
       
    
    except KeyboardInterrupt:
       break
    
      
    # except rospy.ROSInterruptException:
    #   rospy.loginfo("Stopping (Ctrl-C caught)")
    #   break

  # except KeyboardInterrupt:
  #   pass


  imu_pose = np.array(imu_pose)
  imu_sensor_data = np.array(imu_sensor_data)
  # print("imu_pose = ",imu_pose)
  
  np.savetxt('/mah/AI/imu_pose/4_15_data_gt_simple_motion_ros_rate_2.csv', imu_pose, delimiter=',', fmt = '%.9f')
  np.savetxt('/mah/AI/imu_pose/4_15_data_imu_simple_motion_ros_rate_2.csv', imu_sensor_data, delimiter=',', fmt = '%.9f')
  

  # except rospy.ROSInterruptException:
  #   pass

