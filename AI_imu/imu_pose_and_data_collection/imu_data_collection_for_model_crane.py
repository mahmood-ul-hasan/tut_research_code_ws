#!/usr/bin/env python3
import matplotlib
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
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

from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation


array = np.array([0.0, 0.0, 0.0, 0.0, 0.0])


class colect_data:
    # line name to check its currect state
    odom = Pose()
    imu = Imu()
    odom_rpy = Float32MultiArray()

    def __init__(self):

        self.odom_sub = rospy.Subscriber("/camera/odom/sample", Odometry, self.callback)
        self.imu_sub = rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
        self.pose_rpy_pub = rospy.Publisher("/camera/odom/sample_rpy", Float32MultiArray, queue_size=1)
        self.odom_received = False
        self.imu_received = False

    def imu_callback(self, data):
        try:
            self.imu = data
            self.imu_time_stamp = data.header.stamp.to_sec()  # Store the timestamp from the IMU message
            self.imu_received = True

        except ValueError:
            pass

    def callback(self, data):
        try:
            global i
            self.odom = data.pose.pose
            self.odom_time_stamp = data.header.stamp.to_sec()  # Store the timestamp from the IMU message
            self.odom_received = True

            # Convert quat to rpy
            quat_upper = (
                self.odom.orientation.x,
                self.odom.orientation.y,
                self.odom.orientation.z,
                self.odom.orientation.w)
            euler_upper = euler_from_quaternion(quat_upper)
            # publish rpy
            array[0] = euler_upper[0] * 180 / math.pi
            array[1] = euler_upper[1] * 180 / math.pi
            array[2] = euler_upper[2] * 180 / math.pi
            array[3] = i
            self.odom_rpy = Float32MultiArray(data=array)
            # print("self.odom_rpy = ", self.odom_rpy)
            i = i + 1
        except ValueError:
            pass


if __name__ == '__main__':
    # try:
    rospy.init_node('odom_node')
    gp = colect_data()
    duration = 70
    # duration=6
    frequency_save_pose = 1  # 100hz
    rate = rospy.Rate(frequency_save_pose)
    delta_time = 0

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

    while delta_time <= duration and not rospy.is_shutdown():
        # current_time = time.time()
        now_time_stamp = time.time()
        delta_time = now_time_stamp - init_time_stamp

        if gp.odom_received and gp.imu_received:
            try:

                # to publish the state with new topic name

                gp.pose_rpy_pub.publish(gp.odom_rpy)
                print(" t ", delta_time)

                if now_time_stamp >= pervious_time + (0.0001):

                    pos = [delta_time, gp.odom.position.x, gp.odom.position.y, gp.odom.position.z, gp.odom.orientation.w, gp.odom.orientation.x, gp.odom.orientation.y, gp.odom.orientation.z]
                    print("pos:", pos)
                    imu_pose.append(pos)
                    w = gp.imu.angular_velocity
                    acc = gp.imu.linear_acceleration
                    imu_time_stamp = gp.imu_time_stamp
                    sensor_data = [delta_time, w.x, w.y, w.z, acc.x, acc.y, acc.z]
                    print("sensor_data:", sensor_data)
                    imu_sensor_data.append(sensor_data)

                    print("duration = ", duration)
                    print("delta_time", delta_time)
                    print("================================================================", i)

                    i = i + 1
                    pervious_time = now_time_stamp
                    gp.odom_received = False
                    gp.imu_received = False

            except KeyboardInterrupt:
                break

    imu_pose = np.array(imu_pose)
    imu_sensor_data = np.array(imu_sensor_data)
    print("imu_pose = ", imu_pose)

    np.savetxt('/home/aisl2/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods/random_1_gt.csv', imu_pose, delimiter=',', fmt='%.9f')
    np.savetxt('/home/aisl2/catkin_ws/src/data/model_crane_data/23_9_21_model_crane_data_to_compare_different_methods/random_1_imu_data.csv', imu_sensor_data, delimiter=',', fmt='%.9f')

    matplotlib.rcParams.update({'font.size': 18})
    fig = plt.figure(figsize=[14.4, 10.8])
    ax = fig.gca(projection='3d')
    ax.plot(imu_pose[:, 1], imu_pose[:, 2], imu_pose[:, 3])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory Plot')

    plt.show()
