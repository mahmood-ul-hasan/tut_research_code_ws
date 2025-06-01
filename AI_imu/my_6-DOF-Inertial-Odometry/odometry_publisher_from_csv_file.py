#!/usr/bin/env python

import time
import rospy
import csv
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Header
pervious_time = 0.0


if __name__ == '__main__':
    rospy.init_node('csv_odometry_publisher')
    odom_pub = rospy.Publisher('/odom_ai', Odometry, queue_size=10)

    # Define the CSV file path
    csv_file_path = "/media/aisl2/aisl_data/AI_imu/my_6-DOF-Inertial-Odometry/odometry_data.csv"
    publishing_rate = rospy.Rate(10000)  # Publish at 10 Hz

    # Open the CSV file and read the data
    with open(csv_file_path, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)  # Skip the header row if present

        while not rospy.is_shutdown():

            now_time_stamp = time.time()

            if now_time_stamp >= pervious_time + (0.1):

                row = next(csvreader)  # Read the next row

                # Parse the CSV data
                pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z = map(float, row)

                # Create an Odometry message
                odom_msg = Odometry()
                odom_msg.header = Header()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = "world"
                # odom_msg.child_frame_id = "base_link"

                # Fill in the Pose information (excluding velocity fields)
                odom_msg.pose.pose.position.x = pos_x
                odom_msg.pose.pose.position.y = pos_y
                odom_msg.pose.pose.position.z = pos_z
                odom_msg.pose.pose.orientation = Quaternion(w=quat_w, x=quat_x, y=quat_y, z=quat_z)

                # Publish the Odometry message
                odom_pub.publish(odom_msg)
                print(odom_msg)
                pervious_time = now_time_stamp

            # publishing_rate.sleep()
    # End of file, exit the loop
