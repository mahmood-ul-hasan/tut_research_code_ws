#! /usr/bin/env python

import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
import math
import matplotlib.pyplot as plt
import time

# def my_publisher():
    # control part


if __name__ == '__main__':

    rospy.init_node('move_crane_node')
    control_publisher = rospy.Publisher('/k_crane/k_crane_joint_controller/command', JointTrajectory, queue_size=10)
    
    yaw_pos= 0
    yaw_increment = 2 # default = yaw_increment = 1.5

    pitch_pos= -30
    pitch_increment = 0.5
    yaw_cycle_start = True

    pitch_start_point_increment = 2
    yaw_start_point_increment = pitch_start_point_increment *4
    # pitch_cycle_complete = False
    # rate = rospy.Rate(10**10)
    rate = rospy.Rate(1)

    
    # fig = plt.figure()
    t = 0
    int_time = time.time()

    pitch_cycle = False
    yaw_cycle = True
    yaw_start_point = 0
    pitch_start_point = -30



    while not rospy.is_shutdown():
        
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = ['pitch_joint' ,'yaw_joint']

        point = JointTrajectoryPoint()
        # if (yaw_pos > 181):
        #     yaw_increment = 180
        # elif(yaw_pos < -181):
        #     yaw_increment = - 180
        # if (pitch_pos > 1):
        #     pitch_increment = 0
        # elif(pitch_pos < -91):
        #     pitch_increment = - 80
                

        point.positions = [pitch_pos*math.pi/180, yaw_pos*math.pi/180]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = rospy.Duration(1) 

        msg.points.append( point )
        control_publisher.publish( msg )
        # rospy.loginfo( msg ) 
     


        print("pitch = ", round(pitch_pos,2), "yaw = ", round(yaw_pos, 2), '  pitch_start_point = ', pitch_start_point, '  yaw_start_point = ', yaw_start_point, '  yaw_cycle_start =', yaw_cycle_start, "  time = ", round(t,2))
        
        # control the limits of yaw pose
        if (yaw_pos >= 170):
            rospy.sleep(20)
            yaw_increment = -yaw_increment
        elif(yaw_pos <= -170):
            rospy.sleep(20)
            yaw_increment = - yaw_increment
            # yaw_cycle_start = False
        
      
        # if (yaw_cycle_start == False):
        #     pitch_pos = pitch_pos - pitch_increment
        
        yaw_pos = yaw_pos + yaw_increment
  
        t = time.time() - int_time
       
        rate.sleep()
    


