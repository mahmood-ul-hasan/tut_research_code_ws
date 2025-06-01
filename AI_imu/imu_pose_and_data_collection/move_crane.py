#! /usr/bin/env python

import rospy 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import random
import math
import matplotlib.pyplot as plt
import time

def my_publisher():
    # control part

    rospy.init_node('rrr_control_python_node')
    control_publisher = rospy.Publisher('/k_crane/k_crane_joint_controller/command', JointTrajectory, queue_size=10)
    
    yaw_pos= 0
    yaw_increment = 0.01

    pitch_pos= 0
    pitch_increment = 0.25
    yaw_cycle_complete = False
    pitch_cycle_complete = False
    rate = rospy.Rate(10**10)

    
    # fig = plt.figure()
    t = 0
    int_time = time.time()

    while not rospy.is_shutdown():
        
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.joint_names = ['pitch_joint' ,'yaw_joint']

        point = JointTrajectoryPoint()
        point.positions = [pitch_pos*math.pi/180, yaw_pos*math.pi/180]
        point.velocities = []
        point.accelerations = []
        point.effort = []
        point.time_from_start = rospy.Duration(1) 

        msg.points.append( point )
        control_publisher.publish( msg )
        # rospy.loginfo( msg ) 
        print("pitch = ", round(pitch_pos,2), "yaw = ", round(yaw_pos, 2), '  pitch_cycle_complete = ', pitch_cycle_complete, '  yaw_cycle_complete =', pitch_cycle_complete, "  time = ", round(t,2))



        yaw_pos = yaw_pos + yaw_increment

        if (yaw_pos >= 179):
            yaw_increment = -yaw_increment
        elif(yaw_pos <= -179):
            yaw_increment = - yaw_increment
            yaw_cycle_complete = True
        
        if (yaw_cycle_complete == True):
            yaw_cycle_complete = False
            pitch_pos = pitch_pos - pitch_increment
        
        
        # pitch_pos = pitch_pos - pitch_increment

        if (pitch_pos > 0):
            pitch_increment = -pitch_increment
            pitch_cycle_complete = True
        elif(pitch_pos <= -90):
            pitch_increment = - pitch_increment
            


        t = time.time() - int_time

        if(pitch_cycle_complete == True):
            print("pitch_pos = ", pitch_pos, "time = ", t)
            break

        # if(pitch_pos <= -1):
        #     print("pitch_pos = ", pitch_pos, "time = ", t)
        #     break
       
    
       

       


        # j1 = 2 * (random.random() - 0.5)  # 0 - 1 -> -0.5 - 0.5
        # j1 = 0*math.pi/180  # 0 - 1 -> -0.5 - 0.5

       



        # print(point.positions)
        # plt.plot(num, i)
        # num = num +1


        rate.sleep()
    
    # plt.show()


if __name__ == '__main__':

    my_publisher()

