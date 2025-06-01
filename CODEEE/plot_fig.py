
import numpy as np
import matplotlib.pyplot as plt
import matplotlib

from matplotlib.animation import FuncAnimation
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math

def plot_fig(x, cur_gyro_data,cur_acc_data, cur_pos_data, cur_ori_data):

    euler_ori_data = np.zeros((len(cur_ori_data),3))
    euler_y_delta_q = np.zeros((len(cur_ori_data),3))

    cur_ori_data1 = np.concatenate([cur_ori_data[:,1:4], cur_ori_data[:, 0:1]], axis=1)
    for i in range(len(cur_ori_data)):
      euler_ori_data[i,:]= euler_from_quaternion(cur_ori_data1[i,:], axes='szyx')
     # euler_y_delta_q[i,:]= euler_from_quaternion(y_delta_q[i,:])

    fig = plt.figure(str(x)+ "  cur_gyro_data and cur_acc_data")
    plt.subplot(321); plt.plot(cur_gyro_data[:,0]*180/math.pi); plt.grid(True); plt.title("gyro x")
    plt.subplot(323); plt.plot(cur_gyro_data[:,1]*180/math.pi); plt.grid(True); plt.title("gyro y")
    plt.subplot(325); plt.plot(cur_gyro_data[:,2]*180/math.pi); plt.grid(True); plt.title("gyro z")

    plt.subplot(322); plt.plot(cur_acc_data[:,0]); plt.grid(True); plt.title("acc x")
    plt.subplot(324); plt.plot(cur_acc_data[:,1]); plt.grid(True); plt.title("acc y")
    plt.subplot(326); plt.plot(cur_acc_data[:,2]); plt.grid(True); plt.title("acc z")


    fig = plt.figure(str(x)+ "  pos and ori")
    plt.subplot(321); plt.plot(euler_ori_data[:,0]*180/math.pi); plt.grid(True); plt.title("ori x")
    plt.subplot(323); plt.plot(euler_ori_data[:,1]*180/math.pi); plt.grid(True); plt.title("ori y")
    plt.subplot(325); plt.plot(euler_ori_data[:,2]*180/math.pi); plt.grid(True); plt.title("ori z")

    plt.subplot(322); plt.plot(cur_pos_data[:,0]); plt.grid(True); plt.title("pos x")
    plt.subplot(324); plt.plot(cur_pos_data[:,1]); plt.grid(True); plt.title("pos y")
    plt.subplot(326); plt.plot(cur_pos_data[:,2]); plt.grid(True); plt.title("pos z")


    # fig = plt.figure("quat")
    # plt.subplot(411); plt.plot(cur_ori_data[:,0]); plt.grid(True); plt.title("ori x")
    # plt.subplot(412); plt.plot(cur_ori_data[:,1]); plt.grid(True); plt.title("ori x")
    # plt.subplot(413); plt.plot(cur_ori_data[:,2]); plt.grid(True); plt.title("ori x")
    # plt.subplot(414); plt.plot(cur_ori_data[:,3]); plt.grid(True); plt.title("ori x")

    fig = plt.figure(str(x)+ "  trj")
    ax = plt.axes(projection='3d')
    ax.plot3D(cur_pos_data[:,0]*180/math.pi, cur_pos_data[:,1]*180/math.pi, cur_pos_data[:,2])
    #plt.show()


def plot_fig_gazebo(x, cur_gyro_data,cur_acc_data, cur_pos_data, cur_ori_data, n1, n2):


    euler_ori_data = np.zeros((len(cur_ori_data),3))
    euler_y_delta_q = np.zeros((len(cur_ori_data),3))

    # fig = plt.figure(str(x)+ "  quat raw cur_gyro_data and cur_acc_data")
    # plt.subplot(411); plt.plot(cur_ori_data[:,0]); plt.grid(True); plt.title("gyro x")
    # plt.subplot(412); plt.plot(cur_ori_data[:,1]); plt.grid(True); plt.title("gyro y")
    # plt.subplot(413); plt.plot(cur_ori_data[:,2]); plt.grid(True); plt.title("gyro z")
    # plt.subplot(414); plt.plot(cur_ori_data[:,3]); plt.grid(True); plt.title("gyro z")

    cur_ori_data1 = np.concatenate([cur_ori_data[:,1:4], cur_ori_data[:, 0:1]], axis=1)

    fig = plt.figure(str(x)+ "  quat Modified qrientation")
    plt.subplot(411); plt.plot(cur_ori_data1[n1:n2,0]); plt.grid(True); plt.title("quat x")
    plt.subplot(412); plt.plot(cur_ori_data1[n1:n2,1]); plt.grid(True); plt.title("quat y")
    plt.subplot(413); plt.plot(cur_ori_data1[n1:n2,2]); plt.grid(True); plt.title("quat z")
    plt.subplot(414); plt.plot(cur_ori_data1[n1:n2,3]); plt.grid(True); plt.title("quat w")
    plt.ticklabel_format(style='plain')    # to prevent scientific notation.





    for i in range(len(cur_ori_data1)):
      euler_ori_data[i,:]= euler_from_quaternion(cur_ori_data1[i,:], axes='sxyz')
     # euler_y_delta_q[i,:]= euler_from_quaternion(y_delta_q[i,:])

    fig = plt.figure(str(x)+ "  cur_gyro_data and cur_acc_data")
    plt.subplot(321); plt.plot(cur_gyro_data[n1:n2,0]*180/math.pi); plt.grid(True); plt.title("gyro x")
    plt.subplot(323); plt.plot(cur_gyro_data[n1:n2,1]*180/math.pi); plt.grid(True); plt.title("gyro y")
    plt.subplot(325); plt.plot(cur_gyro_data[n1:n2,2]*180/math.pi); plt.grid(True); plt.title("gyro z")

    plt.subplot(322); plt.plot(cur_acc_data[n1:n2,0]); plt.grid(True); plt.title("acc x")
    plt.subplot(324); plt.plot(cur_acc_data[n1:n2,1]); plt.grid(True); plt.title("acc y")
    plt.subplot(326); plt.plot(cur_acc_data[n1:n2,2]); plt.grid(True); plt.title("acc z")


    fig = plt.figure(str(x)+ "  pos and ori")
    plt.subplot(321); plt.plot(euler_ori_data[n1:n2,0]*180/math.pi); plt.grid(True); plt.title("ori x")
    plt.subplot(323); plt.plot(euler_ori_data[n1:n2,1]*180/math.pi); plt.grid(True); plt.title("ori y")
    plt.subplot(325); plt.plot(euler_ori_data[n1:n2,2]*180/math.pi); plt.grid(True); plt.title("ori z")

    plt.subplot(322); plt.plot(cur_pos_data[n1:n2,0]); plt.grid(True); plt.title("pos x")
    plt.subplot(324); plt.plot(cur_pos_data[n1:n2,1]); plt.grid(True); plt.title("pos y")
    plt.subplot(326); plt.plot(cur_pos_data[n1:n2,2]); plt.grid(True); plt.title("pos z")
    plt.ticklabel_format(style='plain')    # to prevent scientific notation.



    # fig = plt.figure("quat")
    # plt.subplot(411); plt.plot(cur_ori_data[:,0]); plt.grid(True); plt.title("ori x")
    # plt.subplot(412); plt.plot(cur_ori_data[:,1]); plt.grid(True); plt.title("ori x")
    # plt.subplot(413); plt.plot(cur_ori_data[:,2]); plt.grid(True); plt.title("ori x")
    # plt.subplot(414); plt.plot(cur_ori_data[:,3]); plt.grid(True); plt.title("ori x")

    fig = plt.figure(str(x)+ "  trj")
    ax = plt.axes(projection='3d')
    ax.plot3D(cur_pos_data[:,0]*180/math.pi, cur_pos_data[:,1]*180/math.pi, cur_pos_data[:,2])
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    #plt.show()



def plot_compare_pos(x, y , predict_pos_data, predict_ori_data1, gt_pos_data, gt_ori_data1):

    print("pos_data", np.shape(gt_pos_data))
    print("y_delta_p", np.shape(predict_pos_data))
    print("ori_data", np.shape(gt_ori_data1))
    print("y_delta_q", np.shape(predict_ori_data1))

    gt_ori_data = np.concatenate([gt_ori_data1[:,1:4], gt_ori_data1[:, 0:1]], axis=1)
    predict_ori_data = np.concatenate([predict_ori_data1[:,1:4], predict_ori_data1[:, 0:1]], axis=1)


    euler_gt_ori_data = np.zeros((len(gt_ori_data),3))
    euler_predict_ori_data = np.zeros((len(predict_ori_data),3))
    
    for i in range(len(gt_ori_data)):
      euler_gt_ori_data[i,:]= euler_from_quaternion(gt_ori_data[i,:], axes='szyx')

    for i in range(len(predict_ori_data)):
      euler_predict_ori_data[i,:]= euler_from_quaternion(predict_ori_data[i,:], axes='szyx')


    fig = plt.figure(str(x) + " vs " + str(y) + " pos and ori")
    plt.subplot(622); plt.plot(euler_gt_ori_data[:,0]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(y) + " ori x")
    plt.subplot(626); plt.plot(euler_gt_ori_data[:,1]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(y) + " ori y")
    plt.subplot(6,2,10); plt.plot(euler_gt_ori_data[:,2]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(y) + " ori z")

    plt.subplot(624); plt.plot(euler_predict_ori_data[:,0]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(x) + " ori x")
    plt.subplot(628); plt.plot(euler_predict_ori_data[:,1]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(x) + " ori y")
    plt.subplot(6,2,12); plt.plot(euler_predict_ori_data[:,2]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(x) + " ori z")

    plt.subplot(621); plt.plot(gt_pos_data[:,0], label='gt'); plt.grid(True); plt.title(str(y) + " pos x")
    plt.subplot(625); plt.plot(gt_pos_data[:,1], label='gt'); plt.grid(True); plt.title(str(y) + " pos y")
    plt.subplot(6,2,9); plt.plot(gt_pos_data[:,2], label='gt'); plt.grid(True); plt.title(str(y) + " pos z")

    plt.subplot(623); plt.plot(predict_pos_data[:,0], label='predict'); plt.grid(True); plt.title(str(x) + " pos x")
    plt.subplot(627); plt.plot(predict_pos_data[:,1], label='predict'); plt.grid(True); plt.title(str(x) + " pos y")
    plt.subplot(6,2,11); plt.plot(predict_pos_data[:,2], label='predict'); plt.grid(True); plt.title(str(x) + " pos z")

    # fig = plt.figure(str(x)+ "  trj")
    # ax = plt.axes(projection='3d')
    # ax.plot3D(cur_pos_data[:,0]*180/math.pi, cur_pos_data[:,1]*180/math.pi, cur_pos_data[:,2])


def plot_compare_imu(x, y,  acc_data, gyro_data, x_acc,  x_gyro):



    print("gyro_data", np.shape(gyro_data))
    print("x_gyro", np.shape(x_gyro))
    print("acc_data", np.shape(acc_data))
    print("x_acc", np.shape(x_acc))

   
    fig = plt.figure(str(x) + " vs " + str(y) + " IMU")

    plt.subplot(621); plt.plot(acc_data[:,0], label='gt'); plt.grid(True); plt.title(str(x) + " acc x")
    plt.subplot(625); plt.plot(acc_data[:,1], label='gt'); plt.grid(True); plt.title(str(x) + " acc y")
    plt.subplot(6,2,9); plt.plot(acc_data[:,2], label='gt'); plt.grid(True); plt.title(str(x) + " acc z")

    plt.subplot(623); plt.plot(x_acc[:,0,0], label='predict'); plt.grid(True); plt.title(str(y) + " acc x")
    plt.subplot(627); plt.plot(x_acc[:,0,1], label='predict'); plt.grid(True); plt.title(str(y) + " acc y")
    plt.subplot(6,2,11); plt.plot(x_acc[:,0,2], label='predict'); plt.grid(True); plt.title(str(y) + " acc z")


    plt.subplot(622); plt.plot(gyro_data[:,0], label='gt'); plt.grid(True); plt.title(str(x) + " gyro x")
    plt.subplot(626); plt.plot(gyro_data[:,1], label='gt'); plt.grid(True); plt.title(str(x) + " gyro y")
    plt.subplot(6,2,10); plt.plot(gyro_data[:,2], label='gt'); plt.grid(True); plt.title(str(x) + " gyro z")

    plt.subplot(624); plt.plot(x_gyro[:,0,0], label='predict'); plt.grid(True); plt.title(str(y) + " gyro x")
    plt.subplot(628); plt.plot(x_gyro[:,0,1], label='predict'); plt.grid(True); plt.title(str(y) + " gyro y")
    plt.subplot(6,2,12); plt.plot(x_gyro[:,0,2], label='predict'); plt.grid(True); plt.title(str(y) + " gyro z")

    
    # fig = plt.figure(str(x)+ "  trj")
    # ax = plt.axes(projection='3d')
    # ax.plot3D(cur_pos_data[:,0]*180/math.pi, cur_pos_data[:,1]*180/math.pi, cur_pos_data[:,2])
    # plt.show()



def plot_delta_position_perdict_vs_gt(title, x , predict_pos_data, gt_pos_data):


  pos_error = gt_pos_data - predict_pos_data

  fig = plt.figure("delta position " + str(title))
  fig.suptitle("delta position " + str(title))
  plt.subplot(321); plt.plot(gt_pos_data[:,0], label='gt'); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta position x (m)");   plt.legend()
  plt.subplot(323); plt.plot(gt_pos_data[:,1], label= 'gt'); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta position y (m)"); plt.legend()
  plt.subplot(325); plt.plot(gt_pos_data[:,2], label='gt'); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta position z (m)");   plt.legend()

  plt.subplot(321); plt.plot(predict_pos_data[:,0], label= 'predict (' + str(x) + ")"); plt.grid(True);   plt.xlabel("Frame"); plt.ylabel("Delta position x (m)");   plt.legend()
  plt.subplot(323); plt.plot(predict_pos_data[:,1], label='predict (' + str(x) + ")");  plt.grid(True);   plt.xlabel("Frame"); plt.ylabel("Delta position y (m)");   plt.legend()
  plt.subplot(325); plt.plot(predict_pos_data[:,2], label='predict (' + str(x) + ")");  plt.grid(True);   plt.xlabel("Frame"); plt.ylabel("Delta position z (m)");   plt.legend()


  plt.subplot(322); plt.plot(pos_error[:,0], label=  str(x)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta position x error (m)");    plt.legend()
  plt.subplot(324); plt.plot(pos_error[:,1], label=  str(x)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta position y error (m)");   plt.legend()
  plt.subplot(326); plt.plot(pos_error[:,2], label=  str(x)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta position x error (m)");    plt.legend()


def plot_delta_rotation_angle_perdict_vs_gt(title, x , predict_ori_data , gt_ori_data):

  gt_ori_data = np.concatenate([gt_ori_data[:,1:4], gt_ori_data[:, 0:1]], axis=1)
  predict_ori_data = np.concatenate([predict_ori_data[:,1:4], predict_ori_data[:, 0:1]], axis=1)


  euler_gt_ori_data = np.zeros((len(gt_ori_data),3))
  euler_predict_ori_data = np.zeros((len(predict_ori_data),3))
  
  for i in range(len(gt_ori_data)):
    euler_gt_ori_data[i,:]= euler_from_quaternion(gt_ori_data[i,:], axes='szyx')

  for i in range(len(predict_ori_data)):
    euler_predict_ori_data[i,:]= euler_from_quaternion(predict_ori_data[i,:], axes='szyx')

    ori_error = euler_gt_ori_data - euler_predict_ori_data
  
  y = x
  fig = plt.figure("delta rotation angle " + str(title))
  fig.suptitle("delta position " + str(title))
  plt.subplot(321); plt.plot(euler_gt_ori_data[:,0]*180/math.pi, label='gt'); plt.grid(True);   plt.legend()
  plt.subplot(323); plt.plot(euler_gt_ori_data[:,1]*180/math.pi, label='gt'); plt.grid(True);   plt.legend()
  plt.subplot(325); plt.plot(euler_gt_ori_data[:,2]*180/math.pi, label='gt'); plt.grid(True);    plt.legend()

  plt.subplot(321); plt.plot(euler_predict_ori_data[:,0]*180/math.pi, label= 'predict (' + str(x) + ")"); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta rotation x error (m)");    plt.legend()
  plt.subplot(323); plt.plot(euler_predict_ori_data[:,1]*180/math.pi,label= 'predict (' + str(x) + ")"); plt.grid(True);  plt.xlabel("Frame"); plt.ylabel("Delta rotation y error (m)");   plt.legend()
  plt.subplot(325); plt.plot(euler_predict_ori_data[:,2]*180/math.pi, label= 'predict (' + str(x) + ")"); plt.grid(True);  plt.xlabel("Frame"); plt.ylabel("Delta rotation z error (m)");   plt.legend()


  plt.subplot(322); plt.plot(ori_error[:,0]*180/math.pi, label=  str(x)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta rotation x error (m)"); plt.legend()
  plt.subplot(324); plt.plot(ori_error[:,1]*180/math.pi, label=  str(x)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta rotation y error (m)"); plt.legend()
  plt.subplot(326); plt.plot(ori_error[:,2]*180/math.pi, label=  str(x)); plt.grid(True); plt.xlabel("Frame"); plt.ylabel("Delta rotation z error (m)"); plt.legend()



def plot_delta_gt(title, x , gt_pos_data , gt_ori_data):

  gt_ori_data = np.concatenate([gt_ori_data[:,1:4], gt_ori_data[:, 0:1]], axis=1)


  euler_gt_ori_data = np.zeros((len(gt_ori_data),3))
  
  for i in range(len(gt_ori_data)):
    euler_gt_ori_data[i,:]= euler_from_quaternion(gt_ori_data[i,:], axes='szyx')

  
  y = x
  fig = plt.figure("delta gt  " + str(title))
  fig.suptitle("delta gt  " + str(title))

  plt.subplot(321); plt.plot(gt_pos_data[:,0], label=  str(x)  ); plt.grid(True); plt.title("gt pos");   plt.legend()
  plt.subplot(323); plt.plot(gt_pos_data[:,1], label= str(x)); plt.grid(True);    plt.legend()
  plt.subplot(325); plt.plot(gt_pos_data[:,2], label= str(x)); plt.grid(True);   plt.legend()



  plt.subplot(322); plt.plot(euler_gt_ori_data[:,0]*180/math.pi, label= str(x)); plt.grid(True);   plt.title(str(x) + " gt ori");  plt.legend()
  plt.subplot(324); plt.plot(euler_gt_ori_data[:,1]*180/math.pi, label= str(x)); plt.grid(True);   plt.legend()
  plt.subplot(326); plt.plot(euler_gt_ori_data[:,2]*180/math.pi, label= str(x)); plt.grid(True);    plt.legend()




def plot_pos_error(x , predict_pos_data, predict_ori_data , gt_pos_data, gt_ori_data, n1, n2):


  gt_ori_data = np.concatenate([gt_ori_data[:,1:4], gt_ori_data[:, 0:1]], axis=1)
  predict_ori_data = np.concatenate([predict_ori_data[:,1:4], predict_ori_data[:, 0:1]], axis=1)


  euler_gt_ori_data = np.zeros((len(gt_ori_data),3))
  euler_predict_ori_data = np.zeros((len(predict_ori_data),3))
  
  for i in range(len(gt_ori_data)):
    euler_gt_ori_data[i,:]= euler_from_quaternion(gt_ori_data[i,:], axes='szyx')

  for i in range(len(predict_ori_data)):
    euler_predict_ori_data[i,:]= euler_from_quaternion(predict_ori_data[i,:], axes='szyx')

    ori_error = euler_gt_ori_data - euler_predict_ori_data
    pos_error = gt_pos_data - predict_pos_data


  
  y = x
  fig = plt.figure(str(x) + " pos and ori")

  plt.subplot(621); plt.plot(gt_pos_data[:,0], label='gt'); plt.grid(True); plt.title(str(x) + " pos x");   plt.legend()
  plt.subplot(623); plt.plot(gt_pos_data[:,1], label='gt'); plt.grid(True); plt.title(str(x) + " pos y");   plt.legend()
  plt.subplot(625); plt.plot(gt_pos_data[:,2], label='gt'); plt.grid(True); plt.title(str(x) + " pos z");   plt.legend()

  plt.subplot(621); plt.plot(predict_pos_data[:,0], label='predict'); plt.grid(True); plt.title(str(y) + " pos x");   plt.legend()
  plt.subplot(623); plt.plot(predict_pos_data[:,1], label='predict'); plt.grid(True); plt.title(str(y) + " pos y");   plt.legend()
  plt.subplot(625); plt.plot(predict_pos_data[:,2], label='predict'); plt.grid(True); plt.title(str(y) + " pos z");   plt.legend()


  plt.subplot(622); plt.plot(pos_error[:,0]); plt.grid(True); plt.title(str(x) + " pos x")
  plt.subplot(624); plt.plot(pos_error[:,1]); plt.grid(True); plt.title(str(x) + " pos y")
  plt.subplot(626); plt.plot(pos_error[:,2]); plt.grid(True); plt.title(str(x) + " pos z")



  plt.subplot(627); plt.plot(euler_gt_ori_data[:,0]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(x) + " ori x");   plt.legend()
  plt.subplot(629); plt.plot(euler_gt_ori_data[:,1]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(x) + " ori y");   plt.legend()
  plt.subplot(6,2,11); plt.plot(euler_gt_ori_data[:,2]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(x) + " ori z");   plt.legend()

  plt.subplot(627); plt.plot(euler_predict_ori_data[:,0]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(y) + " ori x");   plt.legend()
  plt.subplot(629); plt.plot(euler_predict_ori_data[:,1]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(y) + " ori y");   plt.legend()
  plt.subplot(6,2,11); plt.plot(euler_predict_ori_data[:,2]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(y) + " ori z");   plt.legend()
  plt.legend()

  plt.subplot(628); plt.plot(ori_error[:,0]*180/math.pi); plt.grid(True); plt.title(str(y) + " ori x")
  plt.subplot(6,2,10); plt.plot(ori_error[:,1]*180/math.pi); plt.grid(True); plt.title(str(y) + " ori y")
  plt.subplot(6,2,12); plt.plot(ori_error[:,2]*180/math.pi); plt.grid(True); plt.title(str(y) + " ori z")

# fig = plt.figure(str(x)+ "  trj")
# ax = plt.axes(projection='3d')
# ax.plot3D(cur_pos_data[:,0]*180/math.pi, cur_pos_data[:,1]*180/math.pi, cur_pos_data[:,2])



def plot_pos_error_gazebo(x , predict_pos_data, predict_ori_data , gt_pos_data, gt_ori_data, n1, n2):


  gt_ori_data = np.concatenate([gt_ori_data[:,1:4], gt_ori_data[:, 0:1]], axis=1)
  predict_ori_data = np.concatenate([predict_ori_data[:,1:4], predict_ori_data[:, 0:1]], axis=1)


  euler_gt_ori_data = np.zeros((len(gt_ori_data),3))
  euler_predict_ori_data = np.zeros((len(predict_ori_data),3))
  
  for i in range(len(gt_ori_data)):
    euler_gt_ori_data[i,:]= euler_from_quaternion(gt_ori_data[i,:],  axes='sxyz')

  for i in range(len(predict_ori_data)):
    euler_predict_ori_data[i,:]= euler_from_quaternion(predict_ori_data[i,:], axes='sxyz')

    ori_error = euler_gt_ori_data - euler_predict_ori_data
    pos_error = gt_pos_data - predict_pos_data


  fig = plt.figure(str(x) + " pos and ori")
  y = x

  plt.subplot(621); plt.plot(gt_pos_data[n1:n2,0], label='gt'); plt.grid(True); plt.title(str(x) + " pos x");   plt.legend()
  plt.subplot(623); plt.plot(gt_pos_data[n1:n2,1], label='gt'); plt.grid(True); plt.title(str(x) + " pos y");   plt.legend()
  plt.subplot(625); plt.plot(gt_pos_data[n1:n2,2], label='gt'); plt.grid(True); plt.title(str(x) + " pos z");   plt.legend()

  plt.subplot(621); plt.plot(predict_pos_data[n1:n2,0], label='predict'); plt.grid(True); plt.title(str(y) + " pos x");   plt.legend()
  plt.subplot(623); plt.plot(predict_pos_data[n1:n2,1], label='predict'); plt.grid(True); plt.title(str(y) + " pos y");   plt.legend()
  plt.subplot(625); plt.plot(predict_pos_data[n1:n2,2], label='predict'); plt.grid(True); plt.title(str(y) + " pos z");   plt.legend()


  plt.subplot(622); plt.plot(pos_error[n1:n2,0]); plt.grid(True); plt.title(str(x) + " pos x")
  plt.subplot(624); plt.plot(pos_error[n1:n2,1]); plt.grid(True); plt.title(str(x) + " pos y")
  plt.subplot(626); plt.plot(pos_error[n1:n2,2]); plt.grid(True); plt.title(str(x) + " pos z")



  plt.subplot(627); plt.plot(euler_gt_ori_data[n1:n2,0]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(x) + " ori x");   plt.legend()
  plt.subplot(629); plt.plot(euler_gt_ori_data[n1:n2,1]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(x) + " ori y");   plt.legend()
  plt.subplot(6,2,11); plt.plot(euler_gt_ori_data[n1:n2,2]*180/math.pi, label='gt'); plt.grid(True); plt.title(str(x) + " ori z");   plt.legend()

  plt.subplot(627); plt.plot(euler_predict_ori_data[n1:n2,0]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(y) + " ori x");   plt.legend()
  plt.subplot(629); plt.plot(euler_predict_ori_data[n1:n2,1]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(y) + " ori y");   plt.legend()
  plt.subplot(6,2,11); plt.plot(euler_predict_ori_data[n1:n2,2]*180/math.pi, label='predict'); plt.grid(True); plt.title(str(y) + " ori z");   plt.legend()
  plt.legend()

  plt.subplot(628); plt.plot(ori_error[n1:n2,0]*180/math.pi); plt.grid(True); plt.title(str(y) + " ori x")
  plt.subplot(6,2,10); plt.plot(ori_error[n1:n2,1]*180/math.pi); plt.grid(True); plt.title(str(y) + " ori y")
  plt.subplot(6,2,12); plt.plot(ori_error[n1:n2,2]*180/math.pi); plt.grid(True); plt.title(str(y) + " ori z")
  plt.ticklabel_format(style='plain')    # to prevent scientific notation.

# fig = plt.figure(str(x)+ "  trj")
# ax = plt.axes(projection='3d')
# ax.plot3D(cur_pos_data[:,0]*180/math.pi, cur_pos_data[:,1]*180/math.pi, cur_pos_data[:,2])







    
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