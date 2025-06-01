from tf.transformations import euler_from_quaternion, quaternion_from_euler
from dataset import *
from util import *
from plot_fig import *


y_delta_p_array = [[ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00],
       [ 2.04473471e-05, -3.85477973e-01,  5.32226483e-04],
       [-3.29239813e-04, -4.29095088e-01,  1.16005813e-03],
       [-2.26521161e-04, -4.14533933e-01,  9.16486980e-04],
       [ 7.33329143e-05, -3.70888059e-01,  3.01846204e-04],
       [-2.53877620e-03, -6.10787239e-01,  5.04194259e-03]]



y_delta_q_array = [[1., 0., 0., 0.],
[ 9.99903867e-01, -1.20183764e-02, -4.67298834e-06, -6.91479335e-03],
[ 9.99880860e-01, -1.33799075e-02, -5.08176247e-06, -7.69705502e-03],
[ 9.99888789e-01, -1.29274931e-02, -4.79688804e-06, -7.43563575e-03],
[ 9.99910961e-01, -1.15677481e-02, -4.22667950e-06, -6.65260546e-03],
[ 9.99758425e-01, -1.90536943e-02, -7.02292062e-06, -1.09566719e-02]]

init_p = [0, 0, 0]
init_q = [1,0,0,0]

init_p_first = init_p
init_q_first = init_q
init_q_first_thread = quaternion.from_float_array(init_q)
init_p_first_thread = init_p
gt_cur_p = np.array(init_p)
gt_cur_q = quaternion.from_float_array(init_q)

for i in range(0, len(y_delta_p_array)):
    y_delta_p = y_delta_p_array[i,:]
    y_delta_q = y_delta_q_array[i,:]

    init_p_first_thread, init_q_first_thread = generate_trajectory_6d_quat_thread(init_p_first_thread, init_q_first_thread, y_delta_p, y_delta_q)
   
    # gt_cur_q = gt_cur_q * quaternion.from_float_array(y_delta_q[0,:]).normalized()
    # gt_cur_p = gt_cur_p + np.matmul(quaternion.as_rotation_matrix(gt_cur_q), y_delta_p[0,:].T).T
    # gt_trajectory1 = gt_cur_p




delta_gt_trajectory_end  = generate_trajectory_6d_quat(init_p_first, init_q_first, y_delta_p, y_delta_q)