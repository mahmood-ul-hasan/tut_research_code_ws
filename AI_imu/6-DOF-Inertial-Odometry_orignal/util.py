import numpy as np
import quaternion


def generate_trajectory_6d_quat(init_p, init_q, y_delta_p, y_delta_q):
    cur_p = np.array(init_p)
    cur_q = quaternion.from_float_array(init_q)
    pred_p = []
    pred_p.append(np.array(cur_p))

    for [delta_p, delta_q] in zip(y_delta_p, y_delta_q):
        cur_p = cur_p + np.matmul(quaternion.as_rotation_matrix(cur_q), delta_p.T).T
        cur_q = cur_q * quaternion.from_float_array(delta_q).normalized()
        pred_p.append(np.array(cur_p))

    return np.reshape(pred_p, (len(pred_p), 3))


def generate_trajectory_6d_quat_thread(cur_p, cur_q, y_delta_p, y_delta_q):
    cur_p = np.array(cur_p)
    # cur_q = quaternion.from_float_array(init_q)
    cur_p = cur_p + np.matmul(quaternion.as_rotation_matrix(cur_q), y_delta_p.T).T
    cur_q = cur_q * quaternion.from_float_array(y_delta_q).normalized()

    return cur_p, cur_q

def generate_trajectory_6d_quat_sep(x, init_p, init_q, y_delta_p, y_delta_q):

    if x == "gt":
        cur_p = np.array(init_p)
        cur_q = quaternion.from_float_array(init_q)
        pred_p = []
        pred_p.append(np.array(cur_p))

        for [delta_p, delta_q] in zip(y_delta_p, y_delta_q):
            cur_p = cur_p + np.matmul(quaternion.as_rotation_matrix(cur_q), delta_p.T).T
            cur_q = cur_q * quaternion.from_float_array(delta_q).normalized()
            pred_p.append(np.array(cur_p))

    
    if x == "pr":
        cur_p = np.array(init_p)
        cur_q = quaternion.from_float_array(init_q)
        pred_p = []
        pred_p.append(np.array(cur_p))

        for [delta_p, delta_q] in zip(y_delta_p, y_delta_q):
            delta_q2 = np.concatenate([delta_q[3:4], delta_q[0:3]])
            # delta_q1 = quaternion.as_float_array(delta_q)
            # delta_q2 = [delta_q1[1], delta_q1[2], delta_q1[3], delta_q1[0]]
            # delta_q3 = quaternion.from_float_array(delta_q2)
            cur_p = cur_p + np.matmul(quaternion.as_rotation_matrix(cur_q), delta_p.T).T
            cur_q = cur_q * quaternion.from_float_array(delta_q2).normalized()
            pred_p.append(np.array(cur_p))

        
    return np.reshape(pred_p, (len(pred_p), 3))






def generate_trajectory_3d(init_l, init_theta, init_psi, y_delta_l, y_delta_theta, y_delta_psi):
    cur_l = np.array(init_l)
    cur_theta = np.array(init_theta)
    cur_psi = np.array(init_psi)
    pred_l = []
    pred_l.append(np.array(cur_l))

    for [delta_l, delta_theta, delta_psi] in zip(y_delta_l, y_delta_theta, y_delta_psi):
        cur_theta = cur_theta + delta_theta
        cur_psi = cur_psi + delta_psi
        cur_l[0] = cur_l[0] + delta_l * np.sin(cur_theta) * np.cos(cur_psi)
        cur_l[1] = cur_l[1] + delta_l * np.sin(cur_theta) * np.sin(cur_psi)
        cur_l[2] = cur_l[2] + delta_l * np.cos(cur_theta)
        pred_l.append(np.array(cur_l))

    return np.reshape(pred_l, (len(pred_l), 3))