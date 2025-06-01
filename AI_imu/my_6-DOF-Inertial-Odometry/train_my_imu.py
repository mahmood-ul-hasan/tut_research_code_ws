import numpy as np
import matplotlib.pyplot as plt
import argparse

#from tensorflow.keras.
from tensorflow.keras.callbacks import ModelCheckpoint, TensorBoard
from tensorflow.keras.models import load_model
from tensorflow.keras.optimizers import Adam

from sklearn.utils import shuffle

from time import time

from dataset import *
from model import *
from util import *

from plot_fig import *

def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument('dataset', choices=['oxiod', 'euroc', 'my_imu'], help='Training dataset name (\'oxiod\' or \'euroc\' or \'my_imu\')')
    # parser.add_argument('output', help='Model output name')
    # args = parser.parse_args()

    class argument:
        # dataset = "euroc"
        # model = "6dofio_euroc.hdf5"
        # input = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/imu0/data.csv"
        # gt = "/mah/AI/6-DOF-Inertial-Odometry-master/MH_02_easy/mav0/state_groundtruth_estimate0/data.csv"

        dataset = "my_imu"
        # model = "/mah/AI/6-DOF-Inertial-Odometry/my_imu_accel_rotate.hdf5"
        output = "/mah/AI/my_6-DOF-Inertial-Odometry/my_imu_accel_rotate"
        # gt = "/mah/AI/6-DOF-Inertial-Odometry/data_gt_accel_rotate.csv"
    
    args = argument()

    np.random.seed(0)

    window_size = 200
    stride = 10

    x_gyro = []
    x_acc = []

    y_delta_p = []
    y_delta_q = []

    imu_data_filenames = []
    gt_data_filenames = []

    gyro_data = []
    acc_data = []
    pos_data = []
    ori_data = []




    if args.dataset == 'my_imu':
        # imu_data_filenames.append('data_imu_rotate.csv')
        # gt_data_filenames.append('data_gt_rotate.csv')

        # imu_data_filenames.append('data_imu_accel.csv')
        # gt_data_filenames.append('data_gt_accel.csv')

        imu_data_filenames.append('data_imu_accel_rotate.csv')
        gt_data_filenames.append('data_gt_accel_rotate.csv')


    if args.dataset == 'oxiod':
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data5/syn/imu3.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data2/syn/imu1.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data2/syn/imu2.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data5/syn/imu2.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/imu4.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data4/syn/imu4.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data4/syn/imu2.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/imu7.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data5/syn/imu4.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data4/syn/imu5.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/imu3.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/imu2.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data2/syn/imu3.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/imu1.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/imu3.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/imu5.csv')
        imu_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/imu4.csv')

        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data5/syn/vi3.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data2/syn/vi1.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data2/syn/vi2.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data5/syn/vi2.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/vi4.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data4/syn/vi4.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data4/syn/vi2.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/vi7.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data5/syn/vi4.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data4/syn/vi5.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/vi3.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/vi2.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data2/syn/vi3.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/vi1.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/vi3.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data3/syn/vi5.csv')
        gt_data_filenames.append('Oxford Inertial Odometry Dataset/handheld/data1/syn/vi4.csv')
    
    elif args.dataset == 'euroc':
        imu_data_filenames.append('MH_01_easy/mav0/imu0/data.csv')
        imu_data_filenames.append('MH_03_medium/mav0/imu0/data.csv')
        imu_data_filenames.append('MH_05_difficult/mav0/imu0/data.csv')
        imu_data_filenames.append('V1_02_medium/mav0/imu0/data.csv')
        imu_data_filenames.append('V2_01_easy/mav0/imu0/data.csv')
        imu_data_filenames.append('V2_03_difficult/mav0/imu0/data.csv')

        gt_data_filenames.append('MH_01_easy/mav0/state_groundtruth_estimate0/data.csv')
        gt_data_filenames.append('MH_03_medium/mav0/state_groundtruth_estimate0/data.csv')
        gt_data_filenames.append('MH_05_difficult/mav0/state_groundtruth_estimate0/data.csv')
        gt_data_filenames.append('V1_02_medium/mav0/state_groundtruth_estimate0/data.csv')
        gt_data_filenames.append('V2_01_easy/mav0/state_groundtruth_estimate0/data.csv')
        gt_data_filenames.append('V2_03_difficult/mav0/state_groundtruth_estimate0/data.csv')

    for i, (cur_imu_data_filename, cur_gt_data_filename) in enumerate(zip(imu_data_filenames, gt_data_filenames)):
        if args.dataset == 'oxiod':
            cur_gyro_data, cur_acc_data, cur_pos_data, cur_ori_data = load_oxiod_dataset(cur_imu_data_filename, cur_gt_data_filename)
        elif args.dataset == 'euroc' or args.dataset == 'my_imu':
            cur_gyro_data, cur_acc_data, cur_pos_data, cur_ori_data = load_euroc_mav_dataset(cur_imu_data_filename, cur_gt_data_filename)


        # x_gyro_mean = np.mean(cur_gyro_data, axis=0)
        # x_gyro_std = np.std(cur_gyro_data, axis=0)
        # cur_gyro_data = (cur_gyro_data- x_gyro_mean)/x_gyro_std
  
        # x_acc_mean = np.mean(cur_acc_data, axis=0)
        # x_acc_std = np.std(cur_acc_data, axis=0)
        # cur_acc_data = (cur_acc_data- x_acc_mean)/x_acc_std

        x_gyro_min = np.amin(cur_gyro_data, axis=0)
        x_gyro_max = np.amax(cur_gyro_data, axis=0)
        cur_gyro_data = (cur_gyro_data- x_gyro_min)/(x_gyro_max - x_gyro_min)
  
        x_acc_min = np.amin(cur_acc_data, axis=0)
        x_acc_max = np.amax(cur_acc_data, axis=0)
        cur_acc_data = (cur_acc_data- x_acc_min)/(x_acc_max - x_acc_min)

        cur_pos_data_min = np.amin(cur_pos_data, axis=0)
        cur_pos_data_max = np.amax(cur_pos_data, axis=0)
        cur_pos_data1 = (cur_pos_data- cur_pos_data_min)/(cur_pos_data_max - cur_pos_data_min)


        # plt.figure("fff")
        # x = "normal"
        # y = "old"

        # plt.subplot(621); plt.plot(cur_pos_data1[:,0], label='gt'); plt.grid(True); plt.title(str(x) + " acc x")
        # plt.subplot(625); plt.plot(cur_pos_data1[:,1], label='gt'); plt.grid(True); plt.title(str(x) + " acc y")
        # plt.subplot(6,2,9); plt.plot(cur_pos_data1[:,2], label='gt'); plt.grid(True); plt.title(str(x) + " acc z")

        # plt.subplot(623); plt.plot(cur_pos_data[:,0], label='predict'); plt.grid(True); plt.title(str(y) + " acc x")
        # plt.subplot(627); plt.plot(cur_pos_data[:,1], label='predict'); plt.grid(True); plt.title(str(y) + " acc y")
        # plt.subplot(6,2,11); plt.plot(cur_pos_data[:,2], label='predict'); plt.grid(True); plt.title(str(y) + " acc z")

        # plt.subplot(622); plt.plot(cur_acc_data1[:,0], label='gt'); plt.grid(True); plt.title(str(x) + " gyro x")
        # plt.subplot(626); plt.plot(cur_acc_data1[:,1], label='gt'); plt.grid(True); plt.title(str(x) + " gyro y")
        # plt.subplot(6,2,10); plt.plot(cur_acc_data1[:,2], label='gt'); plt.grid(True); plt.title(str(x) + " gyro z")

        # plt.subplot(624); plt.plot(cur_acc_data[:,0], label='predict'); plt.grid(True); plt.title(str(y) + " gyro x")
        # plt.subplot(628); plt.plot(cur_acc_data[:,1], label='predict'); plt.grid(True); plt.title(str(y) + " gyro y")
        # plt.subplot(6,2,12); plt.plot(cur_acc_data[:,2], label='predict'); plt.grid(True); plt.title(str(y) + " gyro z")

        # plt.sh4ow


        # [cur_x_gyro, cur_x_acc], [cur_y_delta_p, cur_y_delta_q], init_p, init_q = modified_load_dataset_6d_quat(cur_gyro_data, cur_acc_data, cur_pos_data, cur_ori_data, window_size, stride)
        [cur_x_gyro, cur_x_acc], [cur_y_delta_p, cur_y_delta_q], init_p, init_q = load_dataset_6d_quat_absolute(cur_gyro_data, cur_acc_data, cur_pos_data, cur_ori_data, window_size, stride)

        x_gyro.append(cur_x_gyro)
        x_acc.append(cur_x_acc)
        y_delta_p.append(cur_y_delta_p)
        y_delta_q.append(cur_y_delta_q)

        # gyro_data.append(cur_gyro_data)
        # acc_data.append(cur_acc_data)
        # pos_data.append(cur_pos_data)
        # ori_data.append(cur_pos_data)




    # plot_compare_imu("given data normal", "change in data2",  cur_x_acc, cur_x_gyro, cur_gyro_data1, cur_gyro_data1)
    # plot_compare_imu("given data22", "change in data2",  cur_x_acc, cur_x_gyro, cur_gyro_data, cur_gyro_data)
    # plot_compare_pos("given data", "change in data",  cur_pos_data, cur_ori_data, cur_y_delta_p, cur_y_delta_q)
    # plot_compare_imu("given data", "change in data",  cur_acc_data, cur_gyro_data, cur_x_acc, cur_x_gyro)


    x_gyro = np.vstack(x_gyro)
    x_acc = np.vstack(x_acc)

    y_delta_p = np.vstack(y_delta_p)
    y_delta_q = np.vstack(y_delta_q)

    print("pos_data", np.shape(pos_data))
    print("y_delta_p", np.shape(y_delta_p))
    print("ori_data", np.shape(ori_data))
    print("y_delta_q", np.shape(y_delta_q))
    print("gyro_data", np.shape(gyro_data))
    print("x_gyro", np.shape(x_gyro))
    print("acc_data", np.shape(acc_data))
    print("x_acc", np.shape(x_acc))

    x_gyro, x_acc, y_delta_p, y_delta_q = shuffle(x_gyro, x_acc, y_delta_p, y_delta_q)


    pred_model = create_pred_model_6d_quat(window_size)
    train_model = create_train_model_6d_quat(pred_model, window_size)
    train_model.compile(optimizer=Adam(0.0001), loss=None)

    model_checkpoint = ModelCheckpoint('model_checkpoint.hdf5', monitor='val_loss', save_best_only=True, verbose=1)
    tensorboard = TensorBoard(log_dir="logs/{}".format(time()))

    history = train_model.fit([x_gyro, x_acc, y_delta_p, y_delta_q], epochs=950, batch_size=32, verbose=1, callbacks=[model_checkpoint, tensorboard], validation_split=0.1)

    train_model = load_model('model_checkpoint.hdf5', custom_objects={'CustomMultiLossLayer':CustomMultiLossLayer}, compile=False)

    pred_model = create_pred_model_6d_quat(window_size)
    pred_model.set_weights(train_model.get_weights()[:-2])
    pred_model.save('%s.hdf5' % args.output)

    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Validation'], loc='upper left')
    plt.show()

if __name__ == '__main__':
    main()