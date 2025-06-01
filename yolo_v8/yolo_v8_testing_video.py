# import supervision as sv
# import numpy as np
# from ultralytics import YOLO

# VIDEO_PATH = "/media/aisl2/aisl_data/backup/catkin_ws/src/data/kobelco_exp_2023/4_varying_boom_Angle_during_cycle_(copy)/output.mp4"
# OUTPUT_VIDEO_PATH = "/media/aisl2/aisl_data/backup/catkin_ws/src/data/kobelco_exp_2023/4_varying_boom_Angle_during_cycle_(copy)/output_detected.mp4"
# model = YOLO("/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/weights/best.pt")

# results = model(VIDEO_PATH, task="detect")  # generator of Results objects



import subprocess

# Replace these paths with your actual paths
# model_path = "/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/weights/best.pt"
# detect_model_path = "/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/weights/best.pt"
# seg_model_path = "/media/aisl2/aisl_data/code_ws/yolo_v8/yolov8n-seg.pt"
detect_model_path = "/media/aisl2/aisl_data/code_ws/yolo_v8/yolov8n.pt"
# seg_model_path =  "/media/aisl2/aisl_data/code_ws/yolo_v8/rope_sky_detection/results/600_epochs_2024_2_27/weights/best.pt"
# detect_model_path = "/media/aisl2/aisl_data/code_ws/yolo_v8/data_set_more_yolov8/results/600_epochs_model_obj_detect/weights/best.pt"
# 4th
# source_path = "/media/aisl2/aisl_data/backup/catkin_ws/src/data/kobelco_exp_2023/4_varying_boom_Angle_during_cycle/4_varying_boom_Angle_during_cycle.mp4"

# 3rd
# source_path = "/media/aisl2/aisl_data/catkin_ws/src/data/kobelco_exp_2023/data_set_for_deep_learning/3_varying_speed_and_boom_Angle_at_the_start_of_cycle/3_varying_speed_and_boom_Angle_at_the_start_of_cycle.mp4"
# source_path = "/media/aisl2/aisl_data/catkin_ws/src/data/kobelco_exp_2023/data_set_for_deep_learning/6_vertical_5_fps.mp4"

# source_path = "/media/aisl2/aisl_data/code_ws/yolo_v8/video/Untitled design (1).mp4"

# # Construct the command using variables
# command = f'yolo task=detect mode=predict source="{source_path}" model="{detect_model_path}" show=True'
# # command = f'yolo task=segment mode=predict source="{source_path}" model="{seg_model_path}" show=True'

# # Run the command in the terminal
# try:
#     subprocess.run(command, shell=True, check=True)
# except subprocess.CalledProcessError as e:
#     print(f"Error: Command '{command}' failed with exit code {e.returncode}")


for i in range(1, 11):
    # Update the source_path for each iteration
    source_path = f"/media/aisl2/aisl_data/code_ws/yolo_v8/video/Untitled design ({i}).mp4"

    # Construct the command using variables
    command = f'yolo task=detect mode=predict source="{source_path}" model="{detect_model_path}" show=True'
    # command = f'yolo task=segment mode=predict source="{source_path}" model="{seg_model_path}" show=True'

    # Run the command in the terminal
    try:
        subprocess.run(command, shell=True, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: Command '{command}' failed with exit code {e.returncode}")