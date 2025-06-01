from ultralytics import YOLO

# # For segmentation
# model = YOLO('path/to/your/segmentation_model.pt')
# results = model.val(data='your_data.yaml')
# 
model = YOLO('/media/aisl2/aisl_data/code_ws/yolo_v8/rope_sky_detection/results/600_epochs_2024_2_27/weights/best.pt')
results = model.val(data='/media/aisl2/aisl_data/code_ws/yolo_v8/rope_sky_detection/data.yaml')

import numpy as np

# Calculate the mean of the mAP scores from IoU=0.5 to 0.95
mean_ap = np.mean(results.maps)
print(f"Mean AP across IoU thresholds: {mean_ap}")

# iou_score = results.mask.iou  # Correct way to access IoU if available
# print(f"Average IoU: {iou_score}")

# Accessing IoU (assuming results contain IoU)
# iou_score = results.metrics['mask'].iou  # Replace 'box' with 'mask' for segmentation
# print(f"IoU score: {iou_score}")

# # Get overall mAP (mean over IoU thresholds 0.5:0.95)
# mean_ap = results.map()  # if this method is defined
# print(f"Mean AP across IoU thresholds: {mean_ap}")

# # Get mAP at IoU threshold of 0.5
# map50 = results.map50()
# print(f"mAP at IoU=0.5: {map50}")



# # For overall mAP across multiple IoU thresholds
# mean_ap = results.map()
# print(f"Mean AP across IoU thresholds from 0.5 to 0.95: {mean_ap}")