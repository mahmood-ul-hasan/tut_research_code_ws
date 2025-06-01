from ultralytics import YOLO
from matplotlib import pyplot as plt
from PIL import Image


from IPython.display import Image
Image("/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/results.png")
Image(filename='/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/train_batch0.jpg', width=900)


my_new_det_model = YOLO('/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/weights/best.pt')
new_image = '/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/train/images/frame0000_jpg.rf.a7ae120001e29a32c98159bf1e203c75.jpg'
detection_result = my_new_det_model.predict(new_image)  #Adjust conf threshold
detection_result_array = detection_result[0].plot()
plt.figure(figsize=(12, 12))
plt.imshow(detection_result_array)




from IPython.display import Image
Image("/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_seg/results.png")
Image(filename='/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_seg/train_batch0.jpg', width=900)


instance_model = YOLO('/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_seg/weights/best.pt')
new_image = '/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/train/images/frame0000_jpg.rf.a7ae120001e29a32c98159bf1e203c75.jpg'
instance_result = instance_model.predict(new_image)  #Adjust conf threshold
instance_result_array = instance_result[0].plot()
plt.figure(figsize=(12, 12))
plt.imshow(instance_result_array)





detection_result_array = detection_result[0].plot()
instance_result_array = instance_result[0].plot()
fig = plt.figure(figsize=(18, 9))
ax1 = fig.add_subplot(1,2,1)
ax1.set_title("Object Detection")
ax1.imshow(detection_result_array)
ax2 = fig.add_subplot(1,2,2)
ax2.set_title("Instance Segmentation")
ax2.imshow(instance_result_array)


#Number of bounding boxes (objects) detected in the image
result = instance_result[0]
len(result.boxes)


#BBOX coordinates, class ID, and probability for a specific box.
box = result.boxes[0]
cords = box.xyxy[0].tolist()
class_id = box.cls[0].item()
conf = box.conf[0].item()
print("Object type:", class_id)
print("Coordinates:", cords)
print("Probability:", conf)
print(result.names)






#Detection model
det_model = YOLO('yolov8n.pt')

#Instance model
inst_model = YOLO('yolov8n-seg.pt')



# Perform object detection on an image using the model
img = '/media/aisl2/aisl_data/backup/code_ws/yolo_v8/img.jpeg'
detection_results = det_model.predict(img)
instance_results = inst_model.predict(img)