from ultralytics import YOLO
from matplotlib import pyplot as plt
from PIL import Image


#Detection model
det_model = YOLO('yolov8n.pt')

#Instance model
inst_model = YOLO('yolov8n-seg.pt')
     
results = det_model.train(data='/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/data.yaml',
                      epochs=1)



# Perform object detection on an image using the model
img = '/media/aisl2/aisl_data/backup/code_ws/yolo_v8/img.jpeg'
detection_results = det_model.predict(img)
instance_results = inst_model.predict(img)


detection_result_array = detection_results[0].plot()
instance_result_array = instance_results[0].plot()
fig = plt.figure(figsize=(18, 9))
ax1 = fig.add_subplot(1,2,1)
ax1.set_title("Object Detection")
ax1.imshow(detection_result_array)
ax2 = fig.add_subplot(1,2,2)
ax2.set_title("Instance Segmentation")
ax2.imshow(instance_result_array)


#Number of bounding boxes (objects) detected in the image
result = instance_results[0]
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