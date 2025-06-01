from ultralytics import YOLO
from matplotlib import pyplot as plt
from PIL import Image


#Detection model
# det_model = YOLO('yolov8n.pt')
det_model =  YOLO('/media/aisl2/aisl_data/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_obj_detect/weights/last.pt')
#Instance model
# inst_model = YOLO('/media/aisl2/aisl_data/code_ws/yolo_v8/data_set_yolov8/results/600_epochs_model_seg/weights/last.pt')

# define number of classes based on YAML
import yaml
with open("/media/aisl2/aisl_data/code_ws/yolo_v8/detect_more_yolov8/data.yaml", 'r') as stream:
    num_classes = str(yaml.safe_load(stream)['nc'])

#Define a project --> Destination directory for all results
project = "/media/aisl2/aisl_data/code_ws/yolo_v8/detect_more_yolov8/results"
#Define subdirectory for this specific training
name = "600_epochs-" #note that if you run the training again, it creates a directory: 200_epochs-2


# Train the model
# results = inst_model.train(data='/media/aisl2/aisl_data/code_ws/yolo_v8/detect_more_yolov8/data.yaml',
results = det_model.train(data='/media/aisl2/aisl_data/code_ws/yolo_v8/detect_more_yolov8/data.yaml',
                      project=project,
                      name=name,
                      epochs=600,
                      patience=0, #I am setting patience=0 to disable early stopping.
                      batch=4,
                      imgsz=800)




