from ultralytics import YOLO
from matplotlib import pyplot as plt
from PIL import Image

import torch

print(torch.cuda.is_available()) # If this returns True, it means PyTorch can use your GPU.


# Check PyTorch CUDA Version: You can also check the version of CUDA that PyTorch is using by running:

print(torch.version.cuda)

torch.cuda.set_device(0) # Set to your desired GPU number


#Detection model
# det_model = YOLO('yolov8n.pt')

#Instance model
inst_model = YOLO('yolov8n-seg.pt')

# define number of classes based on YAML
import yaml
with open("/media/aisl2/aisl_data/code_ws/yolo_v8/data_set_yolov8_version2/data.yaml", 'r') as stream:
    num_classes = str(yaml.safe_load(stream)['nc'])

#Define a project --> Destination directory for all results
project = "/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8_version2/results_gpu"
#Define subdirectory for this specific training
name = "600_epochs-feb_3_gpu" #note that if you run the training again, it creates a directory: 200_epochs-2


results = inst_model.train(data='/media/aisl2/aisl_data/code_ws/yolo_v8/data_set_yolov8_version2/data.yaml',
                      project=project,
                      name=name,
                      epochs=600,
                      device='cuda:0',
                      patience=0, #I am setting patience=0 to disable early stopping.
                      batch=4,
                      imgsz=800)


# Train the model
# results = det_model.train(data='/media/aisl2/aisl_data/backup/code_ws/yolo_v8/data_set_yolov8/data.yaml',
#                       project=project,
#                       name=name,
#                       epochs=600,
#                       patience=0, #I am setting patience=0 to disable early stopping.
#                       batch=4,
#                       imgsz=800)




