from ultralytics import YOLO

# Load the YOLO model
model = YOLO('./yolov8n-seg.pt')

for i in range(1, 5):
    # Update the source_path for each iteration
    source_path = f"./video/Untitled design ({i}).mp4"

    # Use the model for prediction directly
    results = model.predict(source=source_path, show=True)