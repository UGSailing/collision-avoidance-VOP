from ultralytics import YOLO

# Laad een voorgetraind YOLOv8 nano model
# model = YOLO("yolov8n.pt")
model = YOLO("yolo_models/duck.pt")

# Voer detectie uit op een afbeelding
results = model("images/img_006.jpg")

# Bekijk de resultaten
for result in results:
    result.show()  # Toon afbeelding
    result.save(filename="resultaat.jpg")  # Sla resultaat op

