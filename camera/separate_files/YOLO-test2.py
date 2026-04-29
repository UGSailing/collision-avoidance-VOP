import cv2
from ultralytics import YOLO

model = YOLO("yolo_models/duck.pt")
image = cv2.imread("images_to_test_models/eend_camera_aan_boord.png")

h, w = image.shape[:2]

# Middengebied van het beeld (pas ratios aan indien nodig)
y1, y2 = int(0.25 * h), int(0.75 * h)
x1, x2 = int(0.20 * w), int(0.80 * w)
roi = image[y1:y2, x1:x2]

# Verdeel middengebied in 3 tiles
tile_w = roi.shape[1] // 3
tiles = [
    roi[:, 0:tile_w],
    roi[:, tile_w : 2 * tile_w],
    roi[:, 2 * tile_w : roi.shape[1]],
]

# YOLO op elke tile
for i, tile in enumerate(tiles, start=1):
    results = model(tile)
    for result in results:
        result.show()
        result.save(filename=f"resultaat_tile_{i}.jpg")
