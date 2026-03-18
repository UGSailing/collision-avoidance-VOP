import math

def distance_and_angle_from_bbox(
    bbox,               # (x1, y1, x2, y2) in pixels
    object_height_m,    # echte hoogte van object in meter
    fx, fy,             # focal lengths in pixels
    cx, cy              # principal point in pixels
):
    x1, y1, x2, y2 = bbox

    h_px = y2 - y1
    if h_px <= 0:
        raise ValueError("Bounding box height must be > 0")

    x_center = (x1 + x2) / 2

    # afstand recht vooruit (depth)
    distance_m = fy * object_height_m / h_px

    # horizontale hoek t.o.v. het midden van de camera
    angle_rad = math.atan((x_center - cx) / fx)
    angle_deg = math.degrees(angle_rad)

    return distance_m, angle_deg