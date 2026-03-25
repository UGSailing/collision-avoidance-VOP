"""
CAN communication module for sending objects data from the camera to the control system.
"""

from __future__ import annotations

from typing import Any

import can

try:
    from . import config
except ImportError:
    import config  # type: ignore

class CANComms:
    def __init__(self):
        self.bus = can.interface.Bus(
            channel=config.CAN_CHANNEL,
            bustype=config.CAN_BUSTYPE,
            bitrate=config.CAN_BITRATE,
        )

    def close(self) -> None:
        try:
            self.bus.shutdown()
        except Exception:
            pass

    def _encode_object(self, angle_deg: float, distance_m: float) -> bytes | None:
        if distance_m < 0:
            return None

        angle_scale = float(config.CAN_OBSTACLE_ANGLE_SCALE_DEG_PER_LSB)
        distance_scale = float(config.CAN_OBSTACLE_DISTANCE_SCALE_M_PER_LSB)
        if angle_scale == 0 or distance_scale == 0:
            return None

        angle_raw = int(round(angle_deg / angle_scale))
        distance_raw = int(round(distance_m / distance_scale))

        signed_angle = bool(config.CAN_OBSTACLE_ANGLE_SIGNED)
        signed_distance = bool(config.CAN_OBSTACLE_DISTANCE_SIGNED)

        angle_min, angle_max = (-32768, 32767) if signed_angle else (0, 65535)
        dist_min, dist_max = (-32768, 32767) if signed_distance else (0, 65535)
        if not (angle_min <= angle_raw <= angle_max):
            return None
        if not (dist_min <= distance_raw <= dist_max):
            return None

        byteorder = str(config.CAN_OBSTACLE_BYTEORDER).lower()
        if byteorder not in ("big", "little"):
            return None

        return (
            angle_raw.to_bytes(2, byteorder=byteorder, signed=signed_angle)
            + distance_raw.to_bytes(2, byteorder=byteorder, signed=signed_distance)
        )

    def _send_frame(self, data: bytes) -> None:
        msg = can.Message(
            arbitration_id=int(config.CAN_OBSTACLE_ID),
            is_extended_id=bool(config.CAN_OBSTACLE_IS_EXTENDED_ID),
            data=data,
        )
        try:
            self.bus.send(msg)
        except can.CanError:
            # Best-effort transmission; drop frame on bus errors.
            return

    def send_objects(self, payload: dict[str, Any]) -> None:
        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            return

        encoded_objects: list[bytes] = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            angle_deg = detection.get("angle_deg")
            distance_m = detection.get("distance_m")
            if angle_deg is None or distance_m is None:
                continue

            try:
                encoded = self._encode_object(float(angle_deg), float(distance_m))
            except (TypeError, ValueError):
                encoded = None

            if encoded is not None:
                encoded_objects.append(encoded)

        if not encoded_objects:
            return

        object_size = int(config.CAN_OBSTACLE_DLC)
        max_dlc = int(getattr(config, "CAN_OBSTACLE_MAX_DLC", 8))
        max_objects_per_frame = max(1, max_dlc // object_size)

        for idx in range(0, len(encoded_objects), max_objects_per_frame):
            chunk = encoded_objects[idx : idx + max_objects_per_frame]
            self._send_frame(b"".join(chunk))