import cv2
from typing import List
from detectors.base import Det2D  # uses Box2D coords as in base.py

def draw_detections(img, dets: List[Det2D]):
    h, w = img.shape[:2]
    for d in dets:
        x1, y1 = int(d.box.x1), int(d.box.y1)
        x2, y2 = int(d.box.x2), int(d.box.y2)
        # Clamp to image bounds
        x1 = max(0, min(x1, w - 1)); x2 = max(0, min(x2, w - 1))
        y1 = max(0, min(y1, h - 1)); y2 = max(0, min(y2, h - 1))
        # Box
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2, lineType=cv2.LINE_AA)
        # Label text
        label = f"{d.cls} {d.score:.2f}"
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        y1t = max(0, y1 - 8)
        cv2.rectangle(img, (x1, y1t - th - 4), (x1 + tw + 2, y1t), (0, 255, 0), -1)
        cv2.putText(img, label, (x1 + 1, y1t - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return img
