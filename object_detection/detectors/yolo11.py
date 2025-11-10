# Ultralytics YOLOv11 detector plugin. Pure Python, no ROS imports.

from typing import List, Optional
import numpy as np

from .base import DetectorBase, Det2D, Box2D


class YOLO11Detector(DetectorBase):
    def __init__(self):
        self.model = None
        self.names = None
        self.conf = 0.4
        self.iou = 0.45
        self.max_det = 200
        self.class_allow = set()

    def setup(self,
              model: str = "yolo11n.pt",
              conf: float = 0.4,
              iou: float = 0.45,
              max_det: int = 200,
              class_allowlist: Optional[list] = None,
              warmup: bool = True) -> None:
        from ultralytics import YOLO  # lazy import
        self.model = YOLO(model)
        self.names = self.model.names
        self.conf = float(conf)
        self.iou = float(iou)
        self.max_det = int(max_det)
        self.class_allow = set(class_allowlist or [])
        if warmup:
            try:
                self.model.predict(np.zeros((64, 64, 3), dtype=np.uint8),
                                   imgsz=64, conf=self.conf, iou=self.iou, verbose=False)
            except Exception:
                pass

    def detect(self, bgr_image) -> List[Det2D]:
        try:
            res = self.model.predict(
                bgr_image, conf=self.conf, iou=self.iou, max_det=self.max_det, verbose=False
            )[0]
        except Exception:
            return []

        out: List[Det2D] = []
        for b in res.boxes:
            cls_id = int(b.cls[0].item())
            score = float(b.conf[0].item())
            x1, y1, x2, y2 = map(float, b.xyxy[0].tolist())
            cls_name = self.names.get(cls_id, str(cls_id))
            if self.class_allow and cls_name not in self.class_allow:
                continue
            out.append(Det2D(cls=cls_name, score=score, box=Box2D(x1, y1, x2, y2)))
        return out
