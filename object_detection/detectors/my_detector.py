"""
MyDetector template.

Goal:
- Implement a detector that conforms to DetectorBase.
- Use any framework you already learned (PyTorch, TensorFlow, ONNXRuntime, OpenCV, etc.).
- Return a List[Det2D] where each Det2D contains (cls, score, Box2D(x1,y1,x2,y2)) in pixel coords.

Reference API: Det2D, Box2D, DetectorBase. See base.py.  :contentReference[oaicite:0]{index=0}
For a full example using Ultralytics YOLOv11 see yolo11.py.           :contentReference[oaicite:1]{index=1}

Steps you must fill in:
1) setup(...)
   - Load your model and any labels.
   - Configure thresholds and device.
2) detect(bgr_image)
   - Preprocess BGR frame as your model expects.
   - Run inference.
   - Postprocess to pixel-space boxes (x1,y1,x2,y2), class names, scores.
   - Return a list of Det2D.

Keep ROS out of this file. Accept and return pure Python/Numpy types only.
"""

from typing import List, Optional, Any, Dict
import numpy as np

from .base import DetectorBase, Det2D, Box2D  # contract you must satisfy  :contentReference[oaicite:2]{index=2}


class MyDetector(DetectorBase):
    def __init__(self):
        # Put minimal defaults here. Avoid heavy imports here.
        self.model = None
        self.class_names: Optional[List[str]] = None
        self.conf: float = 0.4
        self.iou: float = 0.45  # if your framework uses it
        self.input_size: Optional[int] = None  # e.g., 640
        self.device: str = "cpu"

    def setup(self,
              # Common knobs. Extend as needed.
              weights: Optional[str] = None,
              class_names: Optional[List[str]] = None,
              conf: float = 0.4,
              iou: float = 0.45,
              input_size: Optional[int] = None,
              device: str = "cpu",
              **kwargs: Dict[str, Any]) -> None:
        """
        Load your model here.

        Examples you can implement (choose one):
        - PyTorch:
            import torch
            self.model = torch.jit.load(weights, map_location=device)  # or torch.load(...)
            self.model.eval()
        - TensorFlow/Keras:
            import tensorflow as tf
            self.model = tf.saved_model.load(weights)  # or tf.keras.models.load_model(...)
        - ONNXRuntime:
            import onnxruntime as ort
            self.model = ort.InferenceSession(weights, providers=["CPUExecutionProvider"])
        - OpenCV DNN:
            import cv2
            self.model = cv2.dnn.readNet(weights, config)  # set preferable backend/target
        """
        self.class_names = class_names
        self.conf = float(conf)
        self.iou = float(iou)
        self.input_size = input_size
        self.device = device

        # TODO(student): load your model according to the chosen framework.
        # Keep this method fast to call from detect_and_draw.py.
        self.model = None  # replace with actual loaded model

    def detect(self, bgr_image) -> List[Det2D]:
        """
        Input:
            bgr_image: HxWx3 uint8, OpenCV BGR.
        Output:
            List[Det2D] with pixel-space boxes and class labels.

        Pseudocode outline:
        1) PREPROCESS
           - If model expects RGB: rgb = bgr_image[:, :, ::-1]
           - Resize to self.input_size if needed, keep scale for later box mapping.
           - Normalize/standardize per your model.

        2) INFERENCE
           - Run the model.
           - Get raw outputs (boxes, scores, class IDs). Apply NMS if your framework does not.

        3) POSTPROCESS
           - Map boxes back to original image size (undo resize/letterbox).
           - Convert to Det2D items using class name strings.
           - Apply confidence threshold self.conf.

        Return [] if inference fails.
        """
        if bgr_image is None:
            return []

        H, W = bgr_image.shape[:2]

        # ------- Example scaffolding below. Replace with your real pipeline. -------
        # Convert to RGB if needed by your framework
        # rgb = bgr_image[:, :, ::-1]

        # Resize if needed
        # if self.input_size:
        #     resized = cv2.resize(bgr_image, (self.input_size, self.input_size))
        #     scale_x = W / self.input_size
        #     scale_y = H / self.input_size
        # else:
        #     resized = bgr_image
        #     scale_x = scale_y = 1.0

        # Run your model here and obtain: boxes_xyxy[N,4], scores[N], class_ids[N]
        # boxes_xyxy = np.array([[50, 40, 180, 160], ...], dtype=np.float32)
        # scores = np.array([0.9, ...], dtype=np.float32)
        # class_ids = np.array([0, ...], dtype=np.int32)

        # Example: empty output until student fills in
        boxes_xyxy = np.empty((0, 4), dtype=np.float32)
        scores = np.empty((0,), dtype=np.float32)
        class_ids = np.empty((0,), dtype=np.int32)

        # Map boxes if you resized:
        # boxes_xyxy[:, [0, 2]] *= scale_x
        # boxes_xyxy[:, [1, 3]] *= scale_y

        # Class name resolution
        def id_to_name(i: int) -> str:
            if self.class_names and 0 <= i < len(self.class_names):
                return self.class_names[i]
            return str(i)

        out: List[Det2D] = []
        for box, score, cid in zip(boxes_xyxy, scores, class_ids):
            if float(score) < self.conf:
                continue
            x1, y1, x2, y2 = map(float, box.tolist())
            out.append(Det2D(cls=id_to_name(int(cid)),
                             score=float(score),
                             box=Box2D(x1, y1, x2, y2)))
        return out
