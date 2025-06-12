import cv2
import numpy as np
from yolo5face.get_model import get_model
from .base_detector import BaseDetector

class YOLOv5FaceDetector(BaseDetector):
    def __init__(self, model_type="yolov5n", min_face=24, device=None):
        self.model = get_model(
            model_type,
            device=device,
            min_face=min_face
        )

    def get_faces(self, frame, verbose=False):
        boxes, points, scores = self.model.detector.predict(frame, target_size=640)

        face_positions = [
            (int(x), int(y), int(w - x), int(h - y)) for x, y, w, h in boxes
        ]
        face_scores = [float(s) for s in scores]

        if verbose:
            disp = frame.copy()
            for (x, y, w, h) in face_positions:
                cv2.rectangle(disp, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.imshow("YOLOv5-Face Detection", disp)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return face_positions, face_scores
