import cv2
from ultralytics import YOLO
import numpy as np

from .base_detector import BaseDetector


class YOLOv8FaceDetector(BaseDetector):

    def __init__(self, model_path="yolov8n-face.pt"):
        self.model = YOLO(model_path)

    def get_faces(self, frame, verbose=False):
        frame_copy = frame.copy()
        results = self.model.predict(frame, conf=0.25, verbose=False)[0]

        face_positions = []
        scores = []

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = box.conf.item()
            x, y = int(x1), int(y1)
            w, h = int(x2 - x1), int(y2 - y1)
            face_positions.append((x, y, w, h))
            scores.append(conf)

            if verbose:
                cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

        if verbose:
            cv2.imshow('YOLOv8-Face Detection', frame_copy)

        return face_positions, scores
