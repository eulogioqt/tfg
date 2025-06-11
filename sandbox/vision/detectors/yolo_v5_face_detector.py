import cv2
import torch
import numpy as np

from .base_detector import BaseDetector


class YOLOv5FaceDetector(BaseDetector):

    def __init__(self, model_path="yolov5face.pt"):
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.model.conf = 0.25

    def get_faces(self, frame, verbose=False):
        frame_copy = frame.copy()
        results = self.model(frame)
        detections = results.xyxy[0].cpu().numpy()

        face_positions = []
        scores = []

        for x1, y1, x2, y2, conf, cls in detections:
            x, y = int(x1), int(y1)
            w, h = int(x2 - x1), int(y2 - y1)
            face_positions.append((x, y, w, h))
            scores.append(float(conf))

            if verbose:
                cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

        if verbose:
            cv2.imshow('YOLOv5-Face Detection', frame_copy)

        return face_positions, scores
