import cv2
import numpy as np

from .base_detector import BaseDetector


class RetinaFaceDetector(BaseDetector):

    def __init__(self):
        from retinaface import RetinaFace
        pass  # RetinaFace usa modelo embebido

    def get_faces(self, frame, verbose=False):
        frame_copy = frame.copy()
        results = RetinaFace.detect_faces(frame)
        
        face_positions = []
        scores = []

        if isinstance(results, dict):
            for key in results:
                facial_area = results[key]["facial_area"]
                x1, y1, x2, y2 = facial_area
                x, y = int(x1), int(y1)
                w, h = int(x2 - x1), int(y2 - y1)
                face_positions.append((x, y, w, h))
                scores.append(results[key].get("score", 1.0))

                if verbose:
                    cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

        if verbose:
            cv2.imshow('RetinaFace Detection', frame_copy)

        return face_positions, scores
