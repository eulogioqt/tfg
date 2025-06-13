"""TODO: Add module documentation."""
import cv2

from .base_detector import BaseDetector


class CV2Detector(BaseDetector):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.face_detector = cv2.CascadeClassifier('/home/ubuntu/tfg/ros2_ws/src/hri_vision/hri_vision/models/haarcascade_frontalface_default.xml')

    def get_faces(self, frame, verbose=False):
    """TODO: Describe get_faces.
Args:
    frame (:obj:`Any`): TODO.
    verbose (:obj:`Any`): TODO.
"""
        frame_copy = frame.copy()
        faces = self.face_detector.detectMultiScale(frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        scores = []
        face_positions = []
        for (x, y, w, h) in faces:
            face_positions.append((x, y, w, h))
            scores.append(10)

            if verbose:
                cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)

        if verbose:
            cv2.imshow('Face Detection', frame_copy)

        return face_positions, scores
