import cv2
import insightface
from insightface.app import FaceAnalysis
from .base_detector import BaseDetector

class EfficientFaceDetector(BaseDetector):
    def __init__(self, conf_thresh=0.6, providers=None):
        self.conf_thresh = conf_thresh
        self.app = FaceAnalysis(allowed_modules=['detection'], providers=providers or ['cpu'])
        self.app.prepare(ctx_id=0, det_size=(640, 640))

    def get_faces(self, frame, verbose=False):
        # frame se asume en RGB
        faces = self.app.get(frame)
        face_positions = []
        face_scores = []

        for face in faces:
            box = face.bbox.astype(int)  # [x1, y1, x2, y2]
            score = face.det_score
            if score < self.conf_thresh:
                continue
            x1, y1, x2, y2 = box
            face_positions.append((x1, y1, x2 - x1, y2 - y1))
            face_scores.append(float(score))
            if verbose:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if verbose and face_positions:
            cv2.imshow("EfficientFace Detection", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return face_positions, face_scores
