import cv2
import torch
import numpy as np

from .base_detector import BaseDetector


class EfficientFaceDetector(BaseDetector):

    def __init__(self, model_path="efficientface.pth", device="cuda" if torch.cuda.is_available() else "cpu"):
        self.device = device
        # Carga del modelo. Aquí se asume que tienes una clase EfficientFace ya definida.
        from efficientface.model import EfficientFace  # ajusta esto a tu estructura
        self.model = EfficientFace()
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device).eval()

    def preprocess(self, image):
        # Asegúrate de adaptar esto al tamaño que espera el modelo
        resized = cv2.resize(image, (640, 640))
        img = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        img = img.transpose(2, 0, 1) / 255.0  # CHW, normalización
        return torch.tensor(img, dtype=torch.float32).unsqueeze(0).to(self.device)

    def postprocess(self, preds, original_shape):
        # Aquí es donde transformas predicciones al formato [(x, y, w, h)]
        boxes = []
        scores = []
        h_orig, w_orig = original_shape[:2]
        for det in preds:  # depende del modelo, aquí se simula
            x1, y1, x2, y2, score = det
            x, y = int(x1 * w_orig), int(y1 * h_orig)
            w, h = int((x2 - x1) * w_orig), int((y2 - y1) * h_orig)
            boxes.append((x, y, w, h))
            scores.append(float(score))
        return boxes, scores

    def get_faces(self, frame, verbose=False):
        frame_copy = frame.copy()
        img_tensor = self.preprocess(frame)
        with torch.no_grad():
            output = self.model(img_tensor)[0].cpu().numpy()  # asumimos salida tipo [N, 5]

        face_positions, scores = self.postprocess(output, frame.shape)

        if verbose:
            for (x, y, w, h) in face_positions:
                cv2.rectangle(frame_copy, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.imshow("EfficientFace Detection", frame_copy)

        return face_positions, scores
