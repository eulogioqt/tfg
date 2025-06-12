import os
import cv2
import numpy as np
import traceback

from .detectors import (
    CV2Detector, DLIBCNNDetector, DLIBFrontalDetector,
    MTCNNDetector, InsightFaceDetector,
    RetinaFaceDetector, YOLOv5FaceDetector, YOLOv8FaceDetector
)

def test_detector(name, detector_cls, image_path):
    print(f"\n[INFO] Probando detector: {name}")
    try:
        detector = detector_cls()
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("No se pudo cargar la imagen. Verifica que el archivo existe y es válido.")
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        boxes, _ = detector.get_faces(image_rgb)
        print(boxes)

        if not boxes:
            raise ValueError("No se detectaron caras.")
        
        print(f"[OK] {name} funcionando. Caras detectadas: {len(boxes)}")
        for i, (x, y, w, h) in enumerate(boxes):
            print(f" - Cara {i+1}: (x={x}, y={y}, w={w}, h={h})")
    except Exception as e:
        print(f"[ERROR] Fallo en {name}: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    detectors = {
        "cv2": CV2Detector,
        "dlib_cnn": DLIBCNNDetector,
        "dlib_frontal": DLIBFrontalDetector,
        "mtcnn": MTCNNDetector,
        "yolov5": YOLOv5FaceDetector,
        "yolov8": YOLOv8FaceDetector,
        "retinaface": RetinaFaceDetector,
        "insightface": InsightFaceDetector
    }

    current_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(current_dir, "face.jpg")

    if not os.path.exists(image_path):
        print(f"[ERROR] Imagen face.jpg no encontrada en {current_dir}")
    else:
        for name, detector_cls in detectors.items():
            test_detector(name, detector_cls, image_path)
