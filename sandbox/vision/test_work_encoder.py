import os
import numpy as np
import cv2
import traceback

from .encoders import FacenetEncoder, ArcFaceEncoder, DinoV2Encoder, OpenFaceEncoder, SFaceEncoder, VGGFaceEncoder

def test_encoder(name, encoder_cls, image_path):
    print(f"\n[INFO] Probando encoder: {name}")
    try:
        encoder = encoder_cls()
        image = cv2.imread(image_path)
        if image is None:
            raise ValueError("No se pudo cargar la imagen. Verifica que el archivo existe y es válido.")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        embedding = encoder.encode_face(image)
        if embedding is None:
            raise ValueError("El encoder devolvió None. Revisa si detecta cara correctamente.")
        print(f"[OK] {name} funcionando. Dimensión del embedding: {embedding.shape}, Norma: {np.linalg.norm(embedding):.4f}")
    except Exception as e:
        print(f"[ERROR] Fallo en {name}: {e}")
        traceback.print_exc()

if __name__ == "__main__":
    encoders = {
        "FaceNet": FacenetEncoder,
        "ArcFace": ArcFaceEncoder,
        "DINOv2": DinoV2Encoder,
        "OpenFace": OpenFaceEncoder,
        "SFace": SFaceEncoder,
        "VGGFace": VGGFaceEncoder
    }

    current_dir = os.path.dirname(os.path.abspath(__file__))
    image_path = os.path.join(current_dir, "face.jpg")

    if not os.path.exists(image_path):
        print(f"[ERROR] Imagen face.jpg no encontrada en {current_dir}")
    else:
        for name, encoder_cls in encoders.items():
            test_encoder(name, encoder_cls, image_path)
