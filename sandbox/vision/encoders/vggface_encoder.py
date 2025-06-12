from deepface import DeepFace
import numpy as np
from .base_encoder import BaseEncoder

class VGGFaceEncoder(BaseEncoder):
    def __init__(self):
        self.model_name = 'VGG-Face'

    def encode_face(self, face):
        embedding_objs = DeepFace.represent(
            img_path=face,
            model_name=self.model_name,
            detector_backend='skip',
            enforce_detection=False
        )
        return np.array(embedding_objs[0]["embedding"])
