"""TODO: Add module documentation."""
from deepface import DeepFace
import numpy as np
from .base_encoder import BaseEncoder

class SFaceEncoder(BaseEncoder):
"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.model_name = 'SFace'

    def encode_face(self, face):
    """TODO: Describe encode_face.
Args:
    face (:obj:`Any`): TODO.
"""
        embedding_objs = DeepFace.represent(
            img_path=face,
            model_name=self.model_name,
            detector_backend='skip',
            enforce_detection=False
        )
        
        embeddings = np.array(embedding_objs[0]["embedding"])
        return embeddings / np.linalg.norm(embeddings)
