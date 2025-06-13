"""TODO: Add module documentation."""
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 
import tensorflow as tf

gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
        print("Asignación de memoria dinámica activada")
    except RuntimeError as e:
        print(e)

from keras_facenet import FaceNet
import numpy as np

from .base_encoder import BaseEncoder


class FacenetEncoder(BaseEncoder):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.model = FaceNet()

    def encode_face(self, face):
    """TODO: Describe encode_face.
Args:
    face (:obj:`Any`): TODO.
"""
        shapeFace = np.expand_dims(face, axis=0)
        features = self.model.embeddings(shapeFace)[0]

        return features
