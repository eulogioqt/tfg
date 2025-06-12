import os
import torch
import numpy as np
from PIL import Image
from dotenv import load_dotenv
from huggingface_hub import login
from transformers import AutoImageProcessor, AutoModel
from .base_encoder import BaseEncoder

class DinoV2Encoder(BaseEncoder):
    def __init__(self, device='cuda' if torch.cuda.is_available() else 'cpu'):
        load_dotenv()
        hf_token = os.getenv("HUGGING_FACE_API_KEY")

        if hf_token:
            login(token=hf_token)
        else:
            raise EnvironmentError("HUGGING_FACE_API_KEY no encontrada en .env")

        model_name = "facebook/dinov2-base"
        self.processor = AutoImageProcessor.from_pretrained(model_name, token=hf_token)
        self.model = AutoModel.from_pretrained(model_name, token=hf_token).to(device)
        self.device = device

    def encode_face(self, face):
        if isinstance(face, np.ndarray):
            face = Image.fromarray(face)

        inputs = self.processor(images=face, return_tensors="pt").to(self.device)

        with torch.no_grad():
            outputs = self.model(**inputs)
            embedding = outputs.last_hidden_state.mean(dim=1).squeeze().cpu().numpy()

        return embedding / np.linalg.norm(embedding)
