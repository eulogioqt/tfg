import gc
import torch
import numpy as np

from abc import ABC, abstractmethod


class TTSModel(ABC):

    @abstractmethod
    def synthesize(self, text: str, speaker: str) -> np.ndarray:
        pass

    @abstractmethod
    def save(self, audio: np.ndarray, filename: str):
        pass

    @abstractmethod
    def get_sample_rate(self) -> int:
        pass
    
    @abstractmethod
    def get_speakers(self) -> list[str]:
        pass

    def unload(self):
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        gc.collect()