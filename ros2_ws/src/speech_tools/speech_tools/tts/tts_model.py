from abc import ABC, abstractmethod

import numpy as np


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