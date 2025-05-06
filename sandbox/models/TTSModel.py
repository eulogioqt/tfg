from abc import ABC, abstractmethod

import numpy as np
import sounddevice as sd


class TTSModel(ABC):

    @abstractmethod
    def get_freq(self) -> int:
        pass

    @abstractmethod
    def synthesize(self, text: str) -> np.ndarray:
        pass

    @abstractmethod
    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        pass

    def play(self, audio: np.ndarray, sample_rate: int, wait: bool = True):
        try:
            sd.play(audio, samplerate=sample_rate)
            if wait:
                sd.wait()
        except Exception:
            print("Play audio skipped due to an error.")