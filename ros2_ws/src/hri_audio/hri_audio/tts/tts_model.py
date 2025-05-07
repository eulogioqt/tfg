from abc import ABC, abstractmethod

import numpy as np
import sounddevice as sd


class TTSModel(ABC):

    @abstractmethod
    def synthesize(self, text: str) -> np.ndarray:
        pass

    @abstractmethod
    def save(self, audio: np.ndarray, sample_rate: int, filename: str):
        pass

    def get_sample_rate(self) -> int:
        return self.sample_rate
    
    def get_speakers(self) -> list[str]:
        return self.speakers

    def play(self, audio: np.ndarray, sample_rate: int, wait: bool = True):
        try:
            sd.play(audio, samplerate=sample_rate)
            if wait:
                sd.wait()
        except Exception:
            print("Play audio skipped due to an error.")