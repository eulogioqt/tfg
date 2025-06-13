"""TODO: Add module documentation."""
import gc
import torch
import numpy as np
import soundfile as sf
import sounddevice as sd

from abc import ABC, abstractmethod


class TTSModel(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def synthesize(self, text: str, speaker: str) -> tuple[list[int], str]:
    """TODO: Describe synthesize.
Args:
    text (:obj:`Any`): TODO.
    speaker (:obj:`Any`): TODO.
"""
        pass
    
    @abstractmethod
    def get_sample_rate(self) -> int:
    """TODO: Describe get_sample_rate.
"""
        pass
    
    @abstractmethod
    def get_speakers(self) -> list[str]:
    """TODO: Describe get_speakers.
"""
        pass

    def unload(self):
    """TODO: Describe unload.
"""
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        gc.collect()

    @staticmethod
    def save(filepath, audio, sample_rate):
    """TODO: Describe save.
Args:
    filepath (:obj:`Any`): TODO.
    audio (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
"""
        audio = np.array(audio, dtype=np.int16)
        
        sf.write(filepath, audio, samplerate=sample_rate, subtype="PCM_16")

    @staticmethod
    def play(audio, sample_rate, wait=True):
    """TODO: Describe play.
Args:
    audio (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
    wait (:obj:`Any`): TODO.
"""
        audio = np.array(audio, dtype=np.int16)

        sd.play(audio, samplerate=sample_rate)
        if wait:
            sd.wait()
