"""TODO: Add module documentation."""
import gc
import torch

from abc import ABC, abstractmethod


class STTModel(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def transcribe(self, audio: list[int], sample_rate: int) -> str:
    """TODO: Describe transcribe.
Args:
    audio (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
"""
        pass

    def unload(self):
    """TODO: Describe unload.
"""
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        gc.collect()
