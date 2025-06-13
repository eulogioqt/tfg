"""TODO: Add module documentation."""
from abc import ABC, abstractmethod


class ModelHotword(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def detect(self, audio_chunk: list[int], sample_rate: int) -> bool:
    """TODO: Describe detect.
Args:
    audio_chunk (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
"""
        pass
