"""TODO: Add module documentation."""
from abc import ABC, abstractmethod


class ChunkAttachCriterion(ABC):

    @abstractmethod
"""TODO: Describe class."""
    def should_attach_chunk(self, chunk, sample_rate):
    """TODO: Describe should_attach_chunk.
Args:
    chunk (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
"""
        pass
