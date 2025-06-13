"""TODO: Add module documentation."""
import torch
import numpy as np

from rclpy.node import Node
from scipy.signal import resample_poly


class IntensityAttachCriterion(Node):
    
"""TODO: Describe class."""
    def __init__(self, intensity_threshold):
    """TODO: Describe __init__.
Args:
    intensity_threshold (:obj:`Any`): TODO.
"""
        self.intensity_threshold = intensity_threshold

    def should_attach_chunk(self, chunk, _):
    """TODO: Describe should_attach_chunk.
Args:
    chunk (:obj:`Any`): TODO.
    _ (:obj:`Any`): TODO.
"""
        avg_intensity = self.audio_average_intensity(chunk)

        return avg_intensity >= self.intensity_threshold
    
    def audio_average_intensity(self, chunk):
    """TODO: Describe audio_average_intensity.
Args:
    chunk (:obj:`Any`): TODO.
"""
        average_intensity = np.mean(np.abs(chunk))
        if average_intensity < 0:
            average_intensity = 32767
            
        return average_intensity
