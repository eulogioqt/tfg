"""TODO: Add module documentation."""
import pyaudio
import numpy as np

import rclpy
from rclpy.node import Node

from hri_msgs.msg import ChunkMono, ChunkStereo


class MicrophoneCapturerNode(Node):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__("microphone_capturer")

        self.publisher_stereo = self.create_publisher(ChunkStereo, 'hri_audio/microphone/stereo', 10)
        self.publisher_mono = self.create_publisher(ChunkMono, 'hri_audio/microphone/mono', 10)


class MicrophoneCapturer:

"""TODO: Describe class."""
    def __init__(self, device_name="orbbec", chunk_size=1024):
    """TODO: Describe __init__.
Args:
    device_name (:obj:`Any`): TODO.
    chunk_size (:obj:`Any`): TODO.
"""
        device = self.get_device_by_name(device_name)
        if not device:
            raise Exception(f"Microphone with name {device_name} not found.")

        self.device_index = device['index']
        self.sample_rate = int(device['defaultSampleRate'])
        self.num_channels = device['maxInputChannels']
        self.chunk_size = chunk_size

        self.stream = self.setup_microphone(self.device_index, self.sample_rate, self.num_channels, self.chunk_size)

        self.node = MicrophoneCapturerNode()

    def spin(self):
    """TODO: Describe spin.
"""
        first = True

        while rclpy.ok():
            try:      
                chunk_stereo = ChunkStereo(sample_rate=self.sample_rate)
                chunk_mono = ChunkMono(sample_rate=self.sample_rate)
                
                data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                data = np.frombuffer(data, dtype=np.int16)

                audio_mix = np.ndarray.tolist(data)

                if self.num_channels == 1:
                    chunk_mono.chunk_mono = audio_mix

                elif self.num_channels >= 2:
                    chunk_stereo.chunk_left = audio_mix[::2]
                    chunk_stereo.chunk_right  = audio_mix[1::2]

                    chunk_mono.chunk_mono = audio_mix[::2] # Probar a hacer algun metodo de mono to stereo

                    self.node.publisher_stereo.publish(chunk_stereo)

                if first:
                    self.node.get_logger().info("First Audio Chunk published succesfully")
                    first = False
                
                self.node.publisher_mono.publish(chunk_mono)
            except Exception as e:
                 self.node.get_logger().info(f">> Microphone Publisher Error: {e}")
    
    def get_device_by_name(self, device_name):
    """TODO: Describe get_device_by_name.
Args:
    device_name (:obj:`Any`): TODO.
"""
        p = pyaudio.PyAudio()

        device = None
        for i in range(p.get_device_count()):
            dev = p.get_device_info_by_index(i)
            if dev['maxInputChannels'] > 0:
                print(f"{i}: {dev['name']}")
                if device_name.lower() in dev['name'].lower():
                    device = dev
                    p.terminate()
                    break

        return device

    def setup_microphone(self, device_index, sample_rate, num_channels, chunk_size):
    """TODO: Describe setup_microphone.
Args:
    device_index (:obj:`Any`): TODO.
    sample_rate (:obj:`Any`): TODO.
    num_channels (:obj:`Any`): TODO.
    chunk_size (:obj:`Any`): TODO.
"""
        p = pyaudio.PyAudio()

        return p.open(format=pyaudio.paInt16, channels=num_channels, rate=sample_rate,
                      input=True, frames_per_buffer=chunk_size, input_device_index=device_index)
     

def main(args=None):
"""TODO: Describe main.
Args:
    args (:obj:`Any`): TODO.
"""
    rclpy.init(args=args)

    microphone_capturer = MicrophoneCapturer()

    microphone_capturer.spin()
    rclpy.shutdown()
