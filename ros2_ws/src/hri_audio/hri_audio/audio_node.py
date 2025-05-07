import wave
import numpy as np
import time

import rclpy
from rclpy.node import Node

from hri_msgs.msg import ChunkMono, ChunkStereo


class AudioNode(Node):

    def __init__(self, wav_path="./sounds/sample.wav", chunk_size=1024):
        super().__init__('audio')

        self.publisher_stereo = self.create_publisher(ChunkStereo, 'hri_audio/microphone/stereo', 10)
        self.publisher_mono = self.create_publisher(ChunkMono, 'hri_audio/microphone/mono', 10)

        self.wav_path = wav_path
        self.chunk_size = chunk_size

        self.get_logger().info(f"Microphone Node initializated succesfully with wav path: {wav_path}")


    def spin(self):
        first = True

        while rclpy.ok():
            try:
                wf = wave.open(self.wav_path, 'rb')

                sample_rate = wf.getframerate()
                num_channels = wf.getnchannels()

                self.get_logger().info(f"Reproduciendo {self.wav_path} a {sample_rate} Hz con {num_channels} canales")

                while True:
                    chunk_stereo = ChunkStereo(sample_rate=sample_rate)
                    chunk_mono = ChunkMono(sample_rate=sample_rate)

                    data = wf.readframes(self.chunk_size)
                    data = np.frombuffer(data, dtype=np.int16)
                    if len(data) == 0:
                        break

                    audio_mix = np.ndarray.tolist(data)

                    if num_channels == 1:
                        chunk_mono.chunk_mono = audio_mix
                    elif num_channels >= 2:
                        chunk_stereo.chunk_left = audio_mix[::2]
                        chunk_stereo.chunk_right = audio_mix[1::2]

                        chunk_mono.chunk_mono = audio_mix[::2]

                        self.publisher_stereo.publish(chunk_stereo)

                    if first:
                        self.get_logger().info("First Audio Chunk published succesfully")
                        first = False

                    self.publisher_mono.publish(chunk_mono)

                    time.sleep(self.chunk_size / sample_rate)

                self.get_logger().info("Reproducción terminada, esperando 5s antes de repetir")
                time.sleep(5)
                wf.close()

            except Exception as e:
                self.get_logger().error(f"Error durante la simulación de audio: {e}")
                time.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    audio = AudioNode()

    audio.spin()
    rclpy.shutdown()
