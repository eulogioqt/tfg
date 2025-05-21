import numpy as np
from queue import Queue
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from hri_msgs.msg import ChunkMono
from speech_msgs.srv import STT

from .utils.sound import play
from .utils.sounds import ACTIVATION_SOUND

from .utils import STTHotword, PVPorcupineHotword, IntensityAttachCriterion, SileroVADAttachCriterion


class AUDIO_STATE(int, Enum):
    NO_AUDIO = -1,
    SOME_AUDIO = 0,
    END_AUDIO = 1

class HELPER_STATE(int, Enum):
    COMMAND = 0,
    NAME = 1


# A futuro hacer que la forma de mezclar los chunks sea con un VAD no con threshold de intensidad
class AssistantHelperNode(Node):

    def __init__(self):
        super().__init__("assistant_helper")

        self.face_mode_pub = self.create_publisher(String, "face/mode", 10)
        self.assistant_text_pub = self.create_publisher(String, 'hri_audio/assistant_helper/transcription', 10)
        self.micro_sub = self.create_subscription(ChunkMono, 'hri_audio/microphone/mono', self.microphone_callback, 10)
        
        self.stt_client = self.create_client(STT, 'speech_tools/stt')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('STT service not available, waiting again...')
            
        self.transcribe_queue = Queue(maxsize=1)
        self.chunk_queue = Queue()

        self.get_logger().info("Assistant Helper Node initializated succesfully.")

    def microphone_callback(self, msg):
        new_audio = list([np.int16(x) for x in msg.chunk_mono])
        sample_rate = msg.sample_rate
        
        self.chunk_queue.put([new_audio, sample_rate])


class AssistantHelper:

    def __init__(self, name="Sancho"):
        self.name = name

        self.audio_state = AUDIO_STATE.NO_AUDIO
        self.helper_state = HELPER_STATE.NAME

        self.sample_rate = -1 # Will set on mic callbacks
        self.helper_chunk_size = 0.5 # Revisar este valor
        self.intensity_threshold = 1500

        self.audio = []
        self.check_audio = []
        self.previous_chunk = []

        #self.hotword_detector = STTHotword(self.stt_request, name=name)
        self.hotword_detector = PVPorcupineHotword() # Hacer un de esto para usar intensity o VAD para meter chunks
        self.chunk_attach_criterion = IntensityAttachCriterion(self.intensity_threshold)
        #self.chunk_attach_criterion = SileroVADAttachCriterion()
        
        self.node = AssistantHelperNode()

    def spin(self):
        while rclpy.ok():
            #self.node.get_logger().info(f"Queue size: {self.node.chunk_queue.qsize()}")
            if not self.node.chunk_queue.empty(): # Combine audio chunks
                [new_audio, self.sample_rate] = self.node.chunk_queue.get()

                if self.helper_state == HELPER_STATE.NAME: # Si NAME mode
                    self.process_name_mode(new_audio)

                elif self.helper_state == HELPER_STATE.COMMAND: # Si COMMAND mode
                    self.process_command_mode(new_audio)

            rclpy.spin_once(self.node)

    def process_name_mode(self, new_audio):
        if self.hotword_detector.detect(new_audio, self.sample_rate):
            self.node.face_mode_pub.publish(String(data="listening"))
            self.helper_state = HELPER_STATE.COMMAND

            play(ACTIVATION_SOUND)
            self.node.get_logger().info(f"✅✅✅ '{self.name.upper()}' DETECTED")

    def process_command_mode(self, new_audio):
        self.check_audio = self.check_audio + new_audio
        
        if self.is_audio_length(self.check_audio, self.helper_chunk_size):
            if self.chunk_attach_criterion.should_attach_chunk(self.check_audio, self.sample_rate):
                if self.audio_state == AUDIO_STATE.NO_AUDIO: # Por si es en END, no pase a SOME
                    self.audio_state = AUDIO_STATE.SOME_AUDIO

                if len(self.audio) == 0: # Si intensidad por primera vez, metemos chunk previo
                    self.audio = self.previous_chunk 

                self.audio = self.audio + self.check_audio # Luego el trozo nuevo
                self.node.get_logger().info(f"Chunk attached ({len(self.audio) / self.sample_rate}s)")
            elif self.audio_state != AUDIO_STATE.NO_AUDIO: # Si no hay intensidad y hay audio, terminamos trozo
                self.audio_state = AUDIO_STATE.END_AUDIO

                self.audio = self.audio + self.check_audio # Para que no se corte el audio, metemos tmb este ultimo
                self.node.get_logger().info("No more audio detected. Closing chunk window")
            else: # Si no hay intensidad ni audio, pues aun no hay audio
                self.node.get_logger().info("Listening for command but no audio detected yet")

            self.previous_chunk = self.check_audio
            self.check_audio = []

        if self.audio_state == AUDIO_STATE.END_AUDIO:
            self.process_audio_command(self.audio)
            
            self.audio = []
            self.check_audio = []
            self.audio_state = AUDIO_STATE.NO_AUDIO

            self.node.chunk_queue = Queue() # Limpiar lo que se ha acumulado mientras hace STT, mejor que que procese lo acumulado

    def process_audio_command(self, audio):
        self.node.get_logger().info(f"Transcribing {len(self.audio) / self.sample_rate}s chunk...")

        rec = self.stt_request(list(map(int, audio)), self.sample_rate)                
        if rec:
            self.helper_state = HELPER_STATE.NAME

            self.node.assistant_text_pub.publish(String(data=rec))
            self.node.get_logger().info(f"✅✅✅ Text transcribed ({len(audio) / self.sample_rate}s): {rec}")
        else:
            self.node.get_logger().info("Transcription result is empty.")

    def stt_request(self, audio, sample_rate):
        stt_request = STT.Request()
        stt_request.audio = audio
        stt_request.sample_rate = sample_rate

        future_stt = self.node.stt_client.call_async(stt_request)
        rclpy.spin_until_future_complete(self.node, future_stt)
        result_stt = future_stt.result()

        return result_stt.text
    
    def is_audio_length(self, audio, seconds):
        return len(audio) >= seconds * self.sample_rate


def main(args=None):
    rclpy.init(args=args)

    assistant_helper = AssistantHelper()

    assistant_helper.spin()
    rclpy.shutdown()