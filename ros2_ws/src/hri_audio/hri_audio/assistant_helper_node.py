import re
import unidecode
import numpy as np
from queue import Queue
from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from hri_msgs.msg import ChunkMono
from hri_msgs.srv import AudioRecognition

from .api.sound import play
from .api.sounds import ACTIVATION_SOUND


class AUDIO_STATE(int, Enum):
    NO_AUDIO = -1,
    SOME_AUDIO = 0,
    END_AUDIO = 1

class HELPER_STATE(int, Enum):
    COMMAND = 0,
    NAME = 1


# Cambiar el check each seconds si estamos en name o si estamos en command? En name 0.5 y en command 1?
class AssistantHelperNode(Node):

    def __init__(self):
        super().__init__("assistant_helper")

        self.assistant_text_pub = self.create_publisher(String, 'hri_audio/assistant_helper/transcription', 10)
        self.micro_sub = self.create_subscription(ChunkMono, 'hri_audio/microphone/mono', self.microphone_callback, 10)
        
        self.audio_recognition_client = self.create_client(AudioRecognition, 'hri_audio/stt')
        while not self.audio_recognition_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Audio recognition service not available, waiting again...')
            
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
        self.check_each_seconds = 0.5
        self.intensity_threshold = 750
        self.name_max_seconds = 1.5

        self.audio = []
        self.check_audio = []
        self.previous_chunk = []

        self.node = AssistantHelperNode()

    def spin(self):
        while rclpy.ok():
            if not self.node.chunk_queue.empty(): # Combine audio chunks
                [new_audio, sample_rate] = self.node.chunk_queue.get()

                self.check_audio = self.check_audio + new_audio
                self.sample_rate = sample_rate # Sample rate set
                
                if self.is_audio_length(self.check_audio, self.check_each_seconds):
                    avg_intensity = self.audio_average_intensity(self.check_audio)
                    self.node.get_logger().info(f"Chunk of {self.check_each_seconds}s: {avg_intensity:.2f} avg intensity")

                    if avg_intensity >= self.intensity_threshold: # Si hay intensidad, lo añadimos
                        if self.audio_state == AUDIO_STATE.NO_AUDIO: # Por si es en END, no pase a SOME
                            self.audio_state = AUDIO_STATE.SOME_AUDIO

                        if len(self.audio) == 0: # Si intensidad por primera vez, metemos chunk previo
                            self.audio = self.previous_chunk 

                        self.audio = self.audio + self.check_audio # Luego el trozo nuevo
                        self.node.get_logger().info(f"Chunk attached ({len(self.audio) / self.sample_rate}s)")
                    elif self.audio_state != AUDIO_STATE.NO_AUDIO: # Si no hay intensidad y hay audio, terminamos trozo
                        self.audio_state = AUDIO_STATE.END_AUDIO

                        self.audio = self.audio + self.check_audio # Para que no se corte el audio, metemos tmb este ultimo
                        self.node.get_logger().info("No more audio detected...")
                    else: # Si no hay intensidad ni audio, pues aun no hay audio
                        self.node.get_logger().info("No audio detected yet")

                    self.previous_chunk = self.check_audio
                    self.check_audio = []

                if self.audio_state == AUDIO_STATE.END_AUDIO:
                    if self.node.transcribe_queue.qsize() < 1:
                        self.node.get_logger().info(f"Adding {len(self.audio) / self.sample_rate}s chunk to transcribe queue")
                        self.node.transcribe_queue.put(self.audio)
                    else:
                        self.node.get_logger().info("Transcribe Queue IS FULL!!!")
                    
                    self.audio = []
                    self.check_audio = []
                    self.audio_state = AUDIO_STATE.NO_AUDIO

                # Si ya nos hemos llegado al limite, modo end audio, que añada el siguiente chunk y termine
                if self.helper_state == HELPER_STATE.NAME and self.is_audio_length(self.audio, self.name_max_seconds):
                    self.node.get_logger().info(f"Name mode audio {len(self.audio) / self.sample_rate}s >= {self.name_max_seconds}s -> End audio mode")
                    self.audio_state = AUDIO_STATE.END_AUDIO

            if not self.node.transcribe_queue.empty(): # Transcribe audio chunks
                audio = self.node.transcribe_queue.get()
                self.node.get_logger().info("Transcribing...")
        
                rec = self.audio_recognition_request(audio)                
                if rec:
                    self.node.get_logger().info(f"Text transcribed ({len(audio) / self.sample_rate}s): {rec}")
                    
                    rec_processed = rec.strip().lower()
                    rec_processed = unidecode.unidecode(rec_processed)
                    rec_processed = re.sub(r'[^a-z\s]', '', rec_processed)
                    
                    # Cambiar por porcurpine nojeke para hotword en plan mas pro
                    if (self.name.lower() in rec_processed and self.helper_state == HELPER_STATE.NAME) \
                        or (self.name.lower() == rec_processed and self.helper_state == HELPER_STATE.COMMAND):
                        self.helper_state = HELPER_STATE.COMMAND
                        
                        play(ACTIVATION_SOUND)
                    elif self.helper_state == HELPER_STATE.COMMAND:
                        self.helper_state = HELPER_STATE.NAME
                        self.node.assistant_text_pub.publish(String(data=rec))
            
                else:
                    self.node.get_logger().info("Transcription result is None: Nothing transcribed")

                self.node.get_logger().info(f"Average intensity: {self.audio_average_intensity(audio)}")

            rclpy.spin_once(self.node)

    def audio_recognition_request(self, audio):
        audio_recognition_request = AudioRecognition.Request()

        audio = np.ndarray.tolist(np.array(audio)) # This is weird. Without this conversion it doesn't work. Need to see why

        audio_recognition_request.audio = audio
        audio_recognition_request.sample_rate = self.sample_rate

        future_audio_recognition = self.node.audio_recognition_client.call_async(audio_recognition_request)
        rclpy.spin_until_future_complete(self.node, future_audio_recognition)
        result_audio_recognition = future_audio_recognition.result()

        return result_audio_recognition.text
    
    def is_audio_length(self, audio, seconds):
        return len(audio) >= seconds * self.sample_rate

    def audio_average_intensity(self, audio):
        average_intensity = np.mean(np.abs(audio))
        if average_intensity < 0:
            average_intensity = 32767
            
        return average_intensity
    

def main(args=None):
    rclpy.init(args=args)

    assistant_helper = AssistantHelper()

    assistant_helper.spin()
    rclpy.shutdown()