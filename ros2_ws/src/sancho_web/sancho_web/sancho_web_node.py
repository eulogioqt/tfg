import rclpy
from rclpy.node import Node

from queue import Queue

from .protocol import (
    MessageType, JSONMessage, 
    PromptMessage, AudioPromptChunkMessage,
    ResponseMessage, FaceprintEventMessage, PromptTranscriptionMessage, AudioResponseChunkMessage,
    parse_message
)

from ros2web_msgs.msg import R2WMessage
from hri_msgs.msg import FaceprintEvent
from hri_msgs.srv import SanchoPrompt
from speech_msgs.srv import STT, TTS


class SanchoWebNode(Node):

    def __init__(self):
        super().__init__("sancho_web")

        self.web_queue = Queue()
        self.msg_queue = Queue()

        self.ros_pub = self.create_publisher(R2WMessage, "ros2web/ros", 10) # All publish here go to web
        self.web_sub = self.create_subscription(R2WMessage, "ros2web/web", self.web_callback, 10) # All received here comes from web

        self.faceprint_event_sub = self.create_subscription(FaceprintEvent, "recognition/event", self.faceprint_event_callback, 10)
        self.event_map = {
            FaceprintEvent.CREATE: FaceprintEventMessage.Event.CREATE,
            FaceprintEvent.UPDATE: FaceprintEventMessage.Event.UPDATE,
            FaceprintEvent.DELETE: FaceprintEventMessage.Event.DELETE,
        }

        self.sancho_prompt_client = self.create_client(SanchoPrompt, "sancho_ai/prompt")
        while not self.sancho_prompt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Sancho Prompt Service not available, waiting...")

        self.stt_client = self.create_client(STT, 'speech_tools/stt')
        while not self.stt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('STT service not available, waiting again...')
        # Hacer un mensaje notification o algo asi para que si un servicio o lo que sea falla, mandar un toast a la web
        self.tts_client = self.create_client(TTS, 'speech_tools/tts')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTS service not available, waiting again...')

        self.get_logger().info("Sancho Web Node initializated succesfully")

    def web_callback(self, msg):
        self.web_queue.put([msg.key, msg.value])

    def faceprint_event_callback(self, msg):
        if msg.origin != FaceprintEvent.ORIGIN_WEB: # Si el origen del evento es la web, no mandar a la web
            event = self.event_map.get(msg.event)
            message = FaceprintEventMessage(event, msg.id)
            self.msg_queue.put(message)

class SanchoWeb:
    
    def __init__(self):
        self.node = SanchoWebNode()

        self.audio_buffers = {}

    def spin(self):
        while True:
            if self.node.web_queue.qsize() > 0: # Web messages received
                [key, value] = self.node.web_queue.get()

                self.on_web_message(key, value)

            if self.node.msg_queue.qsize() > 0: # ROS messages to send
                message: JSONMessage = self.node.msg_queue.get()

                message_json = message.to_json()
                self.node.ros_pub.publish(R2WMessage(value=message_json)) # no key means broadcast

            rclpy.spin_once(self.node)

    def on_web_message(self, key, msg) -> JSONMessage:
        type, data = parse_message(msg)
        self.node.get_logger().info(f"Mensaje recibido: {type}")

        if type == MessageType.PROMPT:
            prompt = PromptMessage(data)

            response = self.sancho_prompt_request(prompt.value)
  
            self.send_message(key, ResponseMessage(prompt.id, response))
        elif type == MessageType.AUDIO_PROMPT_CHUNK:
            audio_prompt = AudioPromptChunkMessage(data)

            if not audio_prompt.final: # If not final
                if key not in self.audio_buffers:
                    self.audio_buffers[key] = []

                self.audio_buffers[key] += audio_prompt.audio
            else: # If final
                final_audio = self.audio_buffers.get(key, []) + audio_prompt.audio # El get por first = final
                if key in self.audio_buffers:
                    del self.audio_buffers[key]
                
                transcription = self.stt_request(final_audio, audio_prompt.sample_rate)
                self.send_message(key, PromptTranscriptionMessage(audio_prompt.id, transcription))

                response = self.sancho_prompt_request(transcription)
                self.send_message(key, ResponseMessage(audio_prompt.id, response))

                chunk_size = 48000
                audio, sample_rate = self.tts_request(response)
                for i in range(0, len(audio), chunk_size):
                    chunk = audio[i:i + chunk_size]
                    final = (i + chunk_size) >= len(audio)
                    index = i // chunk_size

                    self.send_message(key, AudioResponseChunkMessage(audio_prompt.id, index, final, chunk, sample_rate))
                    
    def sancho_prompt_request(self, text):
        sancho_prompt_request = SanchoPrompt.Request()
        sancho_prompt_request.text = text

        future_sancho_prompt = self.node.sancho_prompt_client.call_async(sancho_prompt_request)
        rclpy.spin_until_future_complete(self.node, future_sancho_prompt)
        result_sancho_prompt = future_sancho_prompt.result()

        return result_sancho_prompt.text

    def stt_request(self, audio, sample_rate):
        stt_request = STT.Request()
        stt_request.audio = audio
        stt_request.sample_rate = sample_rate

        future_stt = self.node.stt_client.call_async(stt_request)
        rclpy.spin_until_future_complete(self.node, future_stt)
        result_stt = future_stt.result()

        return result_stt.text

    def tts_request(self, text):
        tts_request = TTS.Request()
        tts_request.text = text

        future_tts = self.node.tts_client.call_async(tts_request)
        rclpy.spin_until_future_complete(self.node, future_tts)
        result_tts = future_tts.result()

        return result_tts.audio, result_tts.sample_rate

    def send_message(self, key, msg: JSONMessage):
        self.node.ros_pub.publish(R2WMessage(key=key, value=msg.to_json()))

def main(args=None):
    rclpy.init(args=args)

    sancho_web = SanchoWeb()

    sancho_web.spin()
    rclpy.shutdown()