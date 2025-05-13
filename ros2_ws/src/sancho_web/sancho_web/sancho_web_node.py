import rclpy
from rclpy.node import Node

from queue import Queue

from .protocol import (
    MessageType, JSONMessage, 
    PromptMessage, AudioPromptMessage, TranscriptionRequestMessage,
    ResponseMessage, FaceprintEventMessage, PromptTranscriptionMessage, AudioResponseMessage,
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

            response, method, intent, provider, model = self.sancho_prompt_request(prompt.value)
            self.send_message(key, ResponseMessage(prompt.id, response, method, intent, provider, model))

            if prompt.want_tts:
                audio, sample_rate, model, speaker = self.tts_request(response)
                self.send_message(key, AudioResponseMessage(prompt.id, audio, sample_rate, model, speaker))
        elif type == MessageType.AUDIO_PROMPT:
            audio_prompt = AudioPromptMessage(data)
            
            transcription, model = self.stt_request(audio_prompt.audio, audio_prompt.sample_rate)
            self.send_message(key, PromptTranscriptionMessage(audio_prompt.id, transcription, model))

            response, method, intent, provider, model = self.sancho_prompt_request(transcription)
            self.send_message(key, ResponseMessage(audio_prompt.id, response, method, intent, provider, model))

            if audio_prompt.want_tts:
                audio, sample_rate, model, speaker = self.tts_request(response)
                self.send_message(key, AudioResponseMessage(audio_prompt.id, audio, sample_rate, model, speaker))
        elif type == MessageType.TRANSCRIPTION_REQUEST:
            transcription_request = TranscriptionRequestMessage(data)
            
            transcription, model = self.stt_request(transcription_request.audio, transcription_request.sample_rate)
            self.send_message(key, PromptTranscriptionMessage(transcription_request.id, transcription, model))   

    def sancho_prompt_request(self, text):
        req = SanchoPrompt.Request()
        req.text = text

        future = self.node.sancho_prompt_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        return result.text, result.method, result.intent, result.provider, result.model

    def stt_request(self, audio, sample_rate):
        req = STT.Request()
        req.audio = audio
        req.sample_rate = sample_rate

        future = self.node.stt_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        return result.text, result.model_used

    def tts_request(self, text):
        req = TTS.Request()
        req.text = text

        future = self.node.tts_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        return result.audio, result.sample_rate, result.model_used, result.speaker_used

    def send_message(self, key, msg: JSONMessage):
        self.node.ros_pub.publish(R2WMessage(key=key, value=msg.to_json()))

def main(args=None):
    rclpy.init(args=args)

    sancho_web = SanchoWeb()

    sancho_web.spin()
    rclpy.shutdown()