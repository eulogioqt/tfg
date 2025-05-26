import json
import rclpy
import sounddevice as sd
from rclpy.node import Node

from queue import Queue

from std_msgs.msg import String
from hri_msgs.srv import SanchoPrompt, TriggerUserInteraction
from speech_msgs.srv import TTS

from sancho_ai.sancho_ai.prompts.commands import COMMANDS


class AssistantNode(Node):

    def __init__(self):
        super().__init__("assistant")

        self.face_mode_pub = self.create_publisher(String, "face/mode", 10)
        self.text_sub = self.create_subscription(String, 'hri_audio/assistant_helper/transcription', self.text_callback, 10)

        self.sancho_prompt_client = self.create_client(SanchoPrompt, "sancho_ai/prompt")
        while not self.sancho_prompt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Sancho Prompt Service not available, waiting...")

        self.tts_client = self.create_client(TTS, 'speech_tools/tts')
        while not self.tts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('TTS service not available, waiting again...')

        self.gui_client = self.create_client(TriggerUserInteraction, 'gui/request')
        while not self.gui_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GUI service not available, waiting again...')
        

        self.queue = Queue(maxsize=1)
        self.get_logger().info("Assistant Node initializated succesfully.")

    def text_callback(self, msg):
        if self.queue.qsize() < 1:
            self.queue.put(msg.data)

class Assistant:

    def __init__(self):
        self.node = AssistantNode()
    
    def spin(self):
        while rclpy.ok():
            if not self.node.queue.empty():
                user_text = self.node.queue.get()

                self.node.face_mode_pub.publish(String(data="thinking"))
                ai_response, emotion, data, intent = self.sancho_prompt_request(user_text)
                self.node.get_logger().info(f"✅✅✅ Respuesta recibida '{ai_response}'")

                if intent == COMMANDS.TAKE_PICTURE:
                    self.gui_request("show_photo", data) # Show photo

                self.node.face_mode_pub.publish(String(data="speaking"))
                self.node.face_mode_pub.publish(String(data=emotion.lower()))
                audio, sample_rate = self.tts_request(ai_response)
                self.node.get_logger().info(f"✅✅✅ Reproduciendo por audio la respuesta")

                self.play_audio(audio, sample_rate)
                self.node.face_mode_pub.publish(String(data="idle"))

            rclpy.spin_once(self.node)

    def sancho_prompt_request(self, text):
        sancho_prompt_request = SanchoPrompt.Request()
        sancho_prompt_request.user = "HRI System"
        sancho_prompt_request.text = text

        future_sancho_prompt = self.node.sancho_prompt_client.call_async(sancho_prompt_request)
        rclpy.spin_until_future_complete(self.node, future_sancho_prompt)
        result_sancho_prompt = future_sancho_prompt.result()

        value = json.loads(result_sancho_prompt.value_json)

        text = value["text"]
        emotion = value["emotion"]
        data = value["data"]

        return text, emotion, data, result_sancho_prompt.intent

    def tts_request(self, text):
        tts_request = TTS.Request()
        tts_request.text = text

        future_tts = self.node.tts_client.call_async(tts_request)
        rclpy.spin_until_future_complete(self.node, future_tts)
        result_tts = future_tts.result()

        return result_tts.audio, result_tts.sample_rate

    def gui_request(self, mode, data_json):
        req = TriggerUserInteraction.Request()
        req.mode = mode
        req.data_json = data_json

        future = self.node.gui_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        result = future.result()

        return result.accepted

    def play_audio(self, audio, sample_rate, wait=True):
        sd.play(audio, samplerate=sample_rate)
        if wait:
            sd.wait()


def main(args=None):
    rclpy.init(args=args)

    assistant = Assistant()

    assistant.spin()
    rclpy.shutdown()