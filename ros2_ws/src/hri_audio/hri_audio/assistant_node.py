import rclpy
from rclpy.node import Node

from queue import Queue

from std_msgs.msg import String
from hri_msgs.srv import SanchoPrompt


class AssistantNode(Node):

    def __init__(self):
        super().__init__("assistant")

        self.tts_pub = self.create_publisher(String, "hri_audio/tts", 10)
        self.text_sub = self.create_subscription(String, 'hri_audio/assistant_helper/transcription', self.text_callback, 10)

        self.sancho_prompt_client = self.create_client(SanchoPrompt, "sancho_ai/prompt")
        while not self.sancho_prompt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Sancho Prompt Service not available, waiting...")

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
                ai_response = self.sancho_prompt_request(user_text)
                
                self.node.tts_pub.publish(String(data=ai_response))

            rclpy.spin_once(self.node)

    def sancho_prompt_request(self, text):
        sancho_prompt_request = SanchoPrompt.Request()
        sancho_prompt_request.text = text

        future_sancho_prompt = self.node.sancho_prompt_client.call_async(sancho_prompt_request)
        rclpy.spin_until_future_complete(self.node, future_sancho_prompt)
        result_sancho_prompt = future_sancho_prompt.result()

        return result_sancho_prompt.text

def main(args=None):
    rclpy.init(args=args)

    assistant = Assistant()

    assistant.spin()
    rclpy.shutdown()