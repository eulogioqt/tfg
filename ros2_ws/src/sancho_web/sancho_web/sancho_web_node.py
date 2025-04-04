import rclpy
from rclpy.node import Node

from queue import Queue

from .protocol import MessageType, PromptMessage, ResponseMessage, parse_message

from ros2web_msgs.msg import R2WMessage
from hri_msgs.srv import SanchoPrompt

class SanchoWebNode(Node):

    def __init__(self):
        super().__init__("sancho_web")

        self.web_queue = Queue()

        self.ros_pub = self.create_publisher(R2WMessage, "ros2web/ros", 10) # All publish here go to web
        self.web_sub = self.create_subscription(R2WMessage, "ros2web/web", self.web_callback, 10) # All received here comes from web

        self.sancho_prompt_client = self.create_client(SanchoPrompt, "sancho_ai/prompt")
        while not self.sancho_prompt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning("Sancho Prompt Service not available, waiting...")

        self.get_logger().info("Sancho Web Node initializated succesfully")

    def web_callback(self, msg):
        self.web_queue.put([msg.key, msg.value])

class SanchoWeb:
    
    def __init__(self):
        self.node = SanchoWebNode()

    def spin(self):
        while True:
            if self.node.web_queue.qsize() > 0: # ROS messages
                [key, value] = self.node.web_queue.get()

                response_json = self.on_protocol_message(value)
                self.node.ros_pub.publish(R2WMessage(key=key, value=response_json))

            rclpy.spin_once(self.node)

    def on_protocol_message(self, msg):
        type, data = parse_message(msg)
        print(f"Mensaje recibido: {type}")

        if type == MessageType.PROMPT:
            prompt = PromptMessage(data)

            id = prompt.id
            message = prompt.value
            resp = self.sancho_prompt_request(id, message)
            
            response = ResponseMessage(id, resp)
            return response.to_json()    

    def sancho_prompt_request(self, id, text):
        sancho_prompt_request = SanchoPrompt.Request()
        sancho_prompt_request.id = id
        sancho_prompt_request.text = text

        future_sancho_prompt = self.node.sancho_prompt_client.call_async(sancho_prompt_request)
        rclpy.spin_until_future_complete(self.node, future_sancho_prompt)
        result_sancho_prompt = future_sancho_prompt.result()

        return result_sancho_prompt.text

def main(args=None):
    rclpy.init(args=args)

    sancho_web = SanchoWeb()

    sancho_web.spin()
    rclpy.shutdown()