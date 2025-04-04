import rclpy
from rclpy.node import Node

from queue import Queue
from cv_bridge import CvBridge

from .protocol import MessageType, PromptMessage, ResponseMessage, parse_message

from ros2web_msgs.msg import R2WMessage

class AINode(Node):

    def __init__(self):
        super().__init__("sancho_ai")

        self.web_queue = Queue()

        self.ros_pub = self.create_publisher(R2WMessage, "ros2web/ros", 1)
        self.web_sub = self.create_subscription(R2WMessage, "ros2web/web", self.web_callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Sancho AI Node initializated succesfully")

    def web_callback(self, msg):
        self.web_queue.put([msg.key, msg.value])

class AI:
    
    def __init__(self):
        self.node = AINode()

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
            resp = self.on_message(message)
            
            response = ResponseMessage(id, resp)
            return response.to_json()    

def main(args=None):
    rclpy.init(args=args)

    ai = AI()

    ai.spin()
    rclpy.shutdown()