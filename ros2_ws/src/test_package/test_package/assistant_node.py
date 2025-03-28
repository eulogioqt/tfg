import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer
from .stoppable_node import StoppableNode
from .protocol import MessageType, DisplayDataType, DisplayDataMessage, PromptMessage, ResponseMessage, parse_message


class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.broadcast_data = {}

        self.subscriber = self.create_subscription(Image, "camera/color/image_raw", self.camera_callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Server Node initializated succesfully")

    def camera_callback(self, msg):
        self.broadcast_data[DisplayDataType.IMAGE] = self.bridge.imgmsg_to_cv2(msg)
    

class Server(StoppableNode):
    
    def __init__(self):
        self.node = ServerNode()
        self.run_node = True

        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)

    def spin(self):
        for type in self.node.broadcast_data.keys(): # Broadcast display data
            data = self.node.broadcast_data[type]
            if data is not None:
                message = DisplayDataMessage(data, type)
                message_json = message.to_json()

                self.ws.broadcast_message(message_json)
                self.node.broadcast_data[type] = None

    def on_message(self, msg):
        type, data = parse_message(msg)
        print(f"Mensaje recibido: {type}")

        if type == MessageType.PROMPT:
            prompt = PromptMessage(data)

            id = prompt.id
            value = prompt.value

            response = ResponseMessage(id, value[::-1])
            return response.to_json()

    def on_user_connect(self, client_ip, client_port):
        print(f"Cliente conectado ({client_ip}:{client_port}) (Conexiones: {self.ws.get_connection_count()})")
    
    def on_user_disconnect(self, client_ip, client_port):
        print(f"Cliente desconectado ({client_ip}:{client_port}) (Conexiones: {self.ws.get_connection_count()})")
    

def main(args=None):
    rclpy.init(args=args)

    server = Server()
    websocket_thread_mixer = WebSocketThreadMixer(server.ws, server)
    websocket_thread_mixer.run()