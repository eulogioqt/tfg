import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from queue import Queue

from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer
from .stoppable_node import StoppableNode
from .protocol import MessageType, Message, TopicMessage, parse_message
from .bridge import R2WBridge

from ros2web_msgs.msg import R2WMessage

# añadir lo de autosubscribirse a topics
# documentar todo y el protocolo aunque sea basico
# añadir algo para cuando entra y sale un usuario

class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.ros_queue = Queue()
        self.broadcast_data = {}

        self.img_sub = self.create_subscription(Image, "camera/color/image_raw", self.camera_callback, 1) # temp
        
        self.ros_sub = self.create_subscription(R2WMessage, "ros2web/ros", self.server_callback, 1)
        self.web_pub = self.create_publisher(R2WMessage, "ros2web/web", 1)

        self.r2w_bridge = R2WBridge()
        self.get_logger().info("Server Node initializated succesfully")

    def camera_callback(self, img):
        self.broadcast_data["IMAGE"] = img

    def server_callback(self, msg):
        self.ros_queue.put([msg.key, msg.value])


class Server(StoppableNode):
    
    def __init__(self):
        self.node = ServerNode()
        self.run_node = True

        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)
        #add http server here and to ws th mixer

    def spin(self):
        if self.node.ros_queue.qsize() > 0: # ROS messages
            [key, value] = self.node.ros_queue.get()
            client = self.ws.get_client(key)

            message = Message(value)
            message_json = message.to_json()

            self.ws.send_message(client, message_json)

        for type in self.node.broadcast_data.keys(): # Broadcast topics
            data = self.node.broadcast_data[type]
            if data is not None:
                data = self.node.r2w_bridge.any_to_r2w(data)
                
                message = TopicMessage(type, type, data)
                message_json = message.to_json()
        
                self.ws.broadcast_message(message_json)
                self.node.broadcast_data[type] = None

    def on_message(self, key, msg):
        type, data = parse_message(msg)
        print(f"Mensaje recibido ({key}): {type}")

        if type == MessageType.MESSAGE:
            value = json.dumps(data)
            self.node.web_pub.publish(R2WMessage(key=key, value=value))

    def on_user_connect(self, key):
        print(f"Cliente conectado ({key}) (Conexiones: {self.ws.get_connection_count()})")
    
    def on_user_disconnect(self, key):
        print(f"Cliente desconectado ({key}) (Conexiones: {self.ws.get_connection_count()})")
    

def main(args=None):
    rclpy.init(args=args)

    server = Server()
    websocket_thread_mixer = WebSocketThreadMixer(server.ws, server)
    websocket_thread_mixer.run()