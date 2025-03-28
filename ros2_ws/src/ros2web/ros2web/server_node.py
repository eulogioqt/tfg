import json
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image

from queue import Queue
from cv_bridge import CvBridge

from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer
from .stoppable_node import StoppableNode
from .protocol import MessageType, Message, parse_message

from ros2web_msgs.msg import R2WMessage

class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.ros_queue = Queue()

        self.ros_sub = self.create_subscription(R2WMessage, "ros2web/ros", self.server_callback, 1)
        self.web_pub = self.create_publisher(R2WMessage, "ros2web/web", 1)

        self.bridge = CvBridge()
        self.get_logger().info("Server Node initializated succesfully")

    def server_callback(self, msg):
        self.ros_queue.put([msg.key, msg.value])


class Server(StoppableNode):
    
    def __init__(self):
        self.node = ServerNode()
        self.run_node = True

        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)

    def spin(self):
        if self.node.ros_queue.qsize() > 0: # ROS messages
            [key, value] = self.node.ros_queue.get()
            client = self.ws.get_client(key)

            message = Message(value)
            message_json = message.to_json()

            self.ws.send_message(client, message_json)

        #for type in self.node.broadcast_data.keys(): # Broadcast topics
        #    data = self.node.broadcast_data[type]
        #    if data is not None:
        #        message = DisplayDataMessage(data, type)
        #        message_json = message.to_json()
        #
        #        self.ws.broadcast_message(message_json)
        #        self.node.broadcast_data[type] = None

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