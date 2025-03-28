import json
import rclpy
from rclpy.node import Node
#from sensor_msgs.msg import Image

from queue import Queue
from cv_bridge import CvBridge

from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer
from .stoppable_node import StoppableNode

from ros2web.msg import R2WMessage

class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.ros_messages = [] # hacer con Queue

        self.ros_sub = self.create_subscription(R2W, "ros2web/ros", self.server_callback, 1)
        self.web_pub = self.create_publisher(R2W, "ros2web/web", 1)

        self.bridge = CvBridge()
        self.get_logger().info("Server Node initializated succesfully")

    def server_callback(self, msg):
        self.ros_messages.append([msg.key, msg.value])


class Server(StoppableNode):
    
    def __init__(self):
        self.node = ServerNode()
        self.run_node = True

        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)

    def spin(self):
        if len(self.node.ros_messages) > 0: # Ros messages
            [key, value] = self.node.ros_messages.pop()
            client = self.ws.get_client(key)

            message = {
                "type": "MESSAGE",
                "data": value
            }

            message_json = json.dumps(message)
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
        self.node.web_pub.publish(R2WMessage(key=key, value=msg))

    def on_user_connect(self, key):
        print(f"Cliente conectado ({key}) (Conexiones: {self.ws.get_connection_count()})")
    
    def on_user_disconnect(self, key):
        print(f"Cliente desconectado ({key}) (Conexiones: {self.ws.get_connection_count()})")
    

def main(args=None):
    rclpy.init(args=args)

    server = Server()
    websocket_thread_mixer = WebSocketThreadMixer(server.ws, server)
    websocket_thread_mixer.run()