import json
import rclpy
from rclpy.node import Node

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
# hacer un service para subscribirse al topic que se le pase. Un mensaje R2WSubscription con topic y nombre

class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.ros_queue = Queue()
        self.broadcast_topics = {}

        self.ros_sub = self.create_subscription(R2WMessage, "ros2web/ros", self.server_callback, 1)
        self.web_pub = self.create_publisher(R2WMessage, "ros2web/web", 1)

        self.r2w_bridge = R2WBridge()
        self.get_logger().info("Server Node initializated succesfully")

    def subscribe_to_topic(self, topic_name, name=None):
        name = name if name else topic_name
        topic_name = topic_name if topic_name.startswith("/") else "/" + topic_name
        topic_types = dict(self.get_topic_names_and_types())

        if topic_name not in topic_types:
            self.get_logger().warn(f"Topic {topic_name} not found")
            return
        
        topic_type = topic_types[topic_name][0]
        self.get_logger().info(f"Topic '{topic_name}' is of type '{topic_type}'")

        data = topic_type.split('/')
        msg_module_name = ".".join(data[:-1])
        msg_class_name = data[-1]

        try:
            msg_module = __import__(msg_module_name, fromlist=[msg_class_name])
            msg_class = getattr(msg_module, msg_class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"No se pudo importar el tipo {topic_type}: {e}")
            return

        callback = lambda msg, tn=topic_name, nm=name: self.generic_callback(tn, nm, msg)
        self.create_subscription(msg_class, topic_name, callback, 1)

    def generic_callback(self, topic, name, value):
        self.broadcast_topics[topic] = [name, value]

    def server_callback(self, msg):
        self.ros_queue.put([msg.key, msg.value])


class Server(StoppableNode):
    
    def __init__(self):
        self.node = ServerNode()
        self.run_node = True

        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)
        self.node.subscribe_to_topic("/camera/color/image_raw", "IMAGE") # cambiar a service y que otro nodo sea el que active esto
        #add http server here and add to the ws th mixer

    def spin(self):
        if self.node.ros_queue.qsize() > 0: # ROS messages
            [key, value] = self.node.ros_queue.get()
            client = self.ws.get_client(key)

            message = Message(value)
            message_json = message.to_json()

            self.ws.send_message(client, message_json)

        for topic in self.node.broadcast_topics.keys(): # Broadcast topics
            data = self.node.broadcast_topics[topic]
            if data is not None:
                [name, value] = data
                value = self.node.r2w_bridge.any_to_r2w(value)

                message = TopicMessage(topic, name, value)
                message_json = message.to_json()
        
                self.node.broadcast_topics[topic] = None
                self.ws.broadcast_message(message_json)

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