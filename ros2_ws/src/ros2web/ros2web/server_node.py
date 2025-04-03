import json
import rclpy
from rclpy.node import Node

from queue import Queue

from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer
from .stoppable_node import StoppableNode
from .dynamic_subscribable_node import DynamicSubscribableNode
from .protocol import MessageType, Message, TopicMessage, parse_message
from .r2w_bridge import R2WBridge

from ros2web_msgs.msg import R2WMessage

# añadir lo de autosubscribirse a topics
# documentar todo y el protocolo aunque sea basico
# añadir algo para cuando entra y sale un usuario
# hacer un service para subscribirse al topic que se le pase. Un mensaje R2WSubscription con topic y nombre (mensaje para subscribirse a topics)
# ponerlo tambien como argumentos de un launch
# no me mola que el stoppable node el que lo hereda es el wrapper del nodo no el node
# hacer AutosubscriptableNode para el node real y wrappear el service y todo eso ahi
# hacer que si el topic aun no esta disponible lo intenta cada cierto tiempo hasta que este

# coger todo el codigo y refactorizarlo y documentarlo hasta dejarlo perfecto cuando este todo hecho
# generar una documentacion para la memoria o algo asi de calidad, donde se describa bien tipo
# el websocketserver tiene esto esto y esto
# el httpserver esto y estas funciones "publicas" y asi con todo perfectisimo sabe o no sabe o si sabe

# hacer alguna forma para autorserializar o que el usuario al menos pueda (lo de bridge de ros_to_r2w)
# en esta conversacion pone que se puede "https://chatgpt.com/c/67ed892c-cf14-800f-b877-2e727bcfafa8"
# hacer que eso sea el por defecto y si no que el usuario pueda definir formas de serializar mensajes
# especificos, como las imagenes a jpg codificarlas y todo el rollo para que vaya mas rapido...

# hacer un paquete para el tema llms

# en el refactor hacer mas carpetas dentro de los paquetes aunque no sean paquetes

class ServerNode(DynamicSubscribableNode):

    def __init__(self):
        super().__init__("server")
    
        self.ros_queue = Queue()
        self.broadcast_topics = {}

        self.ros_sub = self.create_subscription(R2WMessage, "ros2web/ros", self.server_callback, 1)
        self.web_pub = self.create_publisher(R2WMessage, "ros2web/web", 1)

        self.r2w_bridge = R2WBridge()
        self.get_logger().info("Server Node initializated succesfully")

    def server_callback(self, msg):
        self.ros_queue.put([msg.key, msg.value])

    def generic_callback(self, topic, name, value):
        self.broadcast_topics[topic] = [name, value]

class Server(StoppableNode):
    
    def __init__(self):
        super().__init__()

        self.node = ServerNode()
        self.ws = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)
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