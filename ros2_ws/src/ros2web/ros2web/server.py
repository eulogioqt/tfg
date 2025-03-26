import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from .websocket_thread_mixer import WebSocketThreadMixer
from .websocket_server import WebSocketServer

class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.img = None

        self.subscriber = self.create_subscription(Image, "camera/color/image_raw", self.camera_callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Server Node initializated succesfully")

    def camera_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg)
    

class Server:
    
    def __init__(self):
        self.node = ServerNode()

        self.websocket_server = WebSocketServer(self.on_message, self.on_user_connect, self.on_user_disconnect)
        #self.http_server

        self.run_node = True

    def spin(self):
        pass

    def on_message(self, msg):
        print(f"Mensaje recibido: {msg}")
        response = None

        if msg == "STOP":
            self.websocket_server.stop_program()
        elif msg == "TEST":
            response = "Working fine"
        
        return response

    def on_user_connect(self, client_ip, client_port):
        print(f"Cliente conectado ({client_ip}:{client_port}) (Conexiones: {self.websocket_server.get_connection_count()})")
    
    def on_user_disconnect(self, client_ip, client_port):
        print(f"Cliente desconectado ({client_ip}:{client_port}) (Conexiones: {self.websocket_server.get_connection_count()})")

    def stop(self):
        self.run_node = False

    def run(self):
        try:
            while rclpy.ok() and self.run_node:
                self.spin()
                
                rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass
    

def main(args=None):
    rclpy.init(args=args)

    server = Server()
    websocket_thread_mixer = WebSocketThreadMixer(server.websocket_server, server)
    websocket_thread_mixer.run()