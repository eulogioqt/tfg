import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from websocket_thread_mixer import WebSocketThreadMixer

class ServerNode(Node):

    def __init__(self):
        super().__init__("server")

        self.subscriber = self.create_subscription(Image, "camera/color/image_raw", self.camera_callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Server Node initializated succesfully")

    def camera_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg)

        cv2.imshow("Camera", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    server = ServerNode()

    rclpy.spin(server)
    rclpy.shutdown()