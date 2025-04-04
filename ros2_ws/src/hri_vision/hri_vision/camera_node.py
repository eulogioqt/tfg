import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

from ros2web_msgs.srv import R2WSubscribe

class Camera(Node):

    def __init__(self):
        super().__init__("camera")

        self.get_logger().info("Trying to start camera on index 0")
        self.camera = cv2.VideoCapture(0)

        self.publisher = self.create_publisher(Image, "camera/color/image_raw", 1)

        self.subscribe_client = self.create_client(R2WSubscribe, "ros2web/subscribe") # hacer una clase para que se herede con estas cosas repetidas
        if self.subscribe_client.wait_for_service(timeout_sec=1.0):
            success = self.subscribe_request("camera/color/image_raw", "IMAGE")
            self.get_logger().info(f"Success: {bool(success)}")

        self.bridge = CvBridge()

        self.spin()

    def subscribe_request(self, topic, name=""):
        subscribe_request = R2WSubscribe.Request()
        subscribe_request.topic = topic
        subscribe_request.name = name

        future_subscribe = self.subscribe_client.call_async(subscribe_request)
        rclpy.spin_until_future_complete(self, future_subscribe)
        result_subscribe = future_subscribe.result()

        return result_subscribe.value

    def spin(self):
        first_frame = True
        while True:
            ret, frame = self.camera.read()

            if not ret:
                break
            
            if first_frame:
                first_frame = False
                self.get_logger().info("Camera Node initializated succesfully")
            
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            cv2.waitKey(1)

        self.camera.release()

def main(args=None):
    rclpy.init(args=args)

    camera = Camera()

    rclpy.spin(camera)
    rclpy.shutdown()