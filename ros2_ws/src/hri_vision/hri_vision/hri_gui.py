import json
import rclpy
import base64
import threading

from rclpy.node import Node
from std_msgs.msg import Empty

from hri_msgs.msg import FaceNameResponse, FaceQuestionResponse
from hri_msgs.srv import TriggerUserInteraction

from .gui import AppController

def encode_image_base64(path):
    with open(path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode("utf-8")


class HRIGUINode(Node): # Hay que meter que se pueda decir el nombre con voz y un LLM lo extraiga y demas
    def __init__(self, hri_gui: 'HRIGUI'):
        super().__init__("hri_gui")

        self.hri_gui = hri_gui

        self.face_name_pub = self.create_publisher(FaceNameResponse, 'gui/face_name_response', 10)
        self.face_question_pub = self.create_publisher(FaceQuestionResponse, 'gui/face_question_response', 10)
        self.face_timeout_pub = self.create_publisher(Empty, 'gui/face_timeout_response', 10)

        self.trigger_interaction_srv = self.create_service(TriggerUserInteraction, 'gui/request', self.hri_gui.user_interaction_service)


class HRIGUI:
    def __init__(self):
        self.controller = AppController(self)
        self.node = HRIGUINode(self)

        self.ros_thread = threading.Thread(target=lambda: rclpy.spin(self.node), daemon=True)
        self.ros_thread.start()

    def spin(self):
        self.controller.start()

    def send_face_name_response(self, name):
        self.node.face_name_pub.publish(FaceNameResponse(name=name))

    def send_face_question_response(self, answer):
        self.node.face_question_pub.publish(FaceQuestionResponse(answer=answer))

    def send_face_timeout_response(self):
        self.node.face_timeout_pub.publish(Empty())

    def user_interaction_service(self, request, response):
        mode = request.mode
        data = json.loads(request.data_json)

        if mode == "get_name":
            image_base64 = data["image"]

            response.accepted = self.controller.set_mode_get_name(image_base64)
        elif mode == "ask_if_name":
            image_base64 = data["image"]
            name = data["name"]
            
            response.accepted = self.controller.set_mode_ask_if_name(image_base64, name)
        elif mode == "show_photo":
            image_base64 = data["image"]

            response.accepted = self.controller.set_mode_show_photo(image_base64)
        else:
            self.node.get_logger().info(f"Pantalla no reconocida: {mode}")
            response.accepted = False

        return response


def main(args=None):
    rclpy.init(args=args)

    hri_gui = HRIGUI()
    hri_gui.spin()

    rclpy.shutdown()
