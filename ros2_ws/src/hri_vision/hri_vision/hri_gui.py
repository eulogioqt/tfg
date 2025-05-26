import os
import sys
import json
import rclpy
import base64

from rclpy.node import Node
from std_msgs.msg import Empty

from hri_msgs.msg import FaceNameResponse, FaceQuestionResponse
from hri_msgs.srv import TriggerUserInteraction

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

from .gui.controller.app_controller import AppController
from .gui.view.splash_screen import SplashScreen

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
        app = QApplication(sys.argv)

        style_path = os.path.join(os.path.dirname(__file__), "gui/style/app_style.qss")
        with open(style_path, "r") as style_file:
            app.setStyleSheet(style_file.read())

        splash = SplashScreen()
        splash.show()

        self.controller = AppController(app, splash)

        img_path = os.path.join(os.path.dirname(__file__), "gui/resources/splash_logo.png")
        img_b64 = encode_image_base64(img_path)

        QTimer.singleShot(2000, lambda: self.controller.finish_splash())
        QTimer.singleShot(4000, lambda: self.controller.set_mode_show_photo(img_b64))
        QTimer.singleShot(11000, lambda: self.controller.set_mode_ask_name(img_b64))
        QTimer.singleShot(16000, lambda: self.controller.set_mode_ask_if_name(img_b64, "María"))

        sys.exit(app.exec())

        self.node = HRIGUINode(self)

    def spin(self):
        while rclpy.ok():

            rclpy.spin_once(self.node)

    def user_interaction_service(self, request, response):
        mode = request.mode
        data = json.loads(request.data_json)

        return response


def main(args=None):
    rclpy.init(args=args)

    hri_gui = HRIGUI()
    hri_gui.spin()

    rclpy.shutdown()
