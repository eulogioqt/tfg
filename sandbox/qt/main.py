import sys
import base64
from PyQt6.QtWidgets import QApplication
from controller.app_controller import AppController
from view.splash_screen import SplashScreen
from PyQt6.QtCore import QTimer

def encode_image_base64(path):
    with open(path, "rb") as img_file:
        return base64.b64encode(img_file.read()).decode("utf-8")

def main():
    app = QApplication(sys.argv)
    
    with open("style/app_style.qss", "r") as style_file:
        app.setStyleSheet(style_file.read())

    splash = SplashScreen()
    splash.show()

    controller = AppController(app, splash)

    img_b64 = encode_image_base64("resources/splash_logo.png")

    QTimer.singleShot(2000, lambda: controller.finish_splash())
    QTimer.singleShot(4000, lambda: controller.set_mode_show_photo(img_b64))
    QTimer.singleShot(11000, lambda: controller.set_mode_ask_name(img_b64))
    QTimer.singleShot(16000, lambda: controller.set_mode_ask_if_name(img_b64, "María"))

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
