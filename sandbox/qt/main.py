from PyQt6.QtWidgets import QApplication
from controller.app_controller import AppController
from view.splash_screen import SplashScreen
from PyQt6.QtCore import QTimer
import sys

def main():
    app = QApplication(sys.argv)

    splash = SplashScreen()
    splash.show()

    controller = AppController(app, splash)
    controller.start()

    # A los 4 segundos, mostrar imagen con show_photo
    QTimer.singleShot(4000, lambda: controller.set_mode_show_photo("resources/splash_logo.png"))

    # A los 11 segundos, cambiar a modo ask_name con la misma imagen
    QTimer.singleShot(11000, lambda: controller.set_mode_ask_name("resources/splash_logo.png"))

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
