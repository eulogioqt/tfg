from PyQt6.QtWidgets import QApplication
from controller.app_controller import AppController
from view.splash_screen import SplashScreen
import sys

def main():
    app = QApplication(sys.argv)

    splash = SplashScreen()
    splash.show()

    controller = AppController(app, splash)
    controller.start()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
