import sys
from PyQt6.QtWidgets import QApplication
from controller.app_controller import AppController
from view.splash_screen import SplashScreen
from PyQt6.QtCore import QTimer

def main():
    app = QApplication(sys.argv)
    
    # Aplicar estilos desde archivo externo
    with open("style/app_style.qss", "r") as style_file:
        app.setStyleSheet(style_file.read())

    splash = SplashScreen()
    splash.show()

    controller = AppController(app, splash)

    # Simulación de flujo
    QTimer.singleShot(2000, lambda: controller.finish_splash())
    QTimer.singleShot(4000, lambda: controller.set_mode_show_photo("resources/splash_logo.png"))
    QTimer.singleShot(11000, lambda: controller.set_mode_ask_name("resources/splash_logo.png"))
    QTimer.singleShot(16000, lambda: controller.set_mode_ask_if_name("resources/splash_logo.png", "María"))

    sys.exit(app.exec())

if __name__ == "__main__":
    main()
