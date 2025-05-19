from PyQt6.QtCore import QTimer
from model.app_model import AppModel
from view.main_view import MainView

class AppController:
    def __init__(self, app, splash):
        self.app = app
        self.model = AppModel()
        self.view = MainView()
        self.splash = splash

        # Conectar acciones
        self.view.button.clicked.connect(self.submit_name)

    def start(self):
        QTimer.singleShot(2000, self.finish_splash)

    def finish_splash(self):
        self.splash.close()
        self.view.show()
        self.set_mode_normal()

    def set_mode_normal(self):
        self.model.mode = "normal"
        self.view.show_normal_mode()

    def set_mode_ask_name(self, photo_path): # hacerlo con la imagen de ros o cv2 o base64
        self.model.mode = "ask_name"
        self.model.photo_path = photo_path
        self.view.show_ask_name_mode(photo_path)

    def set_mode_show_photo(self, photo_path):
        self.model.mode = "show_photo"
        self.model.photo_path = photo_path
        self.view.show_photo_mode(photo_path, self.set_mode_normal)

    def submit_name(self):
        name = self.view.input_field.text()
        self.model.name_input = name
        print(f"Nombre recibido: {name}")
        self.set_mode_normal()
