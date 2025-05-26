from PyQt6.QtCore import QTimer
from model.app_model import AppModel
from view.main_window import MainWindow

class AppController:
    def __init__(self, app, splash):
        self.app = app
        self.model = AppModel()
        self.view = MainWindow()
        self.splash = splash
        self.timeout_timer = QTimer()
        self.timeout_timer.setSingleShot(True)
        self.timeout_timer.timeout.connect(self.handle_timeout)

        self.view.ask_name_screen.send_button.clicked.connect(self.submit_name)
        self.view.ask_if_name_screen.button_yes.clicked.connect(lambda: self.submit_confirmation(True))
        self.view.ask_if_name_screen.button_no.clicked.connect(lambda: self.submit_confirmation(False))

    def finish_splash(self):
        self.splash.close()
        self.view.showMaximized()
        self.set_mode_normal()

    def is_priority_mode(self):
        return self.model.mode in ["ask_name", "ask_if_name"]

    def set_mode_normal(self):
        self.model.mode = "normal"
        self.view.set_screen("normal")

    def set_mode_ask_name(self, photo_base64):
        if self.is_priority_mode():
            print(f"Se ha intentado poner en modo ask name pero hay una pantalla prioritaria activa: {self.model.mode}")
            return
        self.model.mode = "ask_name"
        self.model.photo_base64 = photo_base64
        self.view.set_screen("ask_name", photo_base64=photo_base64)
        self.timeout_timer.start(20000)

    def set_mode_ask_if_name(self, photo_base64, name):
        if self.is_priority_mode():
            print(f"Se ha intentado poner en modo ask IF name pero hay una pantalla prioritaria activa: {self.model.mode}")
            return
        self.model.mode = "ask_if_name"
        self.model.photo_base64 = photo_base64
        self.model.ask_if_name_person = name
        self.view.set_screen("ask_if_name", photo_base64=photo_base64, name=name)
        self.timeout_timer.start(20000)

    def set_mode_show_photo(self, photo_base64):
        if self.is_priority_mode():
            print(f"Se ha intentado poner en modo show photo pero hay una pantalla prioritaria activa: {self.model.mode}")
            return
        self.model.mode = "show_photo"
        self.model.photo_base64 = photo_base64
        self.view.set_screen("photo", photo_base64=photo_base64, callback=self.set_mode_normal)

    def submit_name(self):
        if self.model.mode != "ask_name":
            return
        name = self.view.ask_name_screen.input_field.text().strip()
        if not name:
            self.view.ask_name_screen.show_warning("El nombre no puede estar vacío.")
            return
        if len(name) > 20:
            self.view.ask_name_screen.show_warning("El nombre es demasiado largo.")
            return
        self.timeout_timer.stop()
        print(f"[get_name] Nombre recibido: {name}")
        self.set_mode_normal()

    def submit_confirmation(self, response: bool):
        if self.model.mode != "ask_if_name":
            return
        self.timeout_timer.stop()
        print(f"[ask_if_name] Respuesta del usuario: {'Sí' if response else 'No'}")
        self.set_mode_normal()

    def handle_timeout(self):
        print(f"[timeout] No hubo respuesta en 20 segundos.")
        self.set_mode_normal()
