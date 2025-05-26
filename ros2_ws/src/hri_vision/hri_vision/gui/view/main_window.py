from PyQt6.QtWidgets import QStackedWidget
from .screens.ask_name_screen import AskNameScreen
from .screens.ask_if_name_screen import AskIfNameScreen
from .screens.photo_screen import PhotoScreen
from .screens.normal_screen import NormalScreen


class MainWindow(QStackedWidget):
    def __init__(self):
        super().__init__()
        self.ask_name_screen = AskNameScreen()
        self.ask_if_name_screen = AskIfNameScreen()
        self.photo_screen = PhotoScreen()
        self.normal_screen = NormalScreen()

        self.screens = {
            "ask_name": self.ask_name_screen,
            "ask_if_name": self.ask_if_name_screen,
            "photo": self.photo_screen,
            "normal": self.normal_screen
        }

        for screen in self.screens.values():
            self.addWidget(screen)

        self.set_screen("normal")

    def set_screen(self, screen_name, **kwargs):
        screen = self.screens.get(screen_name)
        if screen:
            screen.update_content(**kwargs)
            self.setCurrentWidget(screen)
