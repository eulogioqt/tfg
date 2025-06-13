"""TODO: Add module documentation."""
from PyQt6.QtWidgets import QStackedWidget
from .screens.get_name_screen import GetNameScreen
from .screens.ask_if_name_screen import AskIfNameScreen
from .screens.photo_screen import PhotoScreen
from .screens.normal_screen import NormalScreen


class MainWindow(QStackedWidget):
"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__()
        self.get_name_screen = GetNameScreen()
        self.ask_if_name_screen = AskIfNameScreen()
        self.photo_screen = PhotoScreen()
        self.normal_screen = NormalScreen()

        self.screens = {
            "get_name": self.get_name_screen,
            "ask_if_name": self.ask_if_name_screen,
            "photo": self.photo_screen,
            "normal": self.normal_screen
        }

        for screen in self.screens.values():
            self.addWidget(screen)

        self.set_screen("normal")

    def set_screen(self, screen_name, **kwargs):
    """TODO: Describe set_screen.
Args:
    screen_name (:obj:`Any`): TODO.
    **kwargs (:obj:`Any`): TODO.
"""
        screen = self.screens.get(screen_name)
        if screen:
            screen.update_content(**kwargs)
            self.setCurrentWidget(screen)
