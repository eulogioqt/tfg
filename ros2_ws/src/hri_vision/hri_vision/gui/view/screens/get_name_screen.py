"""TODO: Add module documentation."""
import base64

from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QLineEdit, QPushButton, QMessageBox
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt


class GetNameScreen(QWidget):
"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.image_label = QLabel(alignment=Qt.AlignmentFlag.AlignCenter)
        self.title_label = QLabel("¿Cuál es tu nombre?", alignment=Qt.AlignmentFlag.AlignCenter)
        self.input_field = QLineEdit()
        self.send_button = QPushButton("Enviar")

        self.layout.addWidget(self.image_label)
        self.layout.addWidget(self.title_label)
        self.layout.addWidget(self.input_field)
        self.layout.addWidget(self.send_button)

    def update_content(self, photo_base64=None):
    """TODO: Describe update_content.
Args:
    photo_base64 (:obj:`Any`): TODO.
"""
        if photo_base64:
            pixmap = QPixmap()
            pixmap.loadFromData(base64.b64decode(photo_base64))
            self.image_label.setPixmap(pixmap.scaled(
                800, 600, Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            ))
        self.input_field.clear()

    def show_warning(self, message: str):
    """TODO: Describe show_warning.
Args:
    message (:obj:`Any`): TODO.
"""
        QMessageBox.warning(self, "Atención", message)
