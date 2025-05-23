from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QLineEdit, QPushButton, QMessageBox
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt

class AskNameScreen(QWidget):
    def __init__(self):
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

    def update_content(self, photo_path=None):
        if photo_path:
            pixmap = QPixmap(photo_path)
            self.image_label.setPixmap(pixmap.scaled(400, 300, Qt.AspectRatioMode.KeepAspectRatio))
        self.input_field.clear()

    def show_warning(self, message: str):
        QMessageBox.warning(self, "Atención", message)
