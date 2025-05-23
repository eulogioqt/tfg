from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QHBoxLayout, QPushButton
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt

class AskIfNameScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.image_label = QLabel(alignment=Qt.AlignmentFlag.AlignCenter)
        self.question_label = QLabel(alignment=Qt.AlignmentFlag.AlignCenter)
        self.button_yes = QPushButton("Sí")
        self.button_no = QPushButton("No")
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.button_yes)
        button_layout.addWidget(self.button_no)

        self.layout.addWidget(self.image_label)
        self.layout.addWidget(self.question_label)
        self.layout.addLayout(button_layout)

    def update_content(self, photo_path=None, name=""):
        if photo_path:
            pixmap = QPixmap(photo_path)
            self.image_label.setPixmap(pixmap.scaled(400, 300, Qt.AspectRatioMode.KeepAspectRatio))
        self.question_label.setText(f"¿Eres {name}?")
