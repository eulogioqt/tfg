from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout, QLineEdit, QPushButton
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt, QTimer

class MainView(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Robot Interface")
        self.setStyleSheet("background-color: black; color: white; font-size: 20px;")
        self.setFixedSize(800, 480)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.image_label = QLabel(alignment=Qt.AlignmentFlag.AlignCenter)
        self.question_label = QLabel("¿Cuál es tu nombre?", alignment=Qt.AlignmentFlag.AlignCenter)
        self.input_field = QLineEdit(placeholderText="Escribe tu nombre aquí...")
        self.button = QPushButton("Enviar")

        self.clear_view()

    def clear_view(self):
        while self.layout.count():
            child = self.layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()

    def show_normal_mode(self):
        self.clear_view()
        label = QLabel("Pantalla en espera", alignment=Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(label)

    def show_ask_name_mode(self, image_path: str):
        self.clear_view()
        pixmap = QPixmap(image_path)
        self.image_label.setPixmap(pixmap.scaled(400, 300, Qt.AspectRatioMode.KeepAspectRatio))
        self.input_field.clear()
        self.layout.addWidget(self.image_label)
        self.layout.addWidget(self.question_label)
        self.layout.addWidget(self.input_field)
        self.layout.addWidget(self.button)

    def show_photo_mode(self, image_path: str, callback_after_5s):
        self.clear_view()
        pixmap = QPixmap(image_path)
        self.image_label.setPixmap(pixmap.scaled(600, 400, Qt.AspectRatioMode.KeepAspectRatio))
        self.layout.addWidget(self.image_label)
        QTimer.singleShot(5000, callback_after_5s)
