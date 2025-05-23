from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt, QTimer

class PhotoScreen(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.image_label = QLabel(alignment=Qt.AlignmentFlag.AlignCenter)
        self.layout.addWidget(self.image_label)

    def update_content(self, photo_path=None, callback=None):
        if photo_path:
            pixmap = QPixmap(photo_path)
            self.image_label.setPixmap(pixmap.scaled(600, 400, Qt.AspectRatioMode.KeepAspectRatio))
        if callback:
            QTimer.singleShot(5000, callback)
