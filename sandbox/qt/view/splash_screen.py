from PyQt6.QtWidgets import QSplashScreen
from PyQt6.QtGui import QPixmap
from PyQt6.QtCore import Qt
import os

class SplashScreen(QSplashScreen):
    def __init__(self):
        # Cargamos la imagen desde el directorio de recursos
        pixmap = QPixmap(os.path.join("resources", "splash_logo.png"))
        super().__init__(pixmap)
        
        # Personalización adicional
        self.setWindowFlag(Qt.WindowType.FramelessWindowHint)
        self.setStyleSheet("color: white; font-size: 24px;")
        self.setFixedSize(800, 480)
        self.showMessage(
            "Cargando robot...",
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignCenter,
            Qt.GlobalColor.white
        )
