import rclpy
from rclpy.node import Node

from .api.text_to_speech_pyttsx3 import read_text_async


class PythonTTS(Node):
    
    def __init__(self):
        super().__init__("python_tts")
        
        self.create_subscription(str, "input_tts", self.text_to_speech, 10)
    
    def text_to_speech(self, msg):
        text = msg
        
        if text is not None:
            self.get_logger().info(">> SANCHO: " + str(text))
            read_text_async(text)
        else:
            self.get_logger().info("Mensaje None recibido. Ignorando...")

def main(args=None):
    rclpy.init(args=args)
    
    python_tts = PythonTTS()
    rclpy.spin(python_tts)
    
    rclpy.shutdown()