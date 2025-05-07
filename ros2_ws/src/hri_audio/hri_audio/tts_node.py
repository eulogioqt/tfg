import rclpy
from rclpy.node import Node

from hri_msgs.srv import TTS

from.api.gtts import tts


class TTSNode(Node):
    
    def __init__(self):
        super().__init__("tts")
        
        self.tts_service = self.create_service(TTS, 'hri_audio/stt', self.handle_tts)

        self.get_logger().info('TTS Node inicializado correctamente')
    
    def handle_tts(self, request, response):
        text = request.text

        audio, sample_rate = tts(text)

        response.audio = audio
        response.sample_rate = sample_rate

        return response

def main(args=None):
    rclpy.init(args=args)
    
    tts = TTSNode()
    rclpy.spin(tts)
    
    rclpy.shutdown()