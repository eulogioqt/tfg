import rclpy
from rclpy.node import Node

from speech_msgs.srv import STT

from .stt.google_stt import GoogleSTT


class STTNode(Node):

    def __init__(self):
        super().__init__("stt")

        self.stt_service = self.create_service(STT, 'speech_tools/stt', self.handle_stt)
        self.g = GoogleSTT()
        self.get_logger().info('STT Node inicializado correctamente')
            
    def handle_stt(self, request, response):
        audio = request.audio
        sample_rate = request.sample_rate
        
        text = self.g.transcribe(audio, sample_rate)

        response.text = text
        
        return response

def main(args=None):
    rclpy.init(args=args)

    stt = STTNode()

    rclpy.spin(stt)
    rclpy.shutdown()