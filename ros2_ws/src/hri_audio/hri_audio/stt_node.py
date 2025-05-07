import rclpy
from rclpy.node import Node

from hri_msgs.srv import AudioRecognition

from .api.gstt import transcribe


class STTNode(Node):

    def __init__(self):
        super().__init__("stt")

        self.stt_service = self.create_service(AudioRecognition, 'hri_audio/stt', self.audio_recognition)

        self.get_logger().info('STT Node inicializado correctamente')
            
    def audio_recognition(self, request, response):
        audio = request.audio
        sample_rate = request.sample_rate
        
        text = transcribe(audio, sample_rate)

        response.text = str(text)
        
        return response

def main(args=None):
    rclpy.init(args=args)

    stt = STTNode()

    rclpy.spin(stt)
    rclpy.shutdown()