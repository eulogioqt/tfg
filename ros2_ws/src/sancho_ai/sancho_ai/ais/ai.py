import rclpy

from sancho_ai_node import SanchoAINode

from abc import ABC, abstractmethod

class AI(ABC):

    def __init__(self):
        self.node = SanchoAINode(self)

    @abstractmethod
    def on_message(self, message):
        pass

    def spin(self): # rclpy.spin(self.node)
        while True: 
            rclpy.spin_once(self.node)