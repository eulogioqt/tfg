import rclpy
from abc import ABC, abstractmethod


class StoppableNode(ABC):

    @abstractmethod
    def spin(self):
        pass

    def stop(self):
        self.run_node = False

    def run(self):
        try:
            while rclpy.ok() and self.run_node:
                self.spin()
                rclpy.spin_once(self.node)
        except Exception:
            pass