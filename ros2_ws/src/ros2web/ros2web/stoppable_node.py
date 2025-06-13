"""TODO: Add module documentation."""
import rclpy
from abc import ABC, abstractmethod


class StoppableNode(ABC):

"""TODO: Describe class."""
    def __init__(self):
    """TODO: Describe __init__.
"""
        self.run_node = True

    def stop(self):
    """TODO: Describe stop.
"""
        self.run_node = False

    def run(self):
    """TODO: Describe run.
"""
        try:
            while rclpy.ok() and self.run_node:
                self.spin()
                rclpy.spin_once(self.node)
        except Exception as e:
            print("ERROR ON SPIN:", e)
    
    @abstractmethod
    def spin(self):
    """TODO: Describe spin.
"""
        pass
