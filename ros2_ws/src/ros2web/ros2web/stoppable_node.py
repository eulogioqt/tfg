import rclpy
from abc import ABC, abstractmethod


class StoppableNode(ABC): # No me gusta que stoppablenode y quien hereda de esto no es el nodo si no el wrapper

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
        except Exception as e:
            print("ERROR ON SPIN:", e)