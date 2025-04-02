import rclpy
from abc import ABC, abstractmethod


class StoppableNode(ABC): # No me gusta que stoppablenode y quien hereda de esto no es el nodo si no el wrapper

    def __init__(self):
        self.run_node = True

    def stop(self):
        self.run_node = False # probar a poner el run_node en el init de aqui a ver si vale

    def run(self):
        try:
            while rclpy.ok() and self.run_node:
                self.spin()
                rclpy.spin_once(self.node)
        except Exception as e:
            print("ERROR ON SPIN:", e)
    
    @abstractmethod
    def spin(self):
        pass