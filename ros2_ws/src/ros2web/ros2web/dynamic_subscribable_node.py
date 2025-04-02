from rclpy.node import Node
from abc import ABC, abstractmethod
# por parameters
# por service
# que si no esta listo siga intentando como con lo de los service
# refactor
class DynamicSubscribableNode(Node, ABC):

    def __init__(self, node_name):
        super().__init__(node_name)
        self.dynamic_subscriptions = []

    def subscribe_to_topic(self, topic_name, name=None):
        name = name if name else topic_name
        topic_name = topic_name if topic_name.startswith("/") else "/" + topic_name
        topic_types = dict(self.get_topic_names_and_types())

        if topic_name not in topic_types:
            self.get_logger().warn(f"Topic {topic_name} not found")
            return
        
        topic_type = topic_types[topic_name][0]
        self.get_logger().info(f"Topic '{topic_name}' is of type '{topic_type}'")

        data = topic_type.split('/')
        msg_module_name = ".".join(data[:-1])
        msg_class_name = data[-1]

        try:
            msg_module = __import__(msg_module_name, fromlist=[msg_class_name])
            msg_class = getattr(msg_module, msg_class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"No se pudo importar el tipo {topic_type}: {e}")
            return

        callback = lambda msg, tn=topic_name, nm=name: self.generic_callback(tn, nm, msg)
        auto_subscription = self.create_subscription(msg_class, topic_name, callback, 1)
        self.dynamic_subscriptions.append(auto_subscription)

    @abstractmethod
    def generic_callback(self, topic, name, value):
        pass