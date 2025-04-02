import ast

from rclpy.node import Node
from abc import ABC, abstractmethod
from ros2web_msgs.srv import R2WSubscribe

# que si no esta listo siga intentando como con lo de los service
# refactor
class DynamicSubscribableNode(Node, ABC):

    def __init__(self, node_name):
        super().__init__(node_name)

        self.dynamic_subscriptions = []

        topics_str = self.declare_parameter('topics', "[]").get_parameter_value().string_value
        for [topic_name, name] in self.parse_topics(topics_str):
            self.subscribe_to_topic(topic_name, name)
        
        self.subscribe_service = self.create_service(R2WSubscribe, "ros2web/subscribe", self.subscribe)

    def subscribe_to_topic(self, topic_name, name=None):
        name = name if name else topic_name
        topic_name = topic_name if topic_name.startswith("/") else "/" + topic_name
        topic_types = dict(self.get_topic_names_and_types())

        actual_topics = [t.topic_name for t in self.dynamic_subscriptions]
        if topic_name in actual_topics:
            self.get_logger().warn(f"Already subscribed to {topic_name}")
            return 0
        
        if topic_name not in topic_types:
            self.get_logger().warn(f"Topic {topic_name} not found")
            return 0
        
        topic_type = topic_types[topic_name][0]
        data = topic_type.split('/')
        msg_module_name = ".".join(data[:-1])
        msg_class_name = data[-1]

        try:
            msg_module = __import__(msg_module_name, fromlist=[msg_class_name])
            msg_class = getattr(msg_module, msg_class_name)
        except (ImportError, AttributeError) as e:
            self.get_logger().error(f"Cannot import type {topic_type}: {e}")
            return 0

        callback = lambda msg, tn=topic_name, nm=name: self.generic_callback(tn, nm, msg)
        auto_subscription = self.create_subscription(msg_class, topic_name, callback, 1)
        self.dynamic_subscriptions.append(auto_subscription)
        self.get_logger().info(f"Succesfully subscribed to topic {topic_name}")

        return 1

    def subscribe(self, request, response):
        success = self.subscribe_to_topic(request.topic, request.name)
        response.value = success
        return response

    def parse_topics(self, raw_string):
        return ast.literal_eval(raw_string)

    @abstractmethod
    def generic_callback(self, topic, name, value):
        pass