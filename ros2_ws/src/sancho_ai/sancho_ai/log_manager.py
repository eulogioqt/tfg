"""TODO: Add module documentation."""
from rclpy.node import Node
from typing import Optional

class LogManager:
"""TODO: Describe class."""
    _node: Optional[Node] = None

    @classmethod
    def set_node(cls, node: Node):
    """TODO: Describe set_node.
Args:
    cls (:obj:`Any`): TODO.
    node (:obj:`Any`): TODO.
"""
        cls._node = node

    @classmethod
    def info(cls, msg: str):
    """TODO: Describe info.
Args:
    cls (:obj:`Any`): TODO.
    msg (:obj:`Any`): TODO.
"""
        if cls._node:
            cls._node.get_logger().info(msg)

    @classmethod
    def warn(cls, msg: str):
    """TODO: Describe warn.
Args:
    cls (:obj:`Any`): TODO.
    msg (:obj:`Any`): TODO.
"""
        if cls._node:
            cls._node.get_logger().warn(msg)

    @classmethod
    def error(cls, msg: str):
    """TODO: Describe error.
Args:
    cls (:obj:`Any`): TODO.
    msg (:obj:`Any`): TODO.
"""
        if cls._node:
            cls._node.get_logger().error(msg)

    @classmethod
    def debug(cls, msg: str):
    """TODO: Describe debug.
Args:
    cls (:obj:`Any`): TODO.
    msg (:obj:`Any`): TODO.
"""
        if cls._node:
            cls._node.get_logger().debug(msg)
