# File: rosbridge_suite/rosbridge_server/src/rosbridge_server/subscriber.py
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rosbridge_library.internal.message_conversion import extract_values
import importlib

class RosbridgeSubscriber:
    def __init__(self, node, topic, type, callback):
        self.node = node
        self.topic = topic
        self.type = type
        self.callback = callback
        #self.node.get_logger().info(f"Subscribing to {self.topic} [{self.type}]") 

        try:
            pkg, msg = self.type.split('/')
            msg_module = importlib.import_module(f'{pkg}.msg')
            self.msg_class = getattr(msg_module, msg)
        except (ValueError, ModuleNotFoundError, AttributeError) as e:
            self.node.get_logger().error(f"Could not import message type {type}: {e}")
            raise e

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sub = self.node.create_subscription(
            self.msg_class,
            self.topic,
            self._internal_callback,
            qos_profile
        )

    def unregister(self):
        self.node.destroy_subscription(self.sub)

    def _internal_callback(self, msg):
        try:
            msg_dict = extract_values(msg)
            self.callback(msg_dict)
        except Exception as e:
            self.node.get_logger().error(f"Error extracting message from {self.topic}: {e}")
            self.callback(None)  # fallback