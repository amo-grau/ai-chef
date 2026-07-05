from rclpy.node import Node
from std_msgs.msg import String

class PrepareOrderSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("kinematic_kitchen_scene")
        self._pending_orders = []

        self.create_subscription(
            String,
            "/kinematic_kitchen/prepare_order",
            self._on_prepare_order,
            10,
        )

    def has_next(self):
        return len(self._pending_orders) > 0
    
    def next(self):
        return self._pending_orders.pop(0)

    def _on_prepare_order(self, msg: String) -> None:
        self.get_logger().info(f"Preparing order: {msg.data}")
        self._pending_orders.append(msg.data)