from rclpy.node import Node
from kinematic_kitchen_interfaces.msg import PrepareOrder

QUEUE_DEPTH = 10

class RosMotionCommandSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("kinematic_kitchen_scene")
        self._pending_commands: list[PrepareOrder] = []

        self.create_subscription(
            PrepareOrder,
            "/kinematic_kitchen/prepare_order",
            self._on_prepare_order,
            QUEUE_DEPTH
        )
        self._active_order = None

    def has_active_order(self):
        return self._active_order != None

    def on_active_order_handeled(self):
        self._active_order = None
    
    def has_next(self) -> bool:
        return len(self._pending_commands) > 0

    def next(self):
        prepareOrder = self._pending_commands.pop(0)
        self.get_logger().info(
            f"Starting pick and place for order {prepareOrder.id}: {', '.join(prepareOrder.items)}"
        )

        self._active_order = prepareOrder

    def _on_prepare_order(self, msg: PrepareOrder) -> None:
        self.get_logger().info(f"Preparing order: {msg.id}, with {msg.items}")
        self._pending_commands.append(msg)
