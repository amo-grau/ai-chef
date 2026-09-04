import time

import rclpy
from kinematic_kitchen_interfaces.msg import PrepareOrder
from rclpy.duration import Duration
from rclpy.node import Node

from hexagon.driven_ports.for_preparing_orders import forPreparingOrders
from hexagon.order import Order


class forPreparingOrdersWithIsaacSim(forPreparingOrders):
    # Seconds to wait for the scene's subscriber to be discovered before
    # publishing, and to wait for it to acknowledge receipt afterwards.
    _DISCOVERY_TIMEOUT = 5.0
    _DELIVERY_TIMEOUT = 2.0

    def __init__(self) -> None:
        if not rclpy.ok():
            rclpy.init()
        self._node = Node("kinematic_kitchen_publisher")
        self._publisher = self._node.create_publisher(
            PrepareOrder, "/kinematic_kitchen/prepare_order", 10
        )

    def prepare(self, order: Order) -> None:
        msg = PrepareOrder()
        msg.id = order.id
        msg.items = order.items

        # The CLI is a short-lived process: it publishes once and exits. Wait
        # for the Isaac Sim scene's subscriber to be discovered first, otherwise
        # the message is sent before the DDS discovery handshake completes and
        # is silently dropped.
        # get_subscription_count() is kept current by the DDS discovery thread,
        # so this resolves without spinning an executor (a QoS "matched" event
        # would only fire while spinning, which a one-shot CLI never does).
        deadline = time.time() + self._DISCOVERY_TIMEOUT
        while (
            self._publisher.get_subscription_count() == 0
            and time.time() < deadline
        ):
            time.sleep(0.02)

        self._publisher.publish(msg)

        # Block until the subscriber has acknowledged the sample (RELIABLE QoS)
        # rather than sleeping a fixed guess. Returns as soon as it is acked —
        # usually a few milliseconds — and guarantees delivery before the
        # process tears the publisher down.
        self._publisher.wait_for_all_acked(
            Duration(seconds=self._DELIVERY_TIMEOUT)
        )
