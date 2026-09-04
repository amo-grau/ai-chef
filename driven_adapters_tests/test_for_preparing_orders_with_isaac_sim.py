import time

import pytest

# These are integration tests for the Isaac Sim adapter: they require a working
# ROS 2 environment (rclpy) and the colcon overlay that provides the project's
# custom messages. When either is missing — e.g. in CI or the Docker image —
# the whole module is skipped rather than failing.
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("kinematic_kitchen_interfaces")

from kinematic_kitchen_interfaces.msg import PrepareOrder
from rclpy.node import Node

from driven_adapters.forPreparingOrders.for_preparing_orders_with_isaac_sim import (
    forPreparingOrdersWithIsaacSim,
)
from hexagon.driven_ports.for_preparing_orders import forPreparingOrders
from hexagon.order import Order

TOPIC = "/kinematic_kitchen/prepare_order"


def test_prepare_does_not_raise() -> None:
    adapter: forPreparingOrders = forPreparingOrdersWithIsaacSim()
    order = Order("order-1", ["burger"])

    adapter.prepare(order)


def test_prepare_publishes_the_whole_order() -> None:
    received: list[PrepareOrder] = []
    listener = Node("test_listener")
    listener.create_subscription(PrepareOrder, TOPIC, received.append, 10)

    adapter: forPreparingOrders = forPreparingOrdersWithIsaacSim()
    order = Order("order-1", ["patty", "bun", "sauce"])

    # Let the subscriber discover the adapter's publisher before publishing,
    # then publish and spin briefly to receive the message.
    deadline = time.time() + 5.0
    while time.time() < deadline and not received:
        adapter.prepare(order)
        rclpy.spin_once(listener, timeout_sec=0.2)

    listener.destroy_node()

    assert received, "no message received before the timeout"
    assert received[0].id == "order-1"
    assert list(received[0].items) == ["patty", "bun", "sauce"]
