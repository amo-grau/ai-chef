from old.driven_adapters.forPersistingOrders.for_persisting_orders_with_memory import (
    forPersistingOrdersWithMemory,
)
from old.hexagon.driven_ports.for_persisting_orders import forPersistingOrders
from old.hexagon.order import Order, OrderStatus


def test_saved_order_can_be_retrieved_by_id() -> None:
    repository: forPersistingOrders = forPersistingOrdersWithMemory()
    order = Order("order-1", ["burger"])

    repository.save(order)

    assert repository.find_by_id("order-1") is order


def test_find_by_id_returns_none_for_unknown_id() -> None:
    repository: forPersistingOrders = forPersistingOrdersWithMemory()

    assert repository.find_by_id("unknown") is None


def test_saving_order_again_overwrites_previous_entry() -> None:
    repository: forPersistingOrders = forPersistingOrdersWithMemory()
    order = Order("order-1", ["burger"])
    repository.save(order)
    order.start()

    repository.save(order)

    retrieved = repository.find_by_id("order-1")
    assert retrieved is not None
    assert retrieved.status == OrderStatus.IN_PROGRESS
