import pytest

from old.hexagon.order import Order, OrderStatus


def test_order_is_created_with_pending_status() -> None:
    order = Order("order-1", ["burger", "fries"])
    assert order.status == OrderStatus.PENDING


def test_order_holds_id_and_items() -> None:
    order = Order("order-1", ["burger", "fries"])
    assert order.id == "order-1"
    assert order.items == ["burger", "fries"]


def test_order_transitions_to_in_progress() -> None:
    order = Order("order-1", ["burger"])
    order.start()
    assert order.status == OrderStatus.IN_PROGRESS


def test_order_transitions_to_complete() -> None:
    order = Order("order-1", ["burger"])
    order.start()
    order.complete()
    assert order.status == OrderStatus.COMPLETE


def test_cannot_start_an_in_progress_order() -> None:
    order = Order("order-1", ["burger"])
    order.start()
    with pytest.raises(ValueError):
        order.start()


def test_cannot_start_a_completed_order() -> None:
    order = Order("order-1", ["burger"])
    order.start()
    order.complete()
    with pytest.raises(ValueError):
        order.start()


def test_cannot_complete_a_pending_order() -> None:
    order = Order("order-1", ["burger"])
    with pytest.raises(ValueError):
        order.complete()


def test_cannot_complete_an_already_completed_order() -> None:
    order = Order("order-1", ["burger"])
    order.start()
    order.complete()
    with pytest.raises(ValueError):
        order.complete()


def test_order_requires_at_least_one_item() -> None:
    with pytest.raises(ValueError):
        Order("order-1", [])
