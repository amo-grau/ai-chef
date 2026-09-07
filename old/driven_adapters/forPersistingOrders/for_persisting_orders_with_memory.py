from old.hexagon.driven_ports.for_persisting_orders import forPersistingOrders
from old.hexagon.order import Order


class forPersistingOrdersWithMemory(forPersistingOrders):
    def __init__(self) -> None:
        self._store: dict[str, Order] = {}

    def save(self, order: Order) -> None:
        self._store[order.id] = order

    def find_by_id(self, order_id: str) -> Order | None:
        return self._store.get(order_id)
