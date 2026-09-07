from enum import Enum, auto


class OrderStatus(Enum):
    PENDING = auto()
    IN_PROGRESS = auto()
    COMPLETE = auto()


class Order:
    def __init__(self, order_id: str, items: list[str]) -> None:
        if not items:
            raise ValueError("Order must have at least one item")
        self._id = order_id
        self._items = list(items)
        self._status = OrderStatus.PENDING

    @property
    def id(self) -> str:
        return self._id

    @property
    def items(self) -> list[str]:
        return list(self._items)

    @property
    def status(self) -> OrderStatus:
        return self._status

    def start(self) -> None:
        if self._status is not OrderStatus.PENDING:
            raise ValueError(f"Cannot start order in status {self._status.name}")
        self._status = OrderStatus.IN_PROGRESS

    def complete(self) -> None:
        if self._status is not OrderStatus.IN_PROGRESS:
            raise ValueError(f"Cannot complete order in status {self._status.name}")
        self._status = OrderStatus.COMPLETE
