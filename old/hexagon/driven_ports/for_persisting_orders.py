from abc import ABC, abstractmethod

from old.hexagon.order import Order


class forPersistingOrders(ABC):
    @abstractmethod
    def save(self, order: Order) -> None: ...

    @abstractmethod
    def find_by_id(self, order_id: str) -> Order | None: ...
