from abc import ABC, abstractmethod

from old.hexagon.order import Order


class forPreparingOrders(ABC):
    @abstractmethod
    def prepare(self, order: Order) -> None: ...
