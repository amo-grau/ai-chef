from abc import ABC, abstractmethod


class forSubmittingOrders(ABC):
    @abstractmethod
    def submit_order(self, items: list[str]) -> None: ...
