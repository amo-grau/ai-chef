from old.hexagon.driven_ports.for_generating_ids import forGeneratingIds
from old.hexagon.driven_ports.for_persisting_orders import forPersistingOrders
from old.hexagon.driven_ports.for_preparing_orders import forPreparingOrders
from old.hexagon.driving_ports.for_submitting_orders import forSubmittingOrders
from old.hexagon.order import Order


class SubmitOrderUseCase(forSubmittingOrders):
    def __init__(
        self,
        repository: forPersistingOrders,
        id_generator: forGeneratingIds,
        preparation: forPreparingOrders,
    ) -> None:
        self._repository = repository
        self._id_generator = id_generator
        self._preparation = preparation

    def submit_order(self, items: list[str]) -> None:
        order = Order(self._id_generator.generate(), items)
        self._repository.save(order)
        self._preparation.prepare(order)
