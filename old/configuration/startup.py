
from old.driven_adapters.forGeneratingIds.for_generating_ids_with_uuid import (
    forGeneratingIdsWithUuid,
)
from old.driven_adapters.forPersistingOrders.for_persisting_orders_with_memory import (
    forPersistingOrdersWithMemory,
)
from old.driven_adapters.forPreparingOrders.for_preparing_orders_with_isaac_sim import (
    forPreparingOrdersWithIsaacSim,
)
from old.driving_adapters.forSubmittingOrders.cli import CliAdapter
from old.hexagon.use_cases.submit_order import SubmitOrderUseCase


def run(args: list[str]) -> None:
    id_generator = forGeneratingIdsWithUuid()
    repository = forPersistingOrdersWithMemory()
    preparation = forPreparingOrdersWithIsaacSim()
    use_case = SubmitOrderUseCase(repository, id_generator, preparation)
    cli = CliAdapter(use_case, get_last_id=lambda: id_generator.last_id)
    cli.run(args)
