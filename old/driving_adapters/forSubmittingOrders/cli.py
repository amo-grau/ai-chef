import sys
from collections.abc import Callable

from old.hexagon.driving_ports.for_submitting_orders import forSubmittingOrders


class CliAdapter:
    def __init__(
        self,
        use_case: forSubmittingOrders,
        get_last_id: Callable[[], str],
    ) -> None:
        self._use_case = use_case
        self._get_last_id = get_last_id

    def run(self, args: list[str]) -> None:
        if len(args) < 2 or args[0] != "submit":
            usage = "Usage: python -m kinematic_kitchen submit <item1,item2,...>"
            print(usage, file=sys.stderr)
            sys.exit(1)

        items = [item.strip() for item in args[1].split(",")]
        self._use_case.submit_order(items)
        print(self._get_last_id())
