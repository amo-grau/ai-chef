import pytest

from old.driving_adapters.forSubmittingOrders.cli import CliAdapter
from old.hexagon.driving_ports.for_submitting_orders import forSubmittingOrders


class FakeSubmitOrderUseCase(forSubmittingOrders):
    def __init__(self) -> None:
        self.received_items: list[str] = []

    def submit_order(self, items: list[str]) -> None:
        self.received_items = items


def make_cli(use_case: forSubmittingOrders, last_id: str = "test-id") -> CliAdapter:
    return CliAdapter(use_case, get_last_id=lambda: last_id)


def test_submit_prints_the_order_id(capsys: pytest.CaptureFixture[str]) -> None:
    cli = make_cli(FakeSubmitOrderUseCase(), last_id="abc-123")

    cli.run(["submit", "patty,bun,sauce"])

    assert capsys.readouterr().out.strip() == "abc-123"


def test_submit_passes_parsed_items_to_use_case() -> None:
    use_case = FakeSubmitOrderUseCase()
    cli = make_cli(use_case)

    cli.run(["submit", "patty,bun,sauce"])

    assert use_case.received_items == ["patty", "bun", "sauce"]


def test_submit_strips_whitespace_from_items() -> None:
    use_case = FakeSubmitOrderUseCase()
    cli = make_cli(use_case)

    cli.run(["submit", " patty , bun , sauce "])

    assert use_case.received_items == ["patty", "bun", "sauce"]


def test_unknown_command_exits_with_error() -> None:
    cli = make_cli(FakeSubmitOrderUseCase())

    with pytest.raises(SystemExit) as exc:
        cli.run(["unknown"])

    assert exc.value.code == 1


def test_missing_items_exits_with_error() -> None:
    cli = make_cli(FakeSubmitOrderUseCase())

    with pytest.raises(SystemExit) as exc:
        cli.run(["submit"])

    assert exc.value.code == 1
