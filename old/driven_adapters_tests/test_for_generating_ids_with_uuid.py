from old.driven_adapters.forGeneratingIds.for_generating_ids_with_uuid import (
    forGeneratingIdsWithUuid,
)
from old.hexagon.driven_ports.for_generating_ids import forGeneratingIds


def test_generates_a_non_empty_id() -> None:
    generator: forGeneratingIds = forGeneratingIdsWithUuid()

    assert generator.generate() != ""


def test_each_call_generates_a_unique_id() -> None:
    generator: forGeneratingIds = forGeneratingIdsWithUuid()

    assert generator.generate() != generator.generate()


def test_last_id_reflects_most_recent_generation() -> None:
    generator = forGeneratingIdsWithUuid()
    generator.generate()
    second = generator.generate()

    assert generator.last_id == second
