from uuid import uuid4

from old.hexagon.driven_ports.for_generating_ids import forGeneratingIds


class forGeneratingIdsWithUuid(forGeneratingIds):
    def __init__(self) -> None:
        self._last_id: str = ""

    def generate(self) -> str:
        self._last_id = str(uuid4())
        return self._last_id

    @property
    def last_id(self) -> str:
        return self._last_id
