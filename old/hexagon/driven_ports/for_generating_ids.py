from abc import ABC, abstractmethod


class forGeneratingIds(ABC):
    @abstractmethod
    def generate(self) -> str: ...
