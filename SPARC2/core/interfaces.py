"""Stable boundaries between algorithms, simulators and device adapters."""
from abc import ABC, abstractmethod
from dataclasses import dataclass

@dataclass(frozen=True)
class RobotState:
    positions: tuple[float, ...]
    timestamp: float

class ManipulationAlgorithm(ABC):
    @abstractmethod
    def plan(self, state: RobotState, target: dict) -> list[RobotState]:
        """Return a trajectory without executing hardware commands."""

class RobotAdapter(ABC):
    @abstractmethod
    def read_state(self) -> RobotState: ...
    @abstractmethod
    def execute(self, trajectory: list[RobotState]) -> None: ...
    @abstractmethod
    def stop(self) -> None: ...

class SimulationAdapter(RobotAdapter):
    """In-memory prototype adapter; never controls physical hardware."""
    def __init__(self):
        self.state = RobotState((), 0)
    def read_state(self):
        return self.state
    def execute(self, trajectory):
        if trajectory:
            self.state = trajectory[-1]
    def stop(self):
        pass
