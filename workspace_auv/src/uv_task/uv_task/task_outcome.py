"""Small, ROS-independent task result type used by the mission runner."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class TaskOutcome:
    """The result of one task, including a stable failure code."""

    success: bool
    failure_code: str = ""
    message: str = ""
    transfer_override: dict | None = None

    @classmethod
    def ok(cls, message: str = "") -> "TaskOutcome":
        return cls(True, "", str(message))

    @classmethod
    def failed(cls, failure_code: str, message: str = "") -> "TaskOutcome":
        return cls(False, str(failure_code), str(message))

    def with_transfer(self, override: dict | None) -> "TaskOutcome":
        """Attach the already-selected one-hop override for the next task."""
        return TaskOutcome(
            self.success,
            self.failure_code,
            self.message,
            override,
        )

    def __bool__(self) -> bool:
        """Keep existing ``assert task.execute()`` tests source-compatible."""
        return self.success
