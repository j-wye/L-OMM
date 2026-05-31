from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


@dataclass(frozen=True)
class FakeGoalRecord:
    goal_id: int
    command_summary: Dict[str, object]

    def as_dict(self) -> Dict[str, object]:
        return {"goal_id": int(self.goal_id), "command_summary": dict(self.command_summary)}


class FakeFollowJointTrajectoryServer:
    """In-memory action server substitute used by no-command tests."""

    def __init__(self) -> None:
        self.goals: List[object] = []
        self.cancel_count = 0
        self.active_goal_id: int | None = None

    @property
    def goal_count(self) -> int:
        return len(self.goals)

    def send_goal(self, command) -> FakeGoalRecord:
        self.goals.append(command)
        self.active_goal_id = len(self.goals)
        return FakeGoalRecord(goal_id=self.active_goal_id, command_summary=dict(getattr(command, "summary", {})))

    def cancel(self) -> bool:
        if self.active_goal_id is not None:
            self.cancel_count += 1
            self.active_goal_id = None
            return True
        return False

    def as_dict(self) -> Dict[str, object]:
        return {
            "goal_count": int(self.goal_count),
            "cancel_count": int(self.cancel_count),
            "active_goal_id": self.active_goal_id,
        }

