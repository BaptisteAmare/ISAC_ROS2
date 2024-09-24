from datetime import datetime
from uuid import UUID

class MissionCompletedEvent:
    def __init__(self, mission_id: UUID, completed_at: datetime):
        self.mission_id = mission_id
        self.completed_at = completed_at

    def to_dict(self):
        return {
            "MissionId": str(self.mission_id),
            "CompletedAt": self.completed_at.isoformat()
        }
