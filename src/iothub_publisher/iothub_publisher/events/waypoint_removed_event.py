from datetime import datetime
from uuid import UUID

class WaypointRemovedEvent:
    def __init__(self, mission_id: UUID, waypoint_id: UUID, removed_at: datetime):
        self.mission_id = mission_id
        self.waypoint_id = waypoint_id
        self.removed_at = removed_at

    def to_dict(self):
        return {
            "MissionId": str(self.mission_id),
            "WaypointId": str(self.waypoint_id),
            "RemovedAt": self.removed_at.isoformat()
        }
