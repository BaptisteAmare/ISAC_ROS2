from datetime import datetime
from uuid import UUID
from events.waypoint import Waypoint

class WaypointAddedEvent:
    def __init__(self, mission_id: UUID, added_at: datetime, waypoint: Waypoint):
        self.mission_id = mission_id
        self.added_at = added_at
        self.waypoint = waypoint

    def to_dict(self):
        return {
            "MissionId": str(self.mission_id),
            "AddedAt": self.added_at.isoformat(),
            "WaypointInfo": self.waypoint.to_dict()
        }
