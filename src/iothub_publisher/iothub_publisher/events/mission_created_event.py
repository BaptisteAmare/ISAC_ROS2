from datetime import datetime
from uuid import UUID
from events.waypoint import Waypoint

class MissionCreatedEvent:
    def __init__(self, mission_id: UUID, mission_name: str, created_at: datetime, first_waypoint: Waypoint):
        self.mission_id = mission_id
        self.mission_name = mission_name
        self.created_at = created_at
        self.first_waypoint = first_waypoint

    def to_dict(self):
        return {
            "MissionId": str(self.mission_id),
            "MissionName": self.mission_name,
            "CreatedAt": self.created_at.isoformat(),
            "FirstWaypoint": self.first_waypoint.to_dict()
        }
