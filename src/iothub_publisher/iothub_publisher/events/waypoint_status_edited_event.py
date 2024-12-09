from datetime import datetime
from uuid import UUID

class Coordinates:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def to_dict(self):
        return {
            "X": self.x,
            "Y": self.y,
        }
    
class WaypointInfoReduced:
    def __init__(self, coordinates: Coordinates, index: int, waypoint_id: UUID, mission_id: UUID, status: str):
        self.coordinates = coordinates
        self.index = index
        self.waypoint_id = waypoint_id
        self.mission_id = mission_id
        self.status = status  # "Done", "Target", or "Waiting"

    def to_dict(self):
        return {
            "Coordinates": self.coordinates.to_dict(),
            "Index": self.index,
            "WaypointId": str(self.waypoint_id),
            "MissionId": str(self.mission_id),
            "Status": self.status
        }

    
class WaypointStatusEditedEvent:
    def __init__(self, mission_id: UUID, edited_at: datetime, waypoint_id: UUID, new_waypoint_info: WaypointInfoReduced):
        self.mission_id = mission_id
        self.edited_at = edited_at
        self.waypoint_id = waypoint_id
        self.new_waypoint_info = new_waypoint_info

    def to_dict(self):
        return {
            "MissionId": str(self.mission_id),
            "EditedAt": self.edited_at.isoformat(),
            "WaypointId": str(self.waypoint_id),
            "NewWaypointInfo": self.new_waypoint_info.to_dict()
        }
