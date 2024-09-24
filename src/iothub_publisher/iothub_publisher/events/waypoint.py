from uuid import UUID

class Waypoint:
    def __init__(self, waypoint_id: UUID, coordinates: dict, heading: float, state: str):
        self.waypoint_id = waypoint_id
        self.coordinates = coordinates  # Dictionary with 'x', 'y', and possibly 'z' coordinates
        self.heading = heading
        self.state = state  # "Done", "Target", or "Waiting"

    def to_dict(self):
        return {
            "Id": str(self.waypoint_id),
            "Coordinates": self.coordinates,
            "Heading": self.heading,
            "State": self.state
        }
