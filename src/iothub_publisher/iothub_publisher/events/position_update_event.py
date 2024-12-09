from datetime import datetime
from uuid import UUID
from iothub_publisher.events.vehicule_info import VehiculeInfo

class PositionUpdateEvent:
    def __init__(self, mission_id: UUID, date: datetime, vehiculeInfo: VehiculeInfo):
        self.missionId = mission_id
        self.date = date
        self.vehiculeInfo = VehiculeInfo
        

    def to_dict(self):
        return {
            "MissionId": str(self.mission_id),
            "Date": self.date,
            "VehiculeInfo": self.vehiculeInfo.to_dict()
        }
