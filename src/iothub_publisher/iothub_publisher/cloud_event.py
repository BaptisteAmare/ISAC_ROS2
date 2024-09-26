import uuid
from datetime import datetime

class CloudEvent:
    def __init__(self, event_type: str, source: str, data: dict):
        self.specversion = "1.0"
        self.id = str(uuid.uuid4()) 
        self.source = source
        self.type = event_type
        self.time = datetime.now().isoformat() + "Z"  # ISO 8601 timestamp
        self.data = data

    def to_dict(self):
        return {
            "specversion": self.specversion,
            "id": self.id,
            "source": self.source,
            "type": self.type,
            "time": self.time,
            "data": self.data
        }
