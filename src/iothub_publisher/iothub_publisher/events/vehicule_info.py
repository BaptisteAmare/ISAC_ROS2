
class VehiculeInfo:
    def __init__(self, heading: float, speed: float, coordinates: dict):
        self.heading = heading
        self.speed = speed
        self.position = coordinates
        

    def to_dict(self):
        return {
            "Heading": self.heading,
            "Speed": self.speed,
            "Position": self.coordinates,
        }
