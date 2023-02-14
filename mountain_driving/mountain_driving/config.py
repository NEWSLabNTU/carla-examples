from dataclasses import dataclass
from typing import List
from carla import Transform, Location, Rotation


@dataclass
class Checkpoint:
    locatoin: Location
    score: float

    def __init__(self, location: Location, score: float):
        self.location = location
        self.score = score


WORLD = "Town07"
INIT_TRANS = Transform(Location(72.31, -7.55, 1.0), Rotation(1.13, -62.79, 0.01))
TARGET_LOCATION = Location(15.71, -240.04, 0.18)
AGENT_STOP_THERSHOLD = 1e-3
CHECKPOINT_DISTANCE_THRESHOLD = 1.0
EXCEED_LAST_CHECKPOINT_PENALTY = 5.0
LANE_INVASION_PENALTY = 2.0
CHECKPOINTS: List[Checkpoint] = [
    Checkpoint(Location(60.64, -76.57, 5.51), 60.0),
    Checkpoint(Location(66.50, -110.84, 8.37), 70.0),
    Checkpoint(Location(44.43, -179.57, 5.76), 80.0),
    Checkpoint(Location(39.98, -216.25, 2.88), 90.0),
    Checkpoint(Location(15.71, -240.04, 0.18), 100.0),
]
