from dataclasses import dataclass

@dataclass
class State:
    finished: bool = False
    collision: bool = False
    exceed_last_checkpoint: bool = False
    checkpoint_index: int = 0
    lane_invasion_count: int = 0
