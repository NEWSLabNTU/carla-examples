from carla import Actor, Image, VehicleControl, VehicleLightState
import numpy as np
from .student import StudentAgent
from ..ui import HUD
from ..utils import get_actor_display_name
import math
from collections import defaultdict
from ..state import State


class TaAgent:
    student_agent = None
    hud = None
    collision_history = list()
    state: State = None

    def __init__(self, state: State, hud: HUD):
        self.student_agent = StudentAgent()
        self.hud = hud
        self.state = state

    def get_collision_history(self):
        history = defaultdict(int)
        for frame, intensity in self.collision_history:
            history[frame] += intensity
        return history

    def step(self, actor: Actor) -> VehicleControl:
        return self.student_agent.step(actor)

    def on_lidar_data(self, points: np.ndarray):
        self.student_agent.on_lidar_data(points)

    def on_camera_data(self, image: np.ndarray):
        self.student_agent.on_camera_data(image)

    def on_collision(self, event):
        self.state.collision = True

        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification("Collision with %r" % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.collision_history.append((event.frame, intensity))
        if len(self.collision_history) > 4000:
            self.history.pop(0)

    def on_lane_invasion(self, event):
        self.state.lane_invasion_count += 1

        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ["%r" % str(x).split()[-1] for x in lane_types]
        self.hud.notification("Crossed line %s" % " and ".join(text))
