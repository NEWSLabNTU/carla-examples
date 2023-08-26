from carla import Actor, Image, VehicleControl, VehicleLightState, Location
import numpy as np
from .ui import HUD
from .utils import get_actor_display_name
import math
from collections import defaultdict
from .state import State
from agents.navigation.basic_agent import BasicAgent
from typing import List


class TaAgent:
    hud = None
    collision_history = list()
    state: State = None
    agent: BasicAgent = None

    def __init__(
        self, actor: Actor, state: State, hud: HUD, speed: float, points: List[Location]
    ):
        physics_control = actor.get_physics_control()
        physics_control.use_sweep_wheel_collision = True
        actor.apply_physics_control(physics_control)

        # MAGIC here. Must tick once to make sure the vehicle data is initialized.
        actor.get_world().tick()

        agent = BasicAgent(actor, speed)
        agent.set_destination(points[0].location)

        self.hud = hud
        self.state = state
        self.actor = actor
        self.agent = agent
        self.points = points
        self.next_index = 0

    def get_collision_history(self):
        history = defaultdict(int)
        for frame, intensity in self.collision_history:
            history[frame] += intensity
        return history

    def step(self) -> VehicleControl:
        if self.agent.done():
            self.next_index = (self.next_index + 1) % len(self.points)
            next_point = self.points[self.next_index].location
            self.agent.set_destination(next_point)

        control = self.agent.run_step()
        control.manual_gear_shift = False
        return control

    def on_lidar_data(self, points: np.ndarray):
        pass

    def on_camera_data(self, image: np.ndarray):
        pass

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
