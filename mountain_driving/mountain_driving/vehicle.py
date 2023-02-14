from carla import Transform, VehicleControl, VehicleLightState, World
from .sensor import (
    CollisionSensor,
    LaneInvasionSensor,
    GnssSensor,
    IMUSensor,
    CameraManager,
    LidarSensor,
    RgbCamera,
)
import sys
import random
from .utils import get_actor_blueprints, get_actor_display_name
from typing import Optional
from .ui import HUD
from .agent import TaAgent
import weakref


class Vehicle:
    actor = None
    hud = None
    show_vehicle_telemetry = False
    max_speed = None
    max_speed_fast = None
    collision_sensor = None
    lane_invasion_sensor = None
    gnss_sensor = None
    imu_sensor = None
    camera_manager = None
    enable_autopilot = True
    lidar_sensor = None
    collision_history = list()

    def __init__(
        self,
        role_name: str,
        world: World,
        hud: HUD,
        agent: TaAgent,
        gamma: Optional[float] = 2.2,
        spawn_point: Optional[Transform] = None,
        actor_filter: Optional[str] = "vehicle.tesla.model3",
        actor_generation: Optional[str] = "2",
    ):
        # Get a blueprint.
        blueprint = random.choice(
            get_actor_blueprints(world, actor_filter, actor_generation)
        )
        blueprint.set_attribute("role_name", role_name)
        if blueprint.has_attribute("color"):
            color = random.choice(blueprint.get_attribute("color").recommended_values)
            blueprint.set_attribute("color", color)
        if blueprint.has_attribute("driver_id"):
            driver_id = random.choice(
                blueprint.get_attribute("driver_id").recommended_values
            )
            blueprint.set_attribute("driver_id", driver_id)
        if blueprint.has_attribute("is_invincible"):
            blueprint.set_attribute("is_invincible", "true")

        # set the max speed
        if blueprint.has_attribute("speed"):
            max_speed = float(blueprint.get_attribute("speed").recommended_values[1])
            max_speed_fast = float(
                blueprint.get_attribute("speed").recommended_values[2]
            )
        else:
            max_speed = 1.589
            max_speed_fast = 3.713

        # Pick a spawn point
        if spawn_point is None:
            spawn_points = world.get_map().get_spawn_points()
            if not spawn_points:
                print("There are no spawn points available in your map/town.")
                print("Please add some Vehicle Spawn Point to your UE4 scene.")
                sys.exit(1)
            spawn_point = random.choice(spawn_points) if spawn_points else Transform()

        # Spawn actor
        actor = world.try_spawn_actor(blueprint, spawn_point)
        assert actor is not None, "unable to spawn actor"
        Vehicle.modify_vehicle_physics(actor)

        actor_type = get_actor_display_name(actor)
        hud.notification(actor_type)

        # Initialize sensors
        lidar_sensor = LidarSensor(actor)
        rgb_camera = RgbCamera(actor)

        # Initialize camera manager
        camera_manager = CameraManager(actor, hud, gamma)

        # Assign fields
        self.show_vehicle_telemetry = False
        self.max_speed = max_speed
        self.max_speed_fast = max_speed_fast
        self.collision_sensor = CollisionSensor(actor)
        self.lane_invasion_sensor = LaneInvasionSensor(actor)
        self.gnss_sensor = GnssSensor(actor)
        self.imu_sensor = IMUSensor(actor)
        self.lidar_sensor = lidar_sensor
        self.rgb_camera = rgb_camera
        self.camera_manager = camera_manager
        self.actor = actor
        self.agent = agent
        self.hud = hud

        # NOTE: Callbacks must be done after assiging class fields.
        weak_self = weakref.ref(self)
        self.lidar_sensor.set_callback(
            lambda points: Vehicle.on_lidar_data(weak_self, points)
        )
        self.rgb_camera.set_callback(
            lambda image: Vehicle.on_rgb_camera_data(weak_self, image)
        )
        self.collision_sensor.set_callback(
            lambda event: Vehicle.on_collision(weak_self, event)
        )
        self.lane_invasion_sensor.set_callback(
            lambda event: Vehicle.on_lane_invasion(weak_self, event)
        )

    def __del__(self):
        if self.actor is not None:
            self.actor.destroy()

        fields = [
            self.camera_manager,
            self.collision_sensor,
            self.lane_invasion_sensor,
            self.gnss_sensor,
            self.imu_sensor,
            self.lidar_sensor,
            self.rgb_camera,
        ]
        for field in fields:
            if field is not None:
                del field

    def set_autopilot(self, yes: bool):
        self.enable_autopilot = yes

    def toggle_autopilot(self):
        self.enable_autopilot = not self.enable_autopilot

    def tick(self):
        control = self.agent.step(self.actor)
        if self.enable_autopilot:
            self.actor.apply_control(control)

    def render(self, display):
        self.camera_manager.render(display)

    def get_collision_history(self):
        return self.agent.get_collision_history()

    @staticmethod
    def modify_vehicle_physics(actor):
        # If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    @staticmethod
    def on_lidar_data(weak_self, points):
        me = weak_self()
        if me is None:
            return

        me.agent.on_lidar_data(points)

    @staticmethod
    def on_rgb_camera_data(weak_self, image):
        me = weak_self()
        if me is None:
            return
        me.agent.on_camera_data(image)

    @staticmethod
    def on_collision(weak_self, event):
        me = weak_self()
        if me is None:
            return

        me.agent.on_collision(event)

    @staticmethod
    def on_lane_invasion(weak_self, event):
        me = weak_self()
        if me is None:
            return

        me.agent.on_lane_invasion(event)
