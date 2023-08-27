import math
from dataclasses import dataclass
import carla
import numpy as np
import pygame
import weakref
from ..ui import HUD
from ..utils import get_actor_bounding_extent
import cv2
from carla import (
    ColorConverter as CC,
    Transform,
    Location,
    Rotation,
    AttachmentType,
    Vector3D,
)
from typing import Optional, Dict, Tuple, Dict
from pathlib import Path
import os
from numpy.typing import ArrayLike
from threading import Lock

VIDEO_RESOLUTION = (1920, 1080)
FRAME_RATE = 10
OUTPUT_DIR = Path("_out")
IMG_DIR = OUTPUT_DIR / "images"
LOG_FILE = OUTPUT_DIR / "transform_log.csv"
CAMERA_CONFIGS = [
    (
        "front",
        "sensor.camera.rgb",
        CC.Raw,
        "Camera RGB",
        {"sensor_tick": str(1.0 / FRAME_RATE)},
    ),
    (
        "front-right",
        "sensor.camera.rgb",
        CC.Raw,
        "Camera RGB",
        {"sensor_tick": str(1.0 / FRAME_RATE)},
    ),
    (
        "rear",
        "sensor.camera.rgb",
        CC.Raw,
        "Camera RGB",
        {"sensor_tick": str(1.0 / FRAME_RATE)},
    ),
    (
        "rear-right",
        "sensor.camera.rgb",
        CC.Raw,
        "Camera RGB",
        {"sensor_tick": str(1.0 / FRAME_RATE)},
    ),
]


@dataclass
class DisplayPosition:
    left: int
    top: int
    width: int
    height: int


def sensor_callback(weak_me, weak_sensor, render_size, image):
    me = weak_me()
    if me is None:
        return

    sensor = weak_sensor()
    if sensor is None:
        return

    sensor.surface = me._parse_image(
        sensor.kind,
        sensor.cc,
        render_size,
        image,
    )

    if me.recording:
        me._record_image(sensor.name, sensor.cc, image)


class CameraManager:
    def __init__(
        self,
        parent_actor,
        hud: HUD,
        gamma_correction,
        record_on_start: bool,
        output_dir,
    ):
        # Create the output directory
        output_dir = Path(output_dir)
        image_dir = output_dir / "images"
        video_dir = output_dir / "videos"
        log_file = output_dir / "transform_log.csv"
        os.makedirs(output_dir, exist_ok=False)
        os.makedirs(image_dir, exist_ok=False)
        os.makedirs(video_dir, exist_ok=False)

        video_recorder = VideoRecorder(video_dir, FRAME_RATE)

        # Write CSV header to the log file
        if record_on_start:
            log_writer = open(log_file, "a")
            log_writer.write("frame,timestamp,x,y,z,pitch,yaw,roll\n")
            video_recorder.start()
        else:
            log_writer = None

        # Generate camera transformations
        if parent_actor.type_id.startswith("walker.pedestrian"):
            camera_transforms = generate_walker_transforms()
        else:
            bbox_extent = get_actor_bounding_extent(parent_actor)
            camera_transforms = generate_vehicle_transforms(bbox_extent)

        # Generate display position for each camera
        camera_display_positions = generate_camera_display_positions(
            hud.dim[0], hud.dim[1]
        )

        # Configure sensors
        world = parent_actor.get_world()
        bp_library = world.get_blueprint_library()
        lidar_range = None

        # Create sensor instances
        sensors = list()

        for args in zip(CAMERA_CONFIGS, camera_transforms, camera_display_positions):
            # Create the blueprint for camera
            (
                (name, kind, cc, desc, params),
                (transform, attachment_type),
                display_pos,
            ) = args
            bp = bp_library.find(kind)
            image_size_x, image_size_y = VIDEO_RESOLUTION
            bp.set_attribute("image_size_x", str(image_size_x))
            bp.set_attribute("image_size_y", str(image_size_y))

            if bp.has_attribute("gamma"):
                bp.set_attribute("gamma", str(gamma_correction))

            for attr_name, attr_value in params.items():
                bp.set_attribute(attr_name, attr_value)

            actor = parent_actor.get_world().spawn_actor(
                bp,
                transform,
                attach_to=parent_actor,
                attachment_type=attachment_type,
            )
            sensor = SensorDesc(
                name,
                kind,
                cc,
                desc,
                params,
                bp,
                actor,
                None,
                transform,
                attachment_type,
                display_pos,
            )
            sensors.append(sensor)

        # Assign fields
        self.sensor_index = 0
        self.sensors = sensors
        self._parent = parent_actor
        self.hud = hud
        self.recording = record_on_start
        self.lidar_range = lidar_range
        self.image_dir = image_dir
        self.log_writer = log_writer
        self._log_file = log_file
        self.video_recorder = video_recorder

        # Register sensor callbacks AFTER assigning class fields.
        for sensor, display_pos in zip(sensors, camera_display_positions):
            weak_self = weakref.ref(self)
            weak_sensor = weakref.ref(sensor)
            render_size = (display_pos.width, display_pos.height)
            sensor.actor.listen(
                lambda image, weak_self=weak_self, weak_sensor=weak_sensor, render_size=render_size: sensor_callback(
                    weak_self, weak_sensor, render_size, image
                )
            )

        self.set_sensor(0, notify=False)

    def __del__(self):
        del self.video_recorder

    def toggle_camera(self):
        # self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        # self.set_sensor(self.sensor_index, notify=False)
        pass

    def set_sensor(self, index: int, notify: bool = True):
        if notify:
            self.hud.notification(self.sensors[index].desc)
        self.sensor_index = index

    def next_sensor(self):
        self.set_sensor((self.sensor_index + 1) % len(self.sensors))

    def toggle_recording(self):
        if self.recording:
            self.video_recorder.stop()
        else:
            self.video_recorder.start()

        self.recording = not self.recording

        if self.recording and self.log_writer is None:
            self.log_writer = open(self._log_file, "a")
            self.log_writer.write("frame,timestamp,x,y,z,pitch,yaw,roll\n")

        self.hud.notification("Recording %s" % ("On" if self.recording else "Off"))

    def render(self, display):
        for sensor in self.sensors:
            if sensor.surface is not None:
                pos = (sensor.display_pos.left, sensor.display_pos.top)
                display.blit(sensor.surface, pos)

    def _record_image(self, name: str, cc: CC, image: carla.Image):
        frame_idx = image.frame
        image.convert(cc)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        # array = array[:, :, ::-1]
        self.video_recorder.push_image(name, frame_idx, array)

    def _parse_image(
        self,
        kind: str,
        cc: Optional[CC],
        render_size: Tuple[int, int],
        image: carla.Image,
    ) -> pygame.Surface:
        recording = self.recording
        lidar_range = self.lidar_range
        size_x, size_y = render_size
        min_size = min(size_x, size_y)

        if kind.startswith("sensor.lidar"):
            points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min_size / (2.0 * lidar_range)
            lidar_data += (0.5 * size_x, 0.5 * size_y)
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (size_x, size_y, 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            surface = pygame.surfarray.make_surface(lidar_img)

        elif kind.startswith("sensor.camera.dvs"):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(
                image.raw_data,
                dtype=np.dtype(
                    [
                        ("x", np.uint16),
                        ("y", np.uint16),
                        ("t", np.int64),
                        ("pol", np.bool),
                    ]
                ),
            )
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[
                dvs_events[:]["y"], dvs_events[:]["x"], dvs_events[:]["pol"] * 2
            ] = 255
            surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))

        elif kind.startswith("sensor.camera.optical_flow"):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        else:
            assert cc is not None
            image.convert(cc)
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        if recording:
            frame_idx = "%08d" % image.frame
            # capture = np.reshape(
            #     np.copy(image.raw_data), (image.height, image.width, 4)
            # )
            # cv2.imwrite(str(self.image_dir / f"{frame_idx}.png"), capture)
            #  transform = self._parent.get_transform()

            data_log = ",".join(
                map(
                    str,
                    [
                        frame_idx,
                        image.timestamp,
                        image.transform.location.x,
                        image.transform.location.y,
                        image.transform.location.z,
                        image.transform.rotation.pitch,
                        image.transform.rotation.yaw,
                        image.transform.rotation.roll,
                    ],
                )
            )

            self.log_writer.write(data_log)
            self.log_writer.write("\n")

        return surface


@dataclass
class SensorDesc:
    name: str
    kind: str
    cc: CC
    desc: str
    params: Dict[str, str]
    blueprint: carla.ActorBlueprint
    actor: carla.Actor
    surface: Optional[pygame.Surface]
    transform: Transform
    attachment_type: AttachmentType
    display_pos: DisplayPosition


def generate_vehicle_transforms(extent: Vector3D):
    # list of (x, y, z, yaw, type)
    params = [
        (0.8, 0.0, 1.3, 0.0, AttachmentType.Rigid),
        (0.8, 1.0, 1.3, 60.0, AttachmentType.Rigid),
        (-1.0, 0.0, 1.3, 180.0, AttachmentType.Rigid),
        (-1.0, 1.0, 1.3, 120.0, AttachmentType.Rigid),
    ]

    def build_transform(
        x_ratio: float, y_ratio: float, z_ratio: float, yaw: float, ty: AttachmentType
    ):
        x = extent.x * x_ratio
        y = extent.y * y_ratio
        z = extent.z * z_ratio
        transform = Transform(Location(x=x, y=y, z=z), Rotation(yaw=yaw))
        return (transform, ty)

    return list(map(lambda params: build_transform(*params), params))


def generate_camera_display_positions(display_w: int, display_h: int):
    upper_h_ratio = 0.6
    upper_h = int(math.floor(display_h * upper_h_ratio))
    lower_h = display_h - upper_h
    upper_w = display_w
    lower_l_w = int(math.floor(display_w / 3))
    lower_r_w = lower_l_w
    lower_m_w = display_w - lower_l_w - lower_r_w

    return [
        DisplayPosition(0, 0, upper_w, upper_h),
        DisplayPosition(0, upper_h, lower_l_w, lower_h),
        DisplayPosition(lower_l_w, upper_h, lower_m_w, lower_h),
        DisplayPosition(lower_l_w + lower_m_w, upper_h, lower_r_w, lower_h),
    ]


def generate_walker_transforms():
    return [
        (
            Transform(Location(x=-2.5, z=0.0), Rotation(pitch=-8.0)),
            AttachmentType.SpringArm,
        ),
        (Transform(Location(x=1.6, z=1.7)), AttachmentType.Rigid),
        (
            Transform(Location(x=2.5, y=0.5, z=0.0), Rotation(pitch=-8.0)),
            AttachmentType.SpringArm,
        ),
        (
            Transform(Location(x=-4.0, z=2.0), Rotation(pitch=6.0)),
            AttachmentType.SpringArm,
        ),
        (
            Transform(Location(x=0, y=-2.5, z=-0.0), Rotation(yaw=90.0)),
            AttachmentType.Rigid,
        ),
    ]


class VideoRecorder:
    session_sn: int
    log_dir: Path
    is_started: bool
    frame_rate: int
    sn: Dict[str, int]
    writers: Dict[str, cv2.VideoWriter]
    last_image: Dict[str, ArrayLike]
    lock: Lock

    def __init__(self, log_dir: Path, frame_rate: int):
        self.session_sn = 0
        self.log_dir = log_dir
        self.frame_rate = frame_rate
        self.writers = dict()
        self.sn = dict()
        self.last_image = dict()
        self.is_started = False
        self.lock = Lock()

    def __del__(self):
        for writer in self.writers.values():
            writer.release()

    def start(self):
        self.lock.acquire()
        assert not self.is_started
        self.session_sn += 1
        self.is_started = True
        self.lock.release()

    def stop(self):
        self.lock.acquire()
        assert self.is_started
        self.is_started = False

        for writer in self.writers.values():
            writer.release()

        self.writers = dict()
        self.sn = dict()
        self.last_image = dict()
        self.lock.release()

    def push_image(self, name: str, frame_idx: int, image: ArrayLike):
        self.lock.acquire()

        if not self.is_started:
            return

        if name not in self.writers:
            img_h, img_w, _ = image.shape
            writer = self.make_writer(name, (img_w, img_h))
            writer.write(image)

            self.writers[name] = writer
            self.sn[name] = frame_idx
            self.last_image[name] = image

        else:
            writer = self.writers[name]
            last_sn = self.sn[name]

            if frame_idx < last_sn:
                return
            elif frame_idx > last_sn:
                last_image = self.last_image[name]
                for _ in range(last_sn, frame_idx):
                    writer.write(last_image)

            writer.write(image)

            self.sn[name] = frame_idx
            self.last_image[name] = image

        self.lock.release()

    def make_writer(self, name: str, size: Tuple[int, int]):
        session_sn = self.session_sn
        video_path = self.log_dir / format(f"{name}-{session_sn}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(video_path), fourcc, self.frame_rate, size)
        return writer
