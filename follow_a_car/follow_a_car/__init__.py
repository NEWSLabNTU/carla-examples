from carla import Client, Location, Rotation, Transform, Vector3D
import random
from argparse import ArgumentParser
from enum import Enum
import time
from . import controller
import math
import os


WORLD = "Town06"

SPEC_HEIGHT = 3
INIT_WAIT_SECS = 1.0
INIT_DISTANCE_M = 10.0
TIMEOUT_S = 30.0
TEACHER_DISTANCE = 100.0

SPECTATOR_TRANS = Transform(
    Location(608.5, -17.0, SPEC_HEIGHT), Rotation(0.0, 180.0, 0.0)
)
STUDENT_TRANS = Transform(Location(608.5, -17.0, 0.1), Rotation(0.0, 180.0, 0.0))
TEACHER_TRANS = Transform(
    Location(608.5 - INIT_DISTANCE_M, -17.0, 0.1), Rotation(0.0, 180.0, 0.0)
)

TEACHER_X_THRESH = 598.5 - TEACHER_DISTANCE
TEACHER_SPEED_KMPH = 20.0
STU_SPEED_THRESH_KMPH = 25.0
MIN_TTC_SECS = 1.0
LATERAL_OFFSET_M = 1.0
VELOCIDY_THRESH = 1e-3
LIDAR_RANGE_M = 30
ACC_THRESH_MS2 = 30


class State(Enum):
    FORWARDING = 1
    BRAKING = 2
    COLLISION = 3
    FINISH = 4


def stop_car(car):
    control = car.get_control()
    control.brake = 0.1
    car.apply_control(control)


def main():
    parser = ArgumentParser()
    parser.add_argument("--addr", default="localhost", help="CARLA server address")
    parser.add_argument("--port", default=2000, help="CARLA server port")
    parser.add_argument("--no-follow-car", action="store_true")
    args = parser.parse_args()

    teacher_speed_mps = TEACHER_SPEED_KMPH * 1000 / 3600
    stu_speed_thresh_mps = STU_SPEED_THRESH_KMPH * 1000 / 3600

    ## Connect to the client and retrieve the world object
    client = Client(args.addr, int(args.port))
    client.load_world(WORLD)
    world = client.get_world()

    ## Enable synchronous mode
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    ## Create state variable
    state = State.FORWARDING

    ## Configure spectator
    spec = world.get_spectator()
    spec.set_transform(SPECTATOR_TRANS)

    ## Spawn vehicles
    vblu = world.get_blueprint_library().find("vehicle.tesla.model3")
    stu_car = world.spawn_actor(vblu, STUDENT_TRANS)
    tea_car = world.spawn_actor(vblu, TEACHER_TRANS)

    ## TTC list used for judge
    ttc_list = list()

    def on_collision(event):
        nonlocal state
        if state == State.FINISH:
            pass
        elif state != State.COLLISION:
            state = State.COLLISION

    def on_success():
        nonlocal tea_car
        nonlocal stu_car
        nonlocal state
        nonlocal ttc_list

        stop_car(tea_car)
        stop_car(stu_car)

        ## Compute TTC score
        if ttc_list:
            mean_ttc = sum(ttc_list) / len(ttc_list)
        else:
            mean_ttc = 0.0

        if mean_ttc < 3.0:
            ttc_score = 0
        elif mean_ttc < 3.5:
            ttc_score = 90
        elif mean_ttc < 4.0:
            ttc_score = 100
        elif mean_ttc < 5.0:
            ttc_score = 80
        elif mean_ttc < 6.0:
            ttc_score = 70
        elif mean_ttc < 7.0:
            ttc_score = 60
        else:
            ttc_score = 0

        ## Compute distance score
        dist = stu_car.get_location().distance(tea_car.get_location())

        if dist < 5.0:
            dist_score = 0
        elif dist < 5.5:
            dist_score = 100
        elif dist < 6.0:
            dist_score = 90
        elif dist < 6.5:
            dist_score = 80
        elif dist < 7.0:
            dist_score = 70
        elif dist < 10.0:
            dist_score = 60
        else:
            dist_score = 0

        final_score = ttc_score * 0.5 + dist_score * 0.5
        print("mean_ttc={:.2f}s dist={:.2f}m".format(mean_ttc, dist))
        print(
            "SCORES: ttc_score={} dist_score={} final_score={}".format(
                ttc_score, dist_score, final_score
            )
        )

        state = State.FINISH

    def on_fail():
        nonlocal tea_car
        nonlocal stu_car
        nonlocal state
        stop_car(tea_car)
        stop_car(stu_car)
        state = State.FINISH

    def check():
        nonlocal tea_car
        nonlocal stu_car
        nonlocal ttc_list
        nonlocal since

        elapsed_s = time.time() - since
        veh = stu_car.get_velocity().length()
        dist = stu_car.get_location().distance(tea_car.get_location())
        acc = stu_car.get_acceleration().length()

        ## Compute TTC when velocity >= 1 m/s
        if veh >= 1.0:
            ttc = dist / veh
        else:
            ttc = float("nan")

        lateral_offset = abs(STUDENT_TRANS.location.y - stu_car.get_location().y)
        print(
            "t={:.2f}s v={:.2f}km/h ttc={:.2f}s lat_off={:.2f}m".format(
                elapsed_s, veh * 3.6, ttc, lateral_offset
            )
        )

        ## Save TTC
        if not math.isnan(ttc):
            ttc_list.append(ttc)

        if acc >= ACC_THRESH_MS2:
            print("FAIL: Acceleration too large (< {} m/s2)", ACC_THRESH_MS2)
            on_fail()

        if elapsed_s > TIMEOUT_S:
            print("FAIL: Exceed timeout {} s".format(TIMEOUT_S))
            on_fail()

        if veh > stu_speed_thresh_mps:
            print("FAIL: Speed exceeds {} km/s".format(STU_SPEED_THRESH_KMPH))
            on_fail()

        if ttc < MIN_TTC_SECS:
            print("FAIL: Too close to coach car (TTC < {} s)".format(MIN_TTC_SECS))
            on_fail()

        if lateral_offset > LATERAL_OFFSET_M:
            print("FAIL: Lateral movement too large (< {} m)".format(LATERAL_OFFSET_M))
            on_fail()

    ## Add a collision sensor on the student car
    cblu = world.get_blueprint_library().find("sensor.other.collision")
    col_sensor = world.spawn_actor(cblu, Transform(), attach_to=stu_car)
    col_sensor.listen(on_collision)

    ## Add a lidar on the student car
    lblu = world.get_blueprint_library().find("sensor.lidar.ray_cast_semantic")
    lblu.set_attribute("channels", "32")
    lblu.set_attribute("points_per_second", "600000")
    lblu.set_attribute("rotation_frequency", "10")
    lblu.set_attribute("range", str(LIDAR_RANGE_M))
    lidar_sensor = world.spawn_actor(lblu, Transform(), attach_to=stu_car)
    lidar_sensor.listen(lambda event: controller.on_sensor_data(event, stu_car))

    try:
        ## Skip 10 frames (~1s for 10fps)
        for _ in range(10):
            world.tick()

        ## Wait for the student car to start moving
        while True:
            controller.step(stu_car)
            world.tick()

            vel = stu_car.get_velocity().length()
            if vel >= VELOCIDY_THRESH:
                break

        ## Teachar car starts running
        tea_car.set_target_velocity(Vector3D(-teacher_speed_mps, 0, 0))

        ## start looping
        since = time.time()

        while True:
            controller.step(stu_car)
            world.tick()

            ## Stick the spectator to the student car
            if not args.no_follow_car:
                stu_trans = stu_car.get_transform()
                spec_loc = Location(
                    stu_trans.location.x, stu_trans.location.y, SPEC_HEIGHT
                )
                spec_rot = stu_trans.rotation
                spec_trans = Transform(spec_loc, spec_rot)
                spec.set_transform(spec_trans)

            ## Check violations
            if state == State.FORWARDING or state == State.BRAKING:
                check()

            ## Run state transition
            if state == State.FORWARDING:
                if tea_car.get_location().x < TEACHER_X_THRESH:
                    stop_car(tea_car)
                    state = State.BRAKING

            elif state == State.BRAKING:
                tea_vel = tea_car.get_velocity().length()
                stu_vel = stu_car.get_velocity().length()
                if tea_vel <= VELOCIDY_THRESH and stu_vel <= VELOCIDY_THRESH:
                    on_success()

            elif state == State.COLLISION:
                stop_car(tea_car)
                stop_car(stu_car)
                print("FAIL: Collision occurred")
                state = State.FINISH

            elif state == State.FINISH:
                break

            else:
                assert False

    except KeyboardInterrupt:
        print("INTERRUPTED")
        return

    try:
        ## loop forever
        while True:
            world.tick()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
