from carla import Vector3D, CityObjectLabel
import time


SINCE = time.time()
BRAKE = False


def step(my_car):
    # control = my_car.get_control()

    ## TODO
    ## Apply control to the car. See the API here.
    ## https://carla.readthedocs.io/en/latest/python_api/#carla.VehicleControl
    # control.throttle = 1.0
    # control.brake = 1.0
    if not BRAKE:
        my_car.set_target_velocity(Vector3D(-(16 * 10 / 36), 0, 0))
    else:
        control = my_car.get_control()
        control.throttle = 0.0
        control.brake = 0.2
        my_car.apply_control(control)

    # my_car.apply_control(control)

    # if elapsed_s < 10.5:
    #     my_car.set_target_velocity(Vector3D(-(20 * 10 / 36 - 0.8), 0, 0))
    #     # my_car.set_target_velocity(Vector3D(-(0.2), 0, 0))
    # else:
    #     # my_car.set_target_velocity(Vector3D(0, 0, 0))
    #     control = my_car.get_control()
    #     control.throttle = 0.0
    #     control.brake = 0.12
    #     my_car.apply_control(control)


def on_sensor_data(event, my_car):
    global BRAKE
    global SINCE

    ## Find points casted on vehicles
    vehicle_tag = int(CityObjectLabel.Car)
    vehicle_points = filter(lambda det: det.object_tag == vehicle_tag, event)

    ## TODO
    ## Find the distance to the coach car using LiDAR points. You may check this.
    ## https://carla.readthedocs.io/en/latest/python_api/#carla.SemanticLidarDetection
    dists = map(lambda pt: pt.point.length(), vehicle_points)
    dist = max(dists)

    elapsed_s = time.time() - SINCE
    if dist < 15 and elapsed_s > 10:
        BRAKE = True
