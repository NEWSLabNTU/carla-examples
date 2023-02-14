from carla import Actor, Image, VehicleControl, VehicleLightState
import numpy as np
import cv2 as cv
import colorsys


class StudentAgent:
    camera_image = None
    lidar_image = None

    def __init__(self):
        ## TODO
        pass

    def step(self, actor: Actor) -> VehicleControl:
        ## TODO
        actor.set_light_state(VehicleLightState.HighBeam)
        control = actor.get_control()

        ## To draw an image using OpenCV, please call imshow() in step().
        ## Do not imshow() in on_xxx_data(). It freezes the program!
        if self.camera_image is not None:
            cv.imshow("camera", self.camera_image)

        if self.lidar_image is not None:
            cv.imshow("lidar", self.lidar_image)

        cv.waitKey(1)

        return control

    def on_lidar_data(self, points: np.ndarray):
        ## TODO
        ## 'points' is an Nx4 array with x, y, z, intensity columns

        lidar_range = 50.0
        ih = 600
        iw = 800

        points = points[:, :2].copy()
        points *= min(ih, iw) / (2.0 * lidar_range)
        points += (0.5 * ih, 0.5 * iw)
        points = np.fabs(points)  # pylint: disable=E1111
        points = points.astype(np.int32)
        points = np.reshape(points, (-1, 2))
        image = np.zeros((ih, iw, 3), dtype=np.uint8)
        image[tuple(points.T)] = (255, 255, 255)

        self.lidar_image = image

    def on_camera_data(self, image: np.ndarray):
        ## TODO
        ## 'image' is an Nx3 array with r, g, b columns

        ## HSV thresholding
        orange_min = np.array([28, 10, 10], np.uint8)
        orange_max = np.array([30, 80, 100], np.uint8)
        image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        image = cv.inRange(image, orange_min, orange_max)

        ## erosion
        erosion_size = 1
        kernel = cv.getStructuringElement(
            cv.MORPH_ELLIPSE,
            (2 * erosion_size + 1, 2 * erosion_size + 1),
            (erosion_size, erosion_size),
        )
        image = cv.dilate(image, kernel, iterations=2)

        self.camera_image = image
