import numpy as np
import cv2
import StationMap
from Detector import Detector
import HandrailFilter
from CameraController import CameraController
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from roboviz import MapVisualizer


class RobotPosition:
    def __init__(self, plane, robot_pos, robot_dir):
        self.plane = plane
        self.pos = robot_pos
        self.dir = robot_dir


class MapManager:

    def __init__(self, station, robot_pos):
        self.station = station
        self.robot_pos = robot_pos

        # SLAM
        self.MAP_SIZE_PIXELS = 500
        self.MAP_SIZE_METERS = 10
        self.MIN_SAMPLES = 200

        self.lidar = Lidar('/dev/ttyUSB0')
        self.slam = RMHC_SLAM(LaserModel(), self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS)
        self.mapbytes = bytearray(self.MAP_SIZE_PIXELS * self.MAP_SIZE_PIXELS)
        self.iterator = self.lidar.iter_scans()
        self.previous_distances = None
        self.previous_angles = None
        next(self.iterator)

    def assign_id_and_conf_to_handrail_detection(self, robot_to_handrail_vector):
        x, y = self.get_handrail_coordinates(robot_to_handrail_vector)
        print(x, y)
        inv_dist_list = []
        for handrail in self.robot_pos.plane.handrails_on_plane:
            inv_dist_list.append(1 / self.get_distance(x, y, handrail.x, handrail.y))
        norm_confidence = [float(i)/sum(inv_dist_list) for i in inv_dist_list]
        max_confidence = max(norm_confidence)
        id = norm_confidence.index(max_confidence)
        handrail_id = self.robot_pos.plane.handrails_on_plane[id].handrail_id
        return handrail_id, max_confidence

    def get_handrail_coordinates(self, robot_to_handrail_vector):
        # add distanceA in the direction of the robot's unit vector
        # distanceA is equal to the distance measured by the HandrailLocator
        # add distanceB in the direction orthogonal to the robot's unit vector
        # distanceB is equal to the "x-offset" measured by the HandrailLocator
        # Then add the robot's current position to the result and return
        return 0, 0

    def get_distance(self, x1, y1, x2, y2):
        dist_squared = pow((x2 - x1), 2) + pow((y2 - y1), 2)
        return pow(dist_squared, 1/2)

    def update(self):
        # Extract (quality, angle, distance) triples from current scan
        items = [item for item in next(self.iterator)]

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles = [item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > self.MIN_SAMPLES:
            self.slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles = angles.copy()

        # If not adequate, use previous
        elif self.previous_distances is not None:
            self.slam.update(self.previous_distances, scan_angles_degrees=self.previous_angles)

        # Get current robot position
        x, y, theta = self.slam.getpos()

        self.robot_pos.pos = (x, y)
        # self.robot_pos.dir = theta  ####################################################### Convert theta to unit vector

        # Get current map bytes as grayscale
        self.slam.getmap(self.mapbytes)


def create_sample_map_manager():
    handrail1 = StationMap.Handrail(1, 0, (100, 100))
    handrail2 = StationMap.Handrail(2, 0, (-100, -100))
    # handrail3 = StationMap.Handrail(3, 0, (-100, 50))
    # handrail4 = StationMap.Handrail(4, 0, (-200, 100))
    floor_plane = StationMap.Plane(1, (300, 300), (-300, -300), (100, 300), [handrail1, handrail2])
    # leftw_plane = StationMap.Plane(2, (0, 300), (-300, 0), (-150, 100), [handrail3, handrail4])
    JEM_node = StationMap.Node("JEM", [floor_plane])
    ISS = StationMap.InternationalSpaceStation([JEM_node])

    robot_pos = RobotPosition(floor_plane, (0, 0), (0.707, 0.707))

    return MapManager(ISS, robot_pos)


def test_id_assignment_easy():
    map_manager = create_sample_map_manager()
    print(map_manager.assign_id_and_conf_to_handrail_detection((100, 50)))


def test_id_assignment_camera():
    handrail_filter = HandrailFilter.calibrate_from_package_return_handrail_filter()
    map_manager = create_sample_map_manager()

    cam = CameraController()
    test_img = cam.get_image()
    vectors = handrail_filter.get_handrail_vectors(test_img)
    for vect in vectors:
        print(map_manager.assign_id_and_conf_to_handrail_detection(vect))


def test_lidar():
    map_manager = create_sample_map_manager()
    while True:
        map_manager.update()


if __name__=="__main__":
    test_id_assignment_camera()