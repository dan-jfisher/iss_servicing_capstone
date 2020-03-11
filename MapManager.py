import numpy as np
import cv2
import StationMap
from Detector import Detector
import HandrailLocator
from CameraController import CameraController
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar
from roboviz import MapVisualizer
import pybreezyslam


class SlamFactory:
    def build(self, laserModel, map_size_pix, map_size_m, initial_position=(-1, -1)):
        slam = RMHC_SLAM(LaserModel(), self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS)
        if not initial_position == (-1, -1):
            slam.position = pybreezyslam.Position(initial_position[0] * 10, initial_position[0] * 10, 0)
        return slam


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
        self.initial_pos_cm = (0, 0)
        self.slam = SlamFactory.build(LaserModel(), self.MAP_SIZE_PIXELS, self.MAP_SIZE_METERS, initial_pos_cm)
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
        ######################################################################################## update handrail location
        return handrail_id, max_confidence

    def get_handrail_coordinates(self, distance_in_dir, distance_orth_dir):
        # Find orthogonal vector based on position of handrail in picture
        if distance_orth_dir < 0:
            orthogonal_matrix = np.array([[0, -1], [1, 0]])
        else:
            orthogonal_matrix = np.array([[0, 1], [-1, 0]])
        orthogonal_vector = np.matmul(self.robot_pos.dir, orthogonal_matrix)

        # Calculate handrail location in global coordinates
        handrail_location = self.robot_pos.dir * distance_in_dir + orthogonal_vector * distance_orth_dir
        return handrail_location

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

        self.robot_pos.pos = (x / 10, y / 10)  # convert from mm to cm
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
    handrail_filter = HandrailLocator.calibrate_from_package_return_handrail_filter()
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