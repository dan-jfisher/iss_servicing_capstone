import numpy as np
import cv2
import StationMap
from Detector import Detector
import HandrailFilter
from CameraController import CameraController

class RobotPosition:
    def __init__(self, plane, robot_pos, robot_dir):
        self.plane = plane
        self.pos = robot_pos
        self.dir = robot_dir


class MapManager:

    def __init__(self, station, robot_pos):
        self.station = station
        self.robot_pos = robot_pos

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
        handrail_x = self.robot_pos.dir[0] * robot_to_handrail_vector[0] + self.robot_pos.pos[0]
        handrail_y = self.robot_pos.dir[1] * robot_to_handrail_vector[1] + self.robot_pos.pos[1]
        return handrail_x, handrail_y

    def get_distance(self, x1, y1, x2, y2):
        dist_squared = pow((x2 - x1), 2) + pow((y2 - y1), 2)
        return pow(dist_squared, 1/2)


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
    detector = Detector()
    handrail_filter = HandrailFilter.calibrate_from_package_return_handrail_filter()
    map_manager = create_sample_map_manager()

    cam = CameraController()
    test_img = cam.get_image()
    detections = detector.get_rects_from_bgr(test_img)
    out = handrail_filter.get_valid_detections_and_distances_list(detections)
    for pair in out:
        detection, distance = pair
        vect = handrail_filter.get_vector_to_handrail(detection, distance)
        print(map_manager.assign_id_and_conf_to_handrail_detection(vect))
        # box = cv2.boxPoints(detection)
        # box = np.int0(box)
        # cv2.drawContours(test_img, [box], 0, (0, 0, 255), 2)
        # img = cv2.resize(test_img, (700, 500))
        # cv2.imshow("original", img)
        # cv2.waitKey(0)


if __name__=="__main__":
    test_id_assignment_camera()
