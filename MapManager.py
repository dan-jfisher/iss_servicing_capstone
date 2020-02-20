import numpy as np
import cv2
import StationMap

class RobotPosition:
    def __init__(self, plane, robot_pos, robot_dir):
        self.plane = plane
        self.robot_pos = robot_pos
        self.robot_dit = robot_dir


class MapManager:

    def __init__(self, station, robot_pos):
        self.station = station
        self.robot_pos = robot_pos

    def assign_id_and_conf_to_handrail_detection(self, robot_to_handrail_vector):
        x, y = self.get_handrail_coordinates(robot_to_handrail_vector)
        inv_dist_list = []
        for handrail in self.robot_pos.plane.handrails_on_plane:
            inv_dist_list.append(1 / self.get_distance(x, y, handrail.x, handrail.y))
        norm_confidence = [float(i)/sum(inv_dist_list) for i in inv_dist_list]
        max_confidence = max(norm_confidence)
        handrail_id = norm_confidence.index(max_confidence)
        return handrail_id, max_confidence

    def get_handrail_coordinates(self, robot_to_handrail_vector):
        return 0, 0

    def get_distance(self, x1, y1, x2, y2):
        dist_squared = pow((x2 - x1), 2) + pow((y2 - y1), 2)
        return pow(dist_squared, 1/2)


def create_sample_map_manager():
    handrail1 = StationMap.Handrail(1, 0, (5.3, 3.2))
    handrail2 = StationMap.Handrail(2, 0, (36.5, 100.5))
    # handrail3 = StationMap.Handrail(3, 0, (-100, 50))
    # handrail4 = StationMap.Handrail(4, 0, (-200, 100))
    floor_plane = StationMap.Plane(1, (300, 300), (0, 0), (100, 300), [handrail1, handrail2])
    # leftw_plane = StationMap.Plane(2, (0, 300), (-300, 0), (-150, 100), [handrail3, handrail4])
    JEM_node = StationMap.Node("JEM", [floor_plane])
    ISS = StationMap.InternationalSpaceStation([JEM_node])

    robot_pos = RobotPosition(floor_plane, (0, 0), (0, 0))

    return MapManager(ISS, robot_pos)


if __name__=="__main__":
    create_sample_map_manager()
