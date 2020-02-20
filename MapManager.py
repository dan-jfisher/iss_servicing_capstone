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

    def assign_id_to_handrail_detection(self, robot_to_handrail_vector):
        x, y = self.get_handrail_coordinates(robot_to_handrail_vector)
        dist_list = []
        for handrail in self.robot_pos.plane.handrails_on_plane:
            dist_list.append(self.get_distance(x, y, handrail.x, handrail.y))


    def get_handrail_coordinates(self, robot_to_handrail_vector):
        return 0, 0

    def get_distance(self, x1, y1, x2, y2):
        dist_squared = pow((x2 - x1), 2) + pow((y2 - y1), 2)
        return pow(dist_squared, 1/2)
