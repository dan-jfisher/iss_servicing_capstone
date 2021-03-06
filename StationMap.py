import numpy as np


class InternationalSpaceStation:
    def __init__(self, nodes):
        self.nodes = nodes


class Node:
    def __init__(self, name, planes):
        self.name = name
        self.planes = planes


class Plane:
    def __init__(self, id, tr_coord, bl_coord, charging_station_coord, handrails_on_plane):
        self.tr_coord = tr_coord
        self.bl_coord = bl_coord
        self.charging_station_coord = charging_station_coord
        self.handrails_on_plane = handrails_on_plane
        self.id = id


class Handrail:
    def __init__(self, id, clean, coord):
        self.handrail_id = id
        self.is_clean = clean
        self.x = coord[0]
        self.y = coord[1]


if __name__ == '__main__':
    handrail1 = Handrail(1, 0, (5.3, 3.2))
    handrail2 = Handrail(2, 0, (36.5, 100.5))
    handrail3 = Handrail(3, 0, (-100, 50))
    handrail4 = Handrail(4, 0, (-200, 100))
    floor_plane = Plane(1, (300, 300), (0, 0), (100, 300), [handrail1, handrail2])
    leftw_plane = Plane(2, (0, 300), (-300, 0), (-150, 100), [handrail3, handrail4])
    JEM_node = Node("JEM", [floor_plane, leftw_plane])
    ISS = InternationalSpaceStation([JEM_node])





