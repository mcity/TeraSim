from terasim_datazoo.utils import xy_to_latlon


class OSMNode:
    def __init__(self, id: int, x: float, y: float, base_latitude: float = 40, base_longitude: float = -73):
        self.id: int = id
        self.x: float = x
        self.y: float = y
        (self.lat, self.lon) = xy_to_latlon(self.x, self.y, base_latitude, base_longitude)
        self.tags: dict = {}


class OSMWay:
    def __init__(self, id: int, node_refs: list[int]):
        self.id: int = id
        self.node_refs: list[int] = node_refs[:]
        self.tags: dict = {}
