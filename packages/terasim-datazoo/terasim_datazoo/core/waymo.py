from enum import Enum
from terasim_datazoo.utils import Pt, TLS

class LaneType(Enum):
    UNDEFINED = 0
    FREEWAY = 1
    SURFACE_STREET = 2
    BIKELANE = 3

class AgentType(Enum):
    VEHICLE = 0
    PEDESTRIAN = 1
    CYCLIST = 2

class AgentRole(Enum):
    SDC = 0
    INTEREST = 1
    PREDICT = 2

class WaymonicTLS(Enum):
    ABSENT = -1
    UNKNOWN = 0
    ARROW_STOP = 1
    ARROW_CAUTION = 2
    ARROW_GO = 3
    STOP = 4
    CAUTION = 5
    GO = 6
    FLASHING_STOP = 7
    FLASHING_CAUTION = 8

    def generalize(self) -> TLS:
        mapping = {
            WaymonicTLS.ABSENT: TLS.ABSENT,
            WaymonicTLS.UNKNOWN: TLS.UNKNOWN,
            WaymonicTLS.ARROW_STOP: TLS.RED,
            WaymonicTLS.ARROW_CAUTION: TLS.YELLOW,
            WaymonicTLS.ARROW_GO: TLS.GREEN,
            WaymonicTLS.STOP: TLS.RED,
            WaymonicTLS.CAUTION: TLS.YELLOW,
            WaymonicTLS.GO: TLS.GREEN,
            WaymonicTLS.FLASHING_STOP: TLS.RED,
            WaymonicTLS.FLASHING_CAUTION: TLS.YELLOW,
        }
        return mapping[self]


class WaymoLane:
    def __init__(
        self,
        speed_limit_mph,
        type,
        polyline,
        interpolating,
        entry_lanes,
        exit_lanes,
        left_neighbors,
        right_neighbors,
        left_boundaries,
        right_boundaries,
    ) -> None:
        self.speed_limit_mph: float = speed_limit_mph
        self.type: int = type
        """
        0 :undefined, 1: highway, 2: surface street (urban), 3: bikelane
        """
        self.polyline: list[Pt] = [Pt(point.x, point.y, point.z) for point in polyline]
        self.interpolating: bool = interpolating
        self.entry_lanes: list[int] = [lane for lane in entry_lanes]
        self.exit_lanes: list[int] = [lane for lane in exit_lanes]
        self.left_neighbors: list[Neighbor] = [
            Neighbor(
                neighbor.feature_id,
                neighbor.self_start_index,
                neighbor.self_end_index,
                neighbor.neighbor_start_index,
                neighbor.neighbor_end_index,
            )
            for neighbor in left_neighbors
        ]
        self.right_neighbors: list[Neighbor] = [
            Neighbor(
                neighbor.feature_id,
                neighbor.self_start_index,
                neighbor.self_end_index,
                neighbor.neighbor_start_index,
                neighbor.neighbor_end_index,
            )
            for neighbor in right_neighbors
        ]

        self.left_boundaries: list[Boundary] = [
            Boundary(
                boundary.lane_start_index,
                boundary.lane_end_index,
                boundary.boundary_type,
                boundary.boundary_feature_id,
            )
            for boundary in left_boundaries
        ]
        self.right_boundaries: list[Boundary] = [
            Boundary(
                boundary.lane_start_index,
                boundary.lane_end_index,
                boundary.boundary_type,
                boundary.boundary_feature_id,
            )
            for boundary in right_boundaries
        ]

        self.diverge_lanes: set[int] = set()
        self.merge_lanes: set[int] = set()


class Neighbor:
    def __init__(
        self,
        feature_id: int,
        self_start_index: int = 0,
        self_end_index: int = 0,
        neighbor_start_index: int = 0,
        neighbor_end_index: int = 0,
    ) -> None:
        self.feature_id: int = feature_id
        self.self_start_index: int = self_start_index
        self.self_end_index: int = self_end_index
        self.neighbor_start_index: int = neighbor_start_index
        self.neighbor_end_index: int = neighbor_end_index


class Boundary:
    def __init__(self, lane_start_index, lane_end_index, type, feature_id, polyline: list = []) -> None:
        self.lane_start_index: int = lane_start_index
        self.lane_end_index: int = lane_end_index
        self.type: int = type
        """
        type: 0 - unknown | road edges
        1~8 different road lines
        """
        self.feature_id: int = feature_id
        self.polyline: list[Pt] = [Pt(point.x, point.y, point.z) for point in polyline]  # to be inserted later

class LaneCenter:
    def __init__(
        self, id, lane, needs_stop: bool = False, tl_state_record: list[WaymonicTLS] = [WaymonicTLS.ABSENT for _ in range(91)]
    ) -> None:
        self.id: int = id
        self.lane: WaymoLane = WaymoLane(
            lane.speed_limit_mph,
            lane.type,
            lane.polyline,
            lane.interpolating,
            lane.entry_lanes,
            lane.exit_lanes,
            lane.left_neighbors,
            lane.right_neighbors,
            lane.left_boundaries,
            lane.right_boundaries,
        )

        # extra attributes for the the ease of computation
        self.needs_stop: bool = needs_stop
        self.to_SUMO_edge: int = None
        self.to_SUMO_lane: int = None
        self.record_tls: list[WaymonicTLS] = tl_state_record[:]  # -1 when there is no associated tl