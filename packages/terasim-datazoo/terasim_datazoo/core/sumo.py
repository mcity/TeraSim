from typing import Union
from enum import Enum
import numpy as np

from shapely import LineString, Polygon, MultiPolygon
from shapely.ops import unary_union

from .waymo import LaneCenter, Boundary
from terasim_datazoo.utils import Pt, Direction, classify_direction, polygon_remove_holes, multipolygon_force_union, points_to_vector

class LaneTypeSUMO(Enum):
    NORMAL = "normal"
    BIKE = "bike"
    SIDEWALK = "sidewalk"

class Node:
    def __init__(
        self,
        index: int,
        x: float,
        y: float,
        z: float,
        type: str = "priority",
        shape: list[Pt] = None,
        keep_clear: bool = True,
    ) -> None:
        # to be filled at initialization
        self.id: int = index
        self.x: float = x
        self.y: float = y
        self.z: float = z
        self.type: str = type
        self.shape: list[Pt] = shape[:] if shape else None
        self.keep_clear: bool = keep_clear

        # to be filled after initialization
        self.tl_program: list[SumoTLPhase] = []
        self.connections: set[Connection] = set()
        self.prohibitions: set[Prohibition] = set()
        self.crossings: list[Crossing] = []
        self.walking_areas: list[WalkingArea] = []

        # extra attributes for the ease of computation
        self.outgoing_SUMO_edges: set[int] = set()
        self.incoming_SUMO_edges: set[int] = set()


class Lane:
    def __init__(self, speed: float, shape: list[Pt], width, type: LaneTypeSUMO = LaneTypeSUMO.NORMAL) -> None:
        # to be filled at initialization
        self.speed: float = speed
        self.width: float = width
        self.shape: list[Pt] = shape[:]
        self.type: LaneTypeSUMO = type

        # extra attributes for the ease of computations
        self.to_WAYMOfeature: int = None


class Edge:
    def __init__(self, index: int, lanes: list[Lane], from_node_id: int = None, to_node_id: int = None) -> None:
        # to be filled at initialization
        self.id: int = index
        self.lanes: list[Lane] = [lane for lane in lanes]

        # to be filled after initialization
        self.from_node_id: int = from_node_id
        self.to_node_id: int = to_node_id

    def get_estimation_vector(self, side: str, max_num_points: int= 50) -> np.ndarray:
        assert len(self.lanes)
        first_lane = self.lanes[len(self.lanes)//2]
        if side == "entry":
            start_point = first_lane.shape[-min(max_num_points, len(first_lane.shape))]
            end_point = first_lane.shape[-1]
        else:
            start_point = first_lane.shape[0]
            end_point = first_lane.shape[min(max_num_points, len(first_lane.shape) - 1)]
        return points_to_vector(start_point, end_point)


class Connection:
    def __init__(
        self,
        from_edge: int,
        to_edge: int,
        from_lane: int,
        to_lane: int,
        speed: float = None,
        shape: list[Pt] = None,
        can_pass: bool = False,
        need_stop: bool = False,
        WAYMO_features: list[int] = [],
        rec_tl_state: list[int] = [-1 for _ in range(0, 91)],
    ) -> None:
        # all to be filled at initialization
        self.from_edge: int = from_edge
        self.to_edge: int = to_edge
        self.from_lane: int = from_lane
        self.to_lane: int = to_lane
        self.speed: float = speed
        self.shape: list[Pt] = shape

        # below are attributes for the ease of computation
        self.can_pass: bool = can_pass
        """can_pass are only True when connection is in a junction of type
        priority_stop && there is no associated stop sign"""

        self.need_stop: bool = need_stop
        """need_stop are True only when connection is in a junction of type
        priority_stop && there is associated stop sign"""
        self.WAYMO_features: list[int] = WAYMO_features

        self.tl_state_record: list[int] = rec_tl_state
        self.direction: Direction = classify_direction([(pt.x, pt.y) for pt in shape]) if shape else None
        self.apply_shape: bool = True


class Prohibition:
    def __init__(self, prohibitor_from: int, prohibitor_to: int, prohibited_from: int, prohibited_to: int):
        # all to be filled at initialization
        self.prohibitor_from: int = prohibitor_from
        self.prohibitor_to: int = prohibitor_to
        self.prohibited_from: int = prohibited_from
        self.prohibited_to: int = prohibited_to

class Crossing:
    def __init__(self, edges: list[int], width: float = 3, shape:list[Pt] = None, outline_shape: list[Pt] = None,) -> None:
        self.shape = shape[:]
        self.edges: list[int] = edges[:]
        self.width = width
        self.outline_shape = outline_shape[:] if outline_shape else None


class WalkingArea:
    def __init__(self, edges: list[int], shape: list[Pt] = None) -> None:
        self.edges: list[int] = edges[:]
        self.shape: list[Pt] = shape[:] if shape else None


class SumonicTLS(Enum):
    RED = "r"
    YELLOW = "y"
    GREEN = "G"
    GREEN_MINOR = "g"
    GREEN_RIGHT = "s"
    OFF_MAJOR = "O"
    OFF_MINOR = "o"


class SumoTLPhase:
    def __init__(self) -> None:
        self.duration: float = None
        self.states: list[SumonicTLS] = []
        self.connections: list[str] = []


def create_edges_polygon(
    features: dict[int, LaneCenter],
    edges: list[Edge],
    use_boundary: bool = True,
    buffer_distance: float = 0.1,
    allow_multipolygon: bool = True,
) -> Union[Polygon, MultiPolygon, None]:
    # try:
        polygons = [create_edge_polygon(features, edge, use_boundary=use_boundary) for edge in edges]
        merged_polygon = unary_union(polygons).buffer(buffer_distance)
        merged_polygon = polygon_remove_holes(merged_polygon)
        if isinstance(merged_polygon, Polygon):
            return merged_polygon
        elif isinstance(merged_polygon, MultiPolygon):
            if allow_multipolygon:
                return merged_polygon
            else:
                return multipolygon_force_union(merged_polygon)


def create_edge_polygon(
    features: dict[int, LaneCenter], edge: Edge, use_boundary: bool = True
) -> Union[Polygon, MultiPolygon]:
    """
    Return the driving area polygon of a sumo edge
    """

    # find polygons of each individual lane
    all_lane_polygons = [create_lane_polygon(features, lane, use_boundary) for lane in edge.lanes]
    edge_polygon_union = unary_union(all_lane_polygons)

    # remove holes
    edge_polygon_union = polygon_remove_holes(edge_polygon_union)
    # make small buffer
    edge_polygon_union = edge_polygon_union.buffer(-0.01)

    return edge_polygon_union


def create_lane_polygon(
    features: dict[int, LaneCenter], lane: Lane, use_boundary: bool = True
) -> Union[Polygon, MultiPolygon]:
    """
    Return the driving area polygon of a sumo lane
    """

    if len(lane.shape) == 1:
        return Polygon()

    # default polygon
    lane_linestring = LineString([(pt.x, pt.y) for pt in lane.shape])
    default_polygon = lane_linestring.buffer(lane.width / 2, cap_style="flat")

    if not use_boundary:
        return default_polygon

    # all boundary polygons
    boundary_polygons: list[Polygon] = []
    for left_boundary in features[lane.to_WAYMOfeature].lane.left_boundaries:
        boundary_polygon = create_boundary_polygon(features, left_boundary, lane)
        boundary_polygons.append(boundary_polygon)
    for right_boundary in features[lane.to_WAYMOfeature].lane.right_boundaries:
        boundary_polygon = create_boundary_polygon(features, right_boundary, lane)
        boundary_polygons.append(boundary_polygon)

    lane_polygon = unary_union([*boundary_polygons, default_polygon])

    return lane_polygon


def create_boundary_polygon(
    features: dict[int, LaneCenter], boundary: Boundary, lane: Lane
) -> Union[Polygon, MultiPolygon]:
    """
    Return the driving area polygon of a segment of a sumo lane that is constraint by a segment of boundary.
    """

    lane_center_segment = [
        (pt.x, pt.y)
        for pt in features[lane.to_WAYMOfeature].lane.polyline[
            boundary.lane_start_index : boundary.lane_end_index + 1
        ]
    ]
    boundary_segment = [(pt.x, pt.y) for pt in boundary.polyline]
    boundary_polygon = Polygon(lane_center_segment + boundary_segment[::-1])

    return boundary_polygon
