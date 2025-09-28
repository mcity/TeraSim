import math
import numpy as np
from typing import Union
from shapely.geometry import LineString, Polygon, MultiPolygon
from shapely.ops import substring, unary_union
from shapelysmooth import chaikin_smooth
from scipy.spatial import cKDTree

from .generic import Pt, Direction, UnionFind

def polygon_remove_holes(geometry: Union[Polygon, MultiPolygon]) -> Union[Polygon, MultiPolygon]:
    """
    Given a geometry represented by shapely polygon or multipolygon,
    remove all the holes within the geometry
    """
    assert isinstance(geometry, Polygon) or isinstance(
        geometry, MultiPolygon
    ), "Input must be a Polygon or MultiPolygon"

    if isinstance(geometry, Polygon):
        return Polygon(geometry.exterior)
    elif isinstance(geometry, MultiPolygon):
        polygons_without_holes: list[Polygon] = [Polygon(polygon.exterior) for polygon in geometry.geoms]
        merged_geom = unary_union(polygons_without_holes)
        return merged_geom


def multipolygon_force_union(geometry: MultiPolygon) -> Polygon:
    assert isinstance(geometry, MultiPolygon)

    polygons = [Polygon(polygon.exterior) for polygon in geometry.geoms]
    buffer_distance = 0.1
    merged_polygon = unary_union(polygons)
    while not isinstance(merged_polygon, Polygon):
        buffer_distance *= 2
        buffered_polygons = [polygon.buffer(buffer_distance) for polygon in polygons]
        merged_polygon = unary_union(buffered_polygons)

    merged_polygon = merged_polygon.buffer(-buffer_distance)
    if isinstance(merged_polygon, Polygon):
        return merged_polygon
    else:
        return None


def xy_to_latlon(
    x: float, y: float, base_latitude: float = 40.0, base_longitude: float = -73.0
) -> tuple[float]:
    """
    Translate x, y coordinates to 'fake' latitude, longitude.

    :param float x: X coordinate in meters
    :param float y: Y coordinate in meters
    :return tuple: Corresponding 'fake' latitude and longitude
    """

    # the default base latitude and longitude(40.0, -73.0) is roughly New York City.

    R = 6371000  # radius of the earth
    fake_latitude = base_latitude + y / R * (180 / np.pi)
    fake_longitude = base_longitude + x / (R * np.cos(base_latitude * np.pi / 180)) * (180 / np.pi)

    return fake_latitude, fake_longitude


def distance_between_points(pt1: Pt, pt2: Pt) -> float:
    """
    Return the distance between two points pt1 and pt2
    """
    if any([pt1.z is None, pt2.z is None]):
        return ((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2) ** 0.5
    return ((pt1.x - pt2.x) ** 2 + (pt1.y - pt2.y) ** 2 + (pt1.z - pt2.z) ** 2) ** 0.5


def points_average(points_list: list[Pt]) -> Pt:
    """
    Return the geometric average of the list of points
    """
    avg_x = sum([point.x for point in points_list]) / len(points_list)
    avg_y = sum([point.y for point in points_list]) / len(points_list)
    avg_z = sum([point.z for point in points_list]) / len(points_list)
    return Pt(avg_x, avg_y, avg_z)


def two_lines_parallel(line1, line2, LINE_PARALLEL_THRESHOLD: float = 15) -> bool:
    """
    Calculate the angle of two lines represented by line1 and line2,
    and return True if they are nearly parallel (and in the same direction)

    'LINE_PARALLEL_THRESHOLD' unit is degree
    """
    # Convert line segments to vectors
    vec1 = (line1[1][0] - line1[0][0], line1[1][1] - line1[0][1])
    vec2 = (line2[1][0] - line2[0][0], line2[1][1] - line2[0][1])

    # calculate dot product
    dot_product = vec1[0] * vec2[0] + vec1[1] * vec2[1]
    # calculate magnitude of vectors
    mag_vec1 = math.sqrt(vec1[0] ** 2 + vec1[1] ** 2)
    mag_vec2 = math.sqrt(vec2[0] ** 2 + vec2[1] ** 2)

    # calculate cosine of angle
    cos_angle = dot_product / (mag_vec1 * mag_vec2)

    # prevent floating point error from exceeding domain
    cos_angle = max(min(cos_angle, 1), -1)

    # calculate angle (in degrees)
    angle = math.acos(cos_angle) * (180 / math.pi)

    # check if angle is close to 0
    return angle < LINE_PARALLEL_THRESHOLD


def polyline_length(polyline: list[Pt]) -> float:
    """
    Return the length of this lane.
    """
    distance_list = [distance_between_points(polyline[i - 1], polyline[i]) for i in range(1, len(polyline))]
    return sum(distance_list)


def find_polyline_nearest_point(polyline: list[Pt], reference_point: Pt) -> int:
    """
    Find the point to be truncated at. The point is on polyline, and nearest to the reference_point.
    """

    # feature is to be truncated by the nearest point of reference_point
    distances = [distance_between_points(pt, reference_point) for pt in polyline]
    return distances.index(min(distances))


def polyline_distance(polyline1: list[Pt], polyline2: list[Pt]) -> tuple[float]:
    curve1 = np.array([pt.to_list() for pt in polyline1])
    curve2 = np.array([pt.to_list() for pt in polyline2])
    tree = cKDTree(curve2)
    distances, _ = tree.query(curve1)
    average_distance = np.mean(distances)
    max_distance = np.max(distances)
    min_distance = np.min(distances)
    return average_distance, max_distance, min_distance


def real_neighbor_type(
    polyline1: list[Pt],
    polyline2: list[Pt],
    POINT_CLOSE_THRESHOLD: float = 5,
    LENGTH_DIFFERENCE_THRESHOLD: float = 3,
) -> str:
    """
    Return the type of neighborship between feature1 and feature2.

       feature1 and feature2 should be already real neighbors. Feature1 should be longer than feature2.
    """
    start_point_close = distance_between_points(polyline1[0], polyline2[0]) < POINT_CLOSE_THRESHOLD
    end_point_close = distance_between_points(polyline1[-1], polyline2[-1]) < POINT_CLOSE_THRESHOLD
    polyline1_length = polyline_length(polyline1)
    polyline2_length = polyline_length(polyline2)
    polyline1_longer = polyline2_length + LENGTH_DIFFERENCE_THRESHOLD < polyline1_length

    if start_point_close and end_point_close:
        return "complete"
    elif start_point_close and (not end_point_close) and polyline1_longer:
        return "side-start"
    elif (not start_point_close) and end_point_close and polyline1_longer:
        return "side-end"
    else:
        return "other"


def shape_str(shape: list[Pt]) -> str:
    """
    Given a list of points, return its str version that fits the .xml file format
    """
    if all([pt.z is not None for pt in shape]):
        return " ".join([f"{point.x},{point.y},{point.z}" for point in shape])
    else:
        return " ".join([f"{point.x},{point.y}" for point in shape])


def interpolate(coords: list[tuple], length: float, step: float = 0.5) -> list[tuple]:
    """
    interpolate the line 'coords'. the function does two things:
    1. smoothing the original line using chaikin_smooth (only for x,y plane)
    2. interpolate the line with 'step' as step size
    3. preserve and interpolate z coordinates
    """
    # Extract coordinates
    xy_coords = [(pt[0], pt[1]) for pt in coords]
    z_coords = [pt[2] for pt in coords]
    
    # Smooth x,y coordinates (Chaikin only supports 2D)
    smoothed_xy = chaikin_smooth(xy_coords)
    linestring = LineString(smoothed_xy)
    
    # Calculate cumulative distances for z interpolation
    original_distances = [0]
    for i in range(1, len(coords)):
        dist = ((coords[i][0] - coords[i-1][0])**2 + 
                (coords[i][1] - coords[i-1][1])**2)**0.5
        original_distances.append(original_distances[-1] + dist)
    
    # Generate new points
    points_array = np.arange(0, length, step)
    new_xy = [substring(linestring, start_dist=i, end_dist=i) for i in points_array]
    new_xy = [(pt.x, pt.y) for pt in new_xy]
    
    # Add end point
    last_point = substring(linestring, start_dist=length, end_dist=length)
    new_xy.append((last_point.x, last_point.y))
    
    # Interpolate z based on the actual arc length
    new_distances = [0]
    for i in range(1, len(new_xy)):
        dist = ((new_xy[i][0] - new_xy[i-1][0])**2 + 
                (new_xy[i][1] - new_xy[i-1][1])**2)**0.5
        new_distances.append(new_distances[-1] + dist)
    
    # Only apply scaling if the total distance is not zero
    if new_distances[-1] > 1e-10:  # Using small epsilon for floating point comparison
        scale = original_distances[-1] / new_distances[-1]
        new_distances = [d * scale for d in new_distances]
    
    # Interpolate z coordinates based on arc length
    new_z = np.interp(new_distances, original_distances, z_coords)
    
    # Combine coordinates
    new_coords = [(x, y, z) for (x, y), z in zip(new_xy, new_z)]
    
    return new_coords


def calculate_turning_angle(points):
    """in radians"""
    # calculate vector between adjacent points
    vectors = np.diff(points, axis=0)

    # calculate direction angle of each vector (in radians)
    angles = np.arctan2(vectors[:, 1], vectors[:, 0])

    # calculate difference between adjacent direction angles
    angle_diffs = np.diff(angles)

    # limit angle difference to [-pi, pi]
    angle_diffs = (angle_diffs + np.pi) % (2 * np.pi) - np.pi

    # calculate total turning angle
    total_turning_angle = np.sum(angle_diffs)

    return total_turning_angle


def classify_direction(shape) -> Direction:

    total_turn_angle = calculate_turning_angle(shape)
    if np.abs(total_turn_angle) < np.pi / 6:
        return Direction.S
    elif total_turn_angle > 0:
        return Direction.L
    else:
        return Direction.R


def compute_direction_vector(x, y):
    # calculate direction vector using start and end points
    direction_vector = np.array([x[-1] - x[0], y[-1] - y[0]])
    return normalized_vector(direction_vector)  # normalize


def normalized_vector(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm


def points_to_vector(pt_start: Pt, pt_end: Pt) -> np.ndarray:
    return np.array([pt_end.x - pt_start.x, pt_end.y - pt_start.y])


def vector_heading(vec, unit: str = "radian") -> float:
    """Heading angle of a single vector in Cartesian coordinate system [-pi, pi]

    unit = 'degree' | 'radian'"""

    if unit == "radian":
        return np.arctan2(vec[1], vec[0])
    elif unit == "degree":
        return np.rad2deg(np.arctan2(vec[1], vec[0]))


def heading_2_unit_vector(angle, unit: str = "radian") -> np.ndarray:
    """unit = 'degree' | 'radian'"""

    if unit == "degree":
        angle = np.deg2rad(angle)

    return np.array([np.cos(angle), np.sin(angle)])


def angle_of_two_vectors(vec1, vec2, unit: str = "radian") -> float:
    """unit = 'degree' | 'radian'
    angle between two vectors in Cartesian coordinate system [-pi, pi]
    """
    # vec1, vec2 = (x,y, [z])
    if np.linalg.norm(vec1) == 0 or np.linalg.norm(vec2) == 0:
        raise ValueError("One of the vectors is a zero vector.")

    cos_theta = np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    if unit == "radian":
        return np.arccos(cos_theta)  # [0, pi]
    elif unit == "degree":
        return np.degrees(np.arccos(cos_theta))  # [0, 180]
    else:
        raise Exception()


def angle_of_twoheadings(angle1, angle2, unit: str = "radian"):
    """angle difference between two headings, unit ='degree' | 'radian'"""

    if unit == "degree":
        angle1, angle2 = np.deg2rad(angle1), np.deg2rad(angle2)

    diff = np.abs(angle1 - angle2)
    # adjust difference to < 2 pi
    while diff > 2 * np.pi:
        diff -= np.pi * 2
    # if diff > 180, reverse it
    if diff > np.pi:
        diff = 2 * np.pi - diff

    if unit == "radian":
        return diff  # [0, pi]
    elif unit == "degree":
        return np.rad2deg(diff)  # [0, 180]

def group_vectors_by_angles(lane_vectors: list[np.ndarray], ANGLE_CRITERIA: float = np.pi/6) -> list[list[int]]:
    uf = UnionFind(len(lane_vectors))  # uf[i] corresponds to node.incoming_SUMO_edges[i]
    for i in range(len(lane_vectors)):
        for j in range(len(lane_vectors)):
            if angle_of_two_vectors(lane_vectors[i], lane_vectors[j]) < ANGLE_CRITERIA:
                uf.union(i, j)
    return uf.form_groups()


def point_side_of_polyline(point: Pt, polyline: list[Pt]):

    index = find_polyline_nearest_point(polyline, point)
    if index == len(polyline) - 1:
        start_point = polyline[index-1]
        end_point = polyline[index]
    else:
        start_point = polyline[index]
        end_point = polyline[index+1]

    vector_curve = points_to_vector(start_point, end_point) # p1->p2
    vector_point = points_to_vector(start_point, point) # p1->point
    cross_product = vector_curve[0] * vector_point[1] - vector_curve[1] * vector_point[0]

    if cross_product >= 0:
        return "Left"
    else:
        return "Right"
