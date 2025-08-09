cimport cython
from libc.math cimport pow, sqrt

import numpy as np

cimport numpy as np

from shapely.geometry import LineString

from ..geometry.geometry_utils_cy import (calculate_circle_radius,
                                          calculate_distance,
                                          get_circle_centers)
from ..trajectory.trajectory_utils_cy import \
    sumo_trajectory_to_normal_trajectory

# Constants
DEFAULT_DISTANCE_THRESHOLD = 30.0

def check_collision(np.ndarray[double, ndim=2] traj1,
                   np.ndarray[double, ndim=2] traj2,
                   double agent1_length,
                   double agent2_length,
                   double agent1_width,
                   double agent2_width,
                   str agent1_type,
                   str agent2_type,
                   double buffer):
    """Check for collision between two trajectories.
    
    Args:
        traj1, traj2: Trajectories to check.
        agent1_length, agent2_length: Agent lengths.
        agent1_width, agent2_width: Agent widths.
        agent1_type, agent2_type: Agent types.
        buffer: Safety buffer distance.
    
    Returns:
        tuple: (bool: collision detected, float: collision time or None).
    """
    # Convert trajectories to normal format
    traj1 = sumo_trajectory_to_normal_trajectory(traj1, agent1_length)
    traj2 = sumo_trajectory_to_normal_trajectory(traj2, agent2_length)
    
    # Calculate circle radii
    cdef double circle_r1 = calculate_circle_radius(agent1_length, agent1_width, agent1_type)
    cdef double circle_r2 = calculate_circle_radius(agent2_length, agent2_width, agent2_type)
    
    cdef np.ndarray[double, ndim=2] center_list_1, center_list_2
    cdef np.ndarray[double, ndim=1] traj_point1, traj_point2
    cdef double dist
    
    for i in range(traj1.shape[0]):
        traj_point1 = traj1[i]
        traj_point2 = traj2[i]
        
        center_list_1 = get_circle_centers(traj_point1, agent1_length, agent1_width, agent1_type)
        center_list_2 = get_circle_centers(traj_point2, agent2_length, agent2_width, agent2_type)
        
        for j in range(center_list_1.shape[0]):
            for k in range(center_list_2.shape[0]):
                dist = calculate_distance(
                    center_list_1[j, 0], center_list_1[j, 1],
                    center_list_2[k, 0], center_list_2[k, 1]
                )
                if dist <= circle_r1 + circle_r2 + buffer * 2.0:
                    return True, traj1[i, 3]
    return False, None

def check_trajectory_intersection(np.ndarray[double, ndim=2] trajectory1,
                                np.ndarray[double, ndim=2] trajectory2,
                                double agent1_length,
                                double agent2_length,
                                double agent1_width,
                                double agent2_width,
                                str agent1_type,
                                str agent2_type,
                                double buffer,
                                double distance_threshold=DEFAULT_DISTANCE_THRESHOLD):
    """Check if two trajectories intersect.
    
    Args:
        trajectory1, trajectory2: Trajectories to check.
        agent1_length, agent2_length: Agent lengths.
        agent1_width, agent2_width: Agent widths.
        agent1_type, agent2_type: Agent types.
        buffer: Safety buffer distance.
        distance_threshold: Maximum initial distance to consider for intersection.
    
    Returns:
        bool: Whether trajectories intersect.
    """
    # Quick distance check
    cdef double initial_distance = calculate_distance(
        trajectory1[0, 0], trajectory1[0, 1],
        trajectory2[0, 0], trajectory2[0, 1]
    )
    if initial_distance > distance_threshold:
        return False

    # Check for collision
    collision_detected, _ = check_collision(
        trajectory1, trajectory2,
        agent1_length, agent2_length,
        agent1_width, agent2_width,
        agent1_type, agent2_type,
        buffer
    )
    if collision_detected:
        return True

    # Check for path intersection section by section    
    cdef int i
    for i in range(1, trajectory1.shape[0]):
        line1 = LineString(trajectory1[i-1:i+1, :2])
        line2 = LineString(trajectory2[i-1:i+1, :2])
        if line1.intersects(line2):
            return True
    return False