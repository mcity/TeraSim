cimport cython
from libc.math cimport M_PI, atan2, cos, pow, sin, sqrt

import numpy as np

cimport numpy as np

# Constants
DEG_TO_RAD = M_PI / 180.0
DEFAULT_DISTANCE_THRESHOLD = 30.0

cpdef double angle_difference(double angle1, double angle2):
    """Compute the absolute difference between two angles in degrees."""
    # Compute the difference between the two angles and reduce it to the range [-180, 180]
    cdef double diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)

cpdef double get_sumo_angle(double np_angle):
    """Convert numpy angle to SUMO angle format."""
    cdef double sumo_angle = (90 - np_angle) % 360
    return sumo_angle

cpdef double calculate_distance(double x1, double y1, double x2, double y2):
    """Calculate Euclidean distance between two points.
    
    Args:
        x1, y1: Coordinates of first point
        x2, y2: Coordinates of second point
    
    Returns:
        double: Euclidean distance
    """
    cdef double dx = x2 - x1
    cdef double dy = y2 - y1
    return sqrt(dx * dx + dy * dy)

cpdef np.ndarray[double, ndim=2] get_circle_centers(np.ndarray[double, ndim=1] point, 
                                                   double agent_length, double agent_width,
                                                   str agent_type):
    """Calculate circle centers for collision detection.
    
    Args:
        point: Point coordinates and heading [x, y, heading]
        agent_length: Length of the agent
        agent_width: Width of the agent
        agent_type: Type of agent ('vehicle' or other)
    
    Returns:
        np.ndarray: Array of circle center coordinates
    """
    cdef double heading = point[2]
    cdef double cos_heading = cos(heading)
    cdef double sin_heading = sin(heading)
    cdef double offset
    cdef np.ndarray[double, ndim=2] center_list
    cdef int num_circles
    cdef int i

    if agent_type == 'vehicle':
        num_circles = np.ceil(agent_length / agent_width) + 1

        offset = agent_length / num_circles
        center_list = np.zeros((num_circles, 2))
        
        if num_circles % 2 == 0:
            for i in range(num_circles):
                center_list[i, 0] = point[0] + (i + 0.5 - num_circles / 2) * offset * cos_heading
                center_list[i, 1] = point[1] + (i + 0.5 - num_circles / 2) * offset * sin_heading
        else:
            for i in range(num_circles):
                center_list[i, 0] = point[0] + (i - num_circles // 2) * offset * cos_heading
                center_list[i, 1] = point[1] + (i - num_circles // 2) * offset * sin_heading
    else:
        # Single circle for other agents
        center_list = np.zeros((1, 2))
        center_list[0, 0] = point[0]
        center_list[0, 1] = point[1]
    
    return center_list

cpdef double calculate_circle_radius(double length, double width, str agent_type):
    """Calculate circle radius for collision detection based on agent type and dimensions.
    
    Args:
        length: Agent length
        width: Agent width
        agent_type: Type of agent ('vehicle' or other)
    
    Returns:
        double: Circle radius
    """
    cdef int num_circles
    if agent_type == "vehicle":
        num_circles = np.ceil(length / width) + 1
        return sqrt((length/num_circles/2.0)**2 + (width/2.0)**2)
    else:
        return max(length, width) / 2.0 