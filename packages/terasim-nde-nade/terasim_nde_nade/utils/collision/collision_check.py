import math
import numpy as np
from shapely.geometry import LineString
from scipy.interpolate import interp1d

# Constants
M_PI = math.pi
DEG_TO_RAD = M_PI / 180.0
DEFAULT_DISTANCE_THRESHOLD = 50.0
SUMO_TO_NORMAL_ANGLE_OFFSET = 90.0


def angle_difference(angle1, angle2):
    """Compute the absolute difference between two angles in degrees."""
    diff = (angle1 - angle2 + 180) % 360 - 180
    return abs(diff)


def get_sumo_angle(np_angle):
    """Convert numpy angle to SUMO angle format."""
    sumo_angle = (90 - np_angle) % 360
    return sumo_angle


def calculate_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points.
    
    Args:
        x1, y1: Coordinates of first point
        x2, y2: Coordinates of second point
    
    Returns:
        float: Euclidean distance
    """
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx * dx + dy * dy)


def get_circle_centers(point, agent_length, agent_width, agent_type):
    """Calculate circle centers for collision detection.
    
    Args:
        point: Point coordinates and heading [x, y, heading]
        agent_length: Length of the agent
        agent_width: Width of the agent
        agent_type: Type of agent ('vehicle' or other)
    
    Returns:
        np.ndarray: Array of circle center coordinates
    """
    heading = point[2]
    cos_heading = math.cos(heading)
    sin_heading = math.sin(heading)

    if agent_type == 'vehicle':
        num_circles = int(np.ceil(agent_length / agent_width)) + 1
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


def calculate_circle_radius(length, width, agent_type):
    """Calculate circle radius for collision detection based on agent type and dimensions.
    
    Args:
        length: Agent length
        width: Agent width
        agent_type: Type of agent ('vehicle' or other)
    
    Returns:
        float: Circle radius
    """
    if agent_type == "vehicle":
        num_circles = int(np.ceil(length / width)) + 1
        return math.sqrt((length/num_circles/2.0)**2 + (width/2.0)**2)
    else:
        return max(length, width) / 2.0


def sumo_trajectory_to_normal_trajectory(sumo_trajectory, veh_length=5.0):
    """Convert SUMO trajectory format to normal trajectory format.
    
    Args:
        sumo_trajectory: Trajectory in SUMO format
        veh_length: Vehicle length
    
    Returns:
        np.ndarray: Converted trajectory
    """
    n = sumo_trajectory.shape[0]
    normal_trajectory = np.empty((n, sumo_trajectory.shape[1]))

    for i in range(n):
        rad = (SUMO_TO_NORMAL_ANGLE_OFFSET - sumo_trajectory[i, 2]) * DEG_TO_RAD
        normal_trajectory[i, 2] = math.atan2(math.sin(rad), math.cos(rad))
        normal_trajectory[i, 0] = sumo_trajectory[i, 0] - veh_length / 2 * math.cos(normal_trajectory[i, 2])
        normal_trajectory[i, 1] = sumo_trajectory[i, 1] - veh_length / 2 * math.sin(normal_trajectory[i, 2])
        if sumo_trajectory.shape[1] > 3:
            normal_trajectory[i, 3:] = sumo_trajectory[i, 3:]
    return normal_trajectory


def interpolate_future_trajectory(trajectory_list_array, interpolate_resolution):
    """Interpolate trajectory with given resolution.
    
    Args:
        trajectory_list_array: Original trajectory array
        interpolate_resolution: Desired time resolution for interpolation
    
    Returns:
        np.ndarray: Interpolated trajectory
    """
    time_values = trajectory_list_array[:, -1]
    position_values = trajectory_list_array[:, :-1]
    
    # Handle angle interpolation (column index 2)
    angles = position_values[:, 2].copy()
    
    # Calculate the differences between consecutive angles
    angle_diffs = np.diff(angles)
    
    # Calculate the wrapped differences
    wrapped_diffs = angle_diffs % 360
    wrapped_diffs[wrapped_diffs > 180] -= 360
    
    # Choose the shortest path for each difference
    shortest_diffs = np.where(
        np.abs(wrapped_diffs) < np.abs(angle_diffs),
        wrapped_diffs,
        angle_diffs
    )
    
    # Reconstruct the angles using the shortest differences
    adjusted_angles = np.zeros_like(angles)
    adjusted_angles[0] = angles[0]
    for i in range(1, len(angles)):
        adjusted_angles[i] = adjusted_angles[i-1] + shortest_diffs[i-1]
    
    # Update the position values with the adjusted angles
    position_values[:, 2] = adjusted_angles
    
    # Create interpolation function
    interpolation_function = interp1d(time_values, position_values, axis=0, kind='linear')
    
    # Create new time values
    start_time = time_values[0]
    end_time = time_values[-1]
    num_points = int((end_time - start_time) / interpolate_resolution) + 1
    
    # Generate new time values with consistent intervals
    new_time_values = np.linspace(start_time, start_time + (num_points - 1) * interpolate_resolution, num_points)
    
    # Interpolate position values
    new_position_values = interpolation_function(new_time_values)
    
    # Normalize angles to [0, 360) range
    new_position_values[:, 2] = new_position_values[:, 2] % 360
    
    # Combine time and position values
    return np.hstack((new_position_values, new_time_values[:, None]))


def check_collision(traj1, traj2, agent1_length, agent2_length, agent1_width, agent2_width,
                   agent1_type, agent2_type, buffer):
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
    circle_r1 = calculate_circle_radius(agent1_length, agent1_width, agent1_type)
    circle_r2 = calculate_circle_radius(agent2_length, agent2_width, agent2_type)
    
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


def check_trajectory_intersection(trajectory1, trajectory2, agent1_length, agent2_length,
                                agent1_width, agent2_width, agent1_type, agent2_type,
                                buffer, distance_threshold=DEFAULT_DISTANCE_THRESHOLD):
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
    initial_distance = calculate_distance(
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
    for i in range(1, trajectory1.shape[0]):
        line1 = LineString(trajectory1[i-1:i+1, :2])
        line2 = LineString(trajectory2[i-1:i+1, :2])
        if line1.intersects(line2):
            return True
    return False