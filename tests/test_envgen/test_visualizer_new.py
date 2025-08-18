import os
import json
import re
import xml.etree.ElementTree as ET
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import shutil
import numpy as np
import functools
import multiprocessing
from concurrent.futures import ProcessPoolExecutor, as_completed
from tqdm import tqdm
import sys
import yaml
import sumolib

import terasim_vis


def interpolate_by_distance(points, step):
    """
    Interpolate a tuple of tuples so that the distance between each point is equal to 'step'.

    Args:
        points (tuple of tuple): Original shape, e.g., ((x1, y1), (x2, y2), ...)
        step (float): Desired distance between points.

    Returns:
        list of list: Interpolated points as [[x, y], ...] with equal spacing.
    """
    points = np.array(points, dtype=np.float32)
    # Compute distances between consecutive points
    deltas = np.diff(points, axis=0)
    seg_lengths = np.hypot(deltas[:, 0], deltas[:, 1])
    cumulative = np.insert(np.cumsum(seg_lengths), 0, 0)
    total_length = cumulative[-1]
    if total_length == 0:
        return [points[0].tolist()]
    # Generate equally spaced distances
    num_points = int(np.floor(total_length / step)) + 1
    distances = np.linspace(0, total_length, num_points)
    # Interpolate x and y separately
    x_interp = np.interp(distances, cumulative, points[:, 0])
    y_interp = np.interp(distances, cumulative, points[:, 1])
    return [[float(x), float(y)] for x, y in zip(x_interp, y_interp)]


def generate_construction_zone_shape(lane_shape, lane_width, direction):
    """
    Generate a construction zone shape based on the lane shape and lane width.
    The first ten points of the lane_shape are offset laterally, with the offset
    gradually changing from direction * lane_width/2 to -direction * lane_width/2.
    The remaining points are offset by a constant -direction * lane_width/2.

    Args:
        lane_shape (list of list): The lane shape as a list of [x, y] points.
        lane_width (float): The width of the lane.
        direction (int): -1 for from left to right, 1 for from right to left.

    Returns:
        list of list: The offset lane shape.
    """
    n = min(10, len(lane_shape))
    construction_zone_shape = []
    for i, pt in enumerate(lane_shape):
        pt = np.array(pt)
        # Compute tangent direction
        if i < len(lane_shape) - 1:
            next_pt = np.array(lane_shape[i + 1])
            dir_vec = next_pt - pt
        else:
            prev_pt = np.array(lane_shape[i - 1])
            dir_vec = pt - prev_pt
        norm = np.linalg.norm(dir_vec)
        if norm == 0:
            dir_vec = np.array([1.0, 0.0])
        else:
            dir_vec = dir_vec / norm
        # Normal vector (perpendicular)
        normal = np.array([-dir_vec[1], dir_vec[0]]) * direction * -1

        # Compute offset
        if i < n:
            # Linear interpolation from +lane_width/2 to -lane_width/2
            alpha = i / (n - 1) if n > 1 else 0
            offset_val = (1 - alpha) * (lane_width / 2) + alpha * (-lane_width / 2)
        else:
            offset_val = - lane_width / 2

        offset_pt = pt + normal * offset_val
        construction_zone_shape.append(offset_pt.tolist())
    return construction_zone_shape


def check_mp4_file_valid(mp4_path: str) -> bool:
    """
    Check if the mp4 file is valid
    """
    if os.path.exists(mp4_path):
        # Try to open the video file to check if it's valid
        try:
            import cv2
            cap = cv2.VideoCapture(mp4_path)
            if not cap.isOpened():
                print(f"Warning: MP4 file exists but is corrupted: {mp4_path}")
                return False
            cap.release()
            return True
        except Exception as e:
            print(f"Error checking MP4 file: {e}")
            return False
    else:
        return False


def test_visualizer(map_folder, scenario_folder, track_vehicle_id=None):
    """
    Test visualizer for a specific scenario
    
    Args:
        map_folder: Path to the map folder
        scenario_folder: Path to the scenario folder containing monitor.json
        track_vehicle_id: ID of the vehicle to track (if None, will track first colliding vehicle)
    """

    # Get absolute paths
    map_path = os.path.abspath(map_folder)
    scenario_path = os.path.abspath(scenario_folder)
    
    # check if the visualization mp4 file already exists
    scenario_name = os.path.basename(scenario_path)
    mp4_path = os.path.join(scenario_folder, f"{scenario_name}_visualization.mp4")
    if check_mp4_file_valid(mp4_path):
        print(f"Visualization.mp4 already exists: {mp4_path}")
        return True

    # Check if monitor.json exists
    # monitor_file = os.path.join(scenario_path, "monitor.json")
    # if not os.path.exists(monitor_file):
    #     print(f"No monitor.json found in {scenario_path}")
    #     return False
    
    # check if construction zone file exists
    construction_zone_file = os.path.join(scenario_path, "construction.yaml")
    construction_lane_id = None
    construction_zone_shape = None
    if os.path.exists(construction_zone_file):
        with open(construction_zone_file, 'r') as f:
            try:
                construction_data = yaml.safe_load(f)
                # Process construction data if needed
                if 'adversity_cfg' in construction_data and 'static' in construction_data['adversity_cfg'] and 'construction' in construction_data['adversity_cfg']['static']:
                    construction_lane_id = construction_data['adversity_cfg']['static']['construction'].get('lane_id', None)
            except yaml.YAMLError as e:
                print(f"Error reading {construction_zone_file}: {e}")
                return False
    if construction_lane_id:
        # Determine the shape of the construction zone
        sumo_net = sumolib.net.readNet(os.path.join(map_path, "map.net.xml"))
        lane_shape = sumo_net.getLane(construction_lane_id).getShape()
        if lane_shape: # convert to list of lists
            lane_shape = interpolate_by_distance(lane_shape, 2.0)
            lane_index = int(construction_lane_id.split("_")[-1])
            edge_id = sumo_net.getLane(construction_lane_id).getEdge().getID()
            if lane_index == 0:
                # From right to left
                direction = 1
            elif lane_index == sumo_net.getEdge(edge_id).getLaneNumber() - 1:
                # From left to right
                direction = -1
            else:
                # Middle lane, no construction zone
                pass
            construction_zone_shape = generate_construction_zone_shape(lane_shape, sumo_net.getLane(construction_lane_id).getWidth(), direction)

    # Check for veh_1_id and veh_2_id in monitor.json
    # with open(monitor_file, 'r') as f:
    #     try:
    #         monitor_data = json.load(f)
    #         json_str = json.dumps(monitor_data)
    #         veh_1_id = None
    #         veh_2_id = None
            
    #         # Search for veh_1_id and veh_2_id
    #         veh_1_id_match = re.search(r'"veh_1_id"\s*:\s*"([^"]+)"', json_str)
    #         veh_2_id_match = re.search(r'"veh_2_id"\s*:\s*"([^"]+)"', json_str)
            
    #         if veh_1_id_match:
    #             veh_1_id = veh_1_id_match.group(1)
    #         if veh_2_id_match:
    #             veh_2_id = veh_2_id_match.group(1)
                
    #         if not veh_1_id and not veh_2_id:
    #             print(f"No veh_1_id and veh_2_id found in {monitor_file}")
    #             return False
                
    #         print(f"Found vehicles: veh_1_id={veh_1_id}, veh_2_id={veh_2_id}")
                
    #     except json.JSONDecodeError:
    #         print(f"Invalid JSON in {monitor_file}")
    #         return False
    
    # Load net file and trajectory file
    net_file = os.path.join(map_path, "map.net.xml")
    traj_file = os.path.join(scenario_path, "fcd_all.xml")
    
    if not os.path.exists(net_file):
        print(f"Net file not found: {net_file}")
        return False
    if not os.path.exists(traj_file):
        print(f"Trajectory file not found: {traj_file}")
        return False
    
    # Load data
    try:
        net = SumoTrajVis.Net(net_file)
        trajectories = SumoTrajVis.Trajectories(traj_file)
    except Exception as e:
        print(f"Error loading data: {e}")
        return False
    
    # Find vehicles involved in collisions
    colliding_vehicles = set()
    
    # Add veh_1_id and veh_2_id to the colliding vehicles
    if veh_1_id:
        colliding_vehicles.add(veh_1_id)
    if veh_2_id:
        colliding_vehicles.add(veh_2_id)
    
    # Also check for other collision data in collision.xml
    collision_file = os.path.join(scenario_path, "collision.xml")
    if os.path.exists(collision_file):
        try:
            collision_tree = ET.parse(collision_file)
            collision_root = collision_tree.getroot()
            for collision in collision_root.findall(".//collision"):
                for attr in ['v', 'v1', 'v2']:
                    if attr in collision.attrib:
                        colliding_vehicles.add(collision.attrib[attr])
        except ET.ParseError:
            print(f"Could not parse collision.xml in {scenario_path}")
    
    # If no collisions found in XML, check run.log for collision warnings
    if len(colliding_vehicles) <= 2:  # Just the veh_1_id and veh_2_id
        run_log = os.path.join(scenario_path, "run.log")
        if os.path.exists(run_log):
            with open(run_log) as f:
                log_content = f.read()
                # Find collision warnings in log
                collision_pattern = r"collision with vehicle '([^']+)'"
                matches = re.findall(collision_pattern, log_content)
                colliding_vehicles.update(matches)
                # Also get the vehicles that had collisions
                vehicle_pattern = r"Vehicle '([^']+)'; collision"
                matches = re.findall(vehicle_pattern, log_content)
                colliding_vehicles.update(matches)
    
    print(f"Vehicles involved in collisions: {colliding_vehicles}")
    
    # Determine which vehicle to track
    track_vehicle = track_vehicle_id
    if track_vehicle is None and colliding_vehicles:
        # Default to first colliding vehicle if not specified
        track_vehicle = next(iter(colliding_vehicles))
    
    print(f"Tracking vehicle: {track_vehicle}")
    
    # Set trajectory color for different vehicles
    track_trajectory = None
    for trajectory in trajectories:
        if trajectory.id == track_vehicle:
            trajectory.assign_colors_constant("#ff0000")  # Red for tracked vehicle
            track_trajectory = trajectory  # Save reference to tracked trajectory
        elif trajectory.id in colliding_vehicles:
            trajectory.assign_colors_constant("#ff0000")  # Red for other colliding vehicles
        else:
            trajectory.assign_colors_constant("#00FF00")  # Green for non-colliding vehicles
    
    if track_trajectory is None:
        print(f"Warning: Could not find trajectory for vehicle {track_vehicle}")
        # In case we can't find the specific vehicle, don't try to track
    
    # Create a custom plot_points function to track a specific vehicle
    def custom_plot_points(time, ax, animate_color, lanes, track_traj=None, zoom_range=120.0):
        # Original implementation from SumoTrajVis.Trajectories.plot_points
        
        # First make sure we track our vehicle of interest
        if track_traj:
            values = track_traj._get_values_at_time(time)
            x_track, y_track = values["x"], values["y"]
            if x_track is not None and y_track is not None:
                # print(f"Frame {time}: Focusing on vehicle {track_traj.id} at ({x_track:.2f}, {y_track:.2f})")
                
                # Create a square viewing area around the tracked vehicle
                # This ensures we maintain the same aspect ratio without needing to set axis('equal')
                xlim = (x_track - zoom_range, x_track + zoom_range)
                ylim = (y_track - zoom_range, y_track + zoom_range)
                
                # Don't use axis("equal") - instead set the limits directly
                # as we ensure the viewing area is already square
                ax.set_xlim(xlim)
                ax.set_ylim(ylim)

        # Plot the construction zone if it exists
        if construction_zone_shape:
            # plot shape as scatter points
            construction_zone_shape = np.array(construction_zone_shape)
            ax.scatter(construction_zone_shape[:, 0], construction_zone_shape[:, 1],
                        color='orange', s=10, label='Construction Zone', zorder=5)
        
        # Then call the original plot_points to render everything
        return trajectories.plot_points(time, ax, animate_color, lanes)
    
    # Create visualization
    fig, ax = plt.subplots(figsize=(8, 8))  # Reduced from (10, 10)
    
    # Don't set aspect explicitly, let the square limits handle it
    # ax.set_aspect('equal', adjustable='datalim')  
    artist_collection = net.plot(ax=ax)
    
    # Get the timestep range, focusing on the last 100 frames
    plot_time_interval = trajectories.timestep_range()
    if len(plot_time_interval) > 100:
        plot_time_interval = plot_time_interval[-100:]  # Only show last 100 frames
    
    # Create animation with our custom plotting function
    animation_obj = animation.FuncAnimation(
        fig,
        custom_plot_points,  # Use our custom function instead
        frames=plot_time_interval,
        interval=1,
        # Pass tracked trajectory as additional parameter
        fargs=(ax, True, artist_collection.lanes, track_trajectory, 120.0),
        blit=False,
    )
    
    # Save the animation
    scenario_name = os.path.basename(scenario_path)
    output_file = os.path.join(scenario_path, f"{scenario_name}_visualization.mp4")
    animation_obj.save(output_file, writer=animation.FFMpegWriter(fps=10), dpi=150)  # Reduced from 300
    print(f"Saved visualization to {output_file}")
    plt.close()
    
    return True

# def process_map(map_path):
#     """
#     Process all scenarios in a map folder
    
#     Args:
#         map_path: Path to the map folder
#     """
#     print(f"Processing map: {os.path.basename(map_path)}")
    
#     simulation_output_path = os.path.join(map_path, "simulation_output")
#     if not os.path.exists(simulation_output_path):
#         print(f"No simulation_output folder found in {map_path}")
#         return
    
#     # Process each scenario
#     for scenario_name in os.listdir(simulation_output_path):
#         scenario_path = os.path.join(simulation_output_path, scenario_name)
#         if os.path.isdir(scenario_path):
#             print(f"Processing scenario: {scenario_name}")
#             result = test_visualizer(map_path, scenario_path)
            
#             # If visualization failed (no veh_1_id and veh_2_id), remove the folder
#             if not result:
#                 print(f"Removing scenario folder: {scenario_path}")
#                 shutil.rmtree(scenario_path)

def get_map_scenario_pairs(test_output_dir: str) -> list:
    """
    Generate a list of (map_path, scenario_path) pairs for parallel processing.
    
    Args:
        test_output_dir (str): Directory containing map folders
        
    Returns:
        list: List of tuples (map_path, scenario_path)
    """
    pairs = []
    for map_name in os.listdir(test_output_dir):
        map_path = os.path.join(test_output_dir, map_name)
        if not os.path.isdir(map_path):
            continue
            
        simulation_output_path = os.path.join(map_path, "simulation_output")
        if not os.path.exists(simulation_output_path):
            continue
            
        for scenario_name in os.listdir(simulation_output_path):
            scenario_path = os.path.join(simulation_output_path, scenario_name)
            if os.path.isdir(scenario_path):
                pairs.append((map_path, scenario_path))
    
    return pairs

def process_maps_parallel(test_output_dir: str, max_workers: int = None) -> None:
    """
    Process multiple map-scenario pairs in parallel using ProcessPoolExecutor.
    
    Args:
        test_output_dir (str): Directory containing map folders
        max_workers (int, optional): Number of worker processes. Defaults to CPU count - 1.
    """
    if max_workers is None:
        max_workers = max(1, multiprocessing.cpu_count() - 1)
    
    # Get list of map-scenario pairs
    pairs = get_map_scenario_pairs(test_output_dir)
    total_pairs = len(pairs)
    print(f"\nProcessing {total_pairs} map-scenario pairs using {max_workers} workers...")
    
    # Process pairs in parallel
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        # Submit all tasks
        futures = {executor.submit(test_visualizer, map_path, scenario_path): (map_path, scenario_path) 
                  for map_path, scenario_path in pairs}
        
        # Process results with progress bar
        with tqdm(total=total_pairs, desc="Processing map-scenario pairs") as pbar:
            for future in as_completed(futures):
                map_path, scenario_path = futures[future]
                try:
                    result = future.result()
                    if not result:
                        print(f"Removing scenario folder: {scenario_path}")
                        # shutil.rmtree(scenario_path)
                except Exception as e:
                    print(f"Error processing map-scenario pair: {e}")
                pbar.update(1)

if __name__ == "__main__":
    # Get the test_output directory
    test_output_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "test_output")
    
    if not os.path.exists(test_output_dir):
        print(f"Error: {test_output_dir} does not exist")
        sys.exit(1)
    
    # Process all maps in parallel
    # process_maps_parallel(test_output_dir, max_workers=5)

    test_visualizer("/path/to/road/directory", "/path/to/scenario/directory")