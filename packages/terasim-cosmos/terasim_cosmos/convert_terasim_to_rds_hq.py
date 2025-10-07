from matplotlib import pyplot as plt
import numpy as np
import tensorflow as tf
import click
import imageio as imageio_v1

from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import List, Dict, Any, Union
from termcolor import cprint
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
from waymo_open_dataset import dataset_pb2, label_pb2
from waymo_open_dataset.utils import frame_utils
from google.protobuf import json_format
from .utils.wds_utils import write_to_tar, encode_dict_to_npz_bytes
from .utils.bbox_utils import interpolate_pose

import sumolib
import xml.etree.ElementTree as ET
from .utils.wds_utils import get_sample

from terasim_datazoo.processors.sumo2waymo import SUMO2Waymo

class TeraSim_Dataset:
    """
    Dataset class for TeraSim data that provides iteration over the last N timesteps.
    Implements similar interface as Waymo dataset for compatibility.
    """
    def __init__(self, terasim_record_root: Union[str, Path], path_to_fcd: Union[str, Path], path_to_map: Union[str, Path], time_start: float, time_end: float, av_id: str = "CAV", agent_clip_distance: float = 100.0):
        """
        Initialize the TeraSim dataset.
        
        Args:
            terasim_record_root: Path to the root directory containing TeraSim data
            time_start: Start time
            time_end: End time 
            av_id: ID of the AV vehicle (default: "CAV")
        """
        self.clip_id = terasim_record_root.stem
        self.sumo_net_path = path_to_map
        if not self.sumo_net_path.exists():
            raise FileNotFoundError(f"Map file not found: {self.sumo_net_path}")
        self.sumo_net = sumolib.net.readNet(self.sumo_net_path, withInternal=True, withPedestrianConnections=True)
        self.fcd_path = path_to_fcd
        if not self.fcd_path.exists():
            raise FileNotFoundError(f"FCD file not found: {self.fcd_path}")
        self.av_id = av_id
        
        # Parse XML data
        self.fcd_data = ET.parse(self.fcd_path).getroot()
        
        # Get all timesteps and keep only the last N
        all_timesteps = self.fcd_data.findall('timestep')
        # Convert time to timestep indices (assuming 0.1s per timestep)
        timestep_start = int(time_start * 10) if time_start >= 0 else 0
        timestep_end_idx = int(time_end * 10) if time_end >= 0 else len(all_timesteps)
        stop = None if time_end == -1 else timestep_end_idx + 1
        self.timesteps = all_timesteps[timestep_start:stop]
        self.current_idx = 0
        
        # Store the first and last timestamp for debugging
        self.start_time = float(self.timesteps[0].get('time'))
        self.end_time = float(self.timesteps[-1].get('time'))
        print(f"Dataset loaded: time range [{self.start_time}, {self.end_time}], {len(self.timesteps)} frames")
        self.agent_clip_distance = agent_clip_distance
        
    def __iter__(self):
        """Reset iterator index and return self"""
        self.current_idx = 0
        return self
        
    def __next__(self):
        """
        Get next frame data in format similar to Waymo dataset.
        Raises StopIteration when no more frames are available.
        """
        if self.current_idx >= len(self.timesteps):
            raise StopIteration
            
        timestep = self.timesteps[self.current_idx]
        self.current_idx += 1
        
        # Create frame data in a format similar to Waymo dataset
        frame_data = {
            'timestamp': float(timestep.get('time')),
            'vehicle_pose': self._get_vehicle_pose(timestep),
            'all_agent_bbox': self._get_all_agent_bbox(timestep),
        }
        
        return frame_data
    
    def _get_all_agent_bbox(self, timestep):
        """
        Get all agent bbox from timestep data.
        """
        all_agent_bbox = {}
        # iterate all agents in the timestep
        for agent in timestep.findall('vehicle'):
            agent_id = agent.get('id')
            if agent_id == self.av_id:
                continue
            agent_bbox = self._get_agent_bbox(timestep, agent_id)
            if agent_bbox is not None:
                all_agent_bbox[agent_id] = agent_bbox
        for agent in timestep.findall('person'):
            agent_id = agent.get('id')
            agent_bbox = self._get_agent_bbox(timestep, agent_id)
            if agent_bbox is not None:
                all_agent_bbox[agent_id] = agent_bbox
        return all_agent_bbox
    
    def _get_agent_bbox(self, timestep, agent_id):
        """
        Get agent bbox from timestep data, similar to Waymo bbox format.

        Args:
            timestep: XML element for the current timestep
            agent_id: string, the id of the agent (vehicle, pedestrian, etc.)

        Returns:
            dict: {
                'object_to_world': 4x4 list, transformation matrix in world coordinates,
                'object_lwh': [length, width, height],
                'object_is_moving': bool,
                'object_type': str
            }
        """
        # Find the agent in the current timestep
        agent = timestep.find(f"*[@id='{agent_id}']") # find both vehicle and person
        if agent is None:
            raise ValueError(f"Cannot find agent with id {agent_id}")

        # Extract position, orientation, and size
        x = float(agent.get('x'))
        y = float(agent.get('y'))
        z = float(agent.get('z', 0.0))

        av_agent = timestep.find(f"vehicle[@id='{self.av_id}']")
        av_x = float(av_agent.get('x'))
        av_y = float(av_agent.get('y'))
        av_z = float(av_agent.get('z', 0.0))

        av_distance = np.sqrt((x - av_x)**2 + (y - av_y)**2)
        if av_distance > self.agent_clip_distance:
            return None

        # Get object type from agent tag and type attribute
        agent_tag = agent.tag  # 'vehicle' or 'person'
        agent_type = agent.get('type', '')  # Get type attribute if exists
        
        if agent_tag == 'person':
            object_type = 'Pedestrian'
        elif 'bike' in agent_type.lower():
            object_type = 'Cyclist' 
        else:
            object_type = 'Car'  # Default type for vehicles

        bbox_type_size_dict = {
            'Car': [5.0, 1.85, 1.5],
            'Cyclist': [1.8, 0.6, 1.7],
            'Pedestrian': [0.5, 0.5, 1.7]
        }

        length, width, height = bbox_type_size_dict[object_type]
        
        sumo_angle = float(agent.get('angle'))  # Angle in degrees, SUMO: 0=north, 90=east

        # Convert SUMO position (front bumper) to Waymo position (rear axle)
        # x, y, z = self.convert_sumo_to_waymo_position(x, y, z, sumo_angle, length)

        # Convert SUMO angle to FLU convention
        # SUMO: 0=north, 90=east, 180=south, 270=west
        # FLU: x-forward, y-left, z-up
        # We need to convert from SUMO's heading angle to FLU's rotation angle
        heading = (90 - sumo_angle) % 360   # Convert to FLU convention
        heading_rad = np.radians(heading)

        # Build rotation matrix (FLU: x-forward, y-left, z-up)
        rotation = np.eye(4)
        rotation[0:3, 0:3] = np.array([
            [np.cos(heading_rad), -np.sin(heading_rad), 0],
            [np.sin(heading_rad),  np.cos(heading_rad), 0],
            [0,                 0,                 1]
        ])

        

        x, y, z = self.convert_sumo_to_waymo_bbox_center(x, y, z, sumo_angle, length, width, height)

        # Set translation (FLU: x = north, y = -east)
        translation = np.eye(4)
        translation[0, 3] = x      # SUMO x (east) -> FLU x
        translation[1, 3] = y     # SUMO y (north) -> FLU y
        translation[2, 3] = z      # Now at rear axle height

        # Combine rotation and translation
        object_to_world = translation @ rotation

        # Get speed (if available), otherwise set to 0
        speed = float(agent.get('speed', 0.0))
        min_moving_speed = 0.2
        object_is_moving = bool(speed > min_moving_speed)

        # Pack results
        bbox_info = {
            'object_to_world': object_to_world.tolist(),
            'object_lwh': [length, width, height],
            'object_is_moving': object_is_moving,
            'object_type': object_type
        }

        return bbox_info
    
    def _get_vehicle_pose(self, timestep) -> np.ndarray:
        """
        Extract vehicle pose from timestep data and convert to 4x4 transformation matrix.
        
        Args:
            timestep: XML element containing vehicle data for current timestep
            
        Returns:
            np.ndarray: 4x4 transformation matrix in world coordinate system (x=east, y=north)
            
        Raises:
            ValueError: If vehicle with specified ID is not found
        """
        # Find vehicle element in current timestep
        vehicle = timestep.find(f"vehicle[@id='{self.av_id}']")
        if vehicle is None:
            raise ValueError(f"Cannot find vehicle with id {self.av_id}")
            
        # Extract position and angle from XML
        x = float(vehicle.get('x'))
        y = float(vehicle.get('y'))
        z = float(vehicle.get('z', 0.0))
        sumo_angle = float(vehicle.get('angle'))  # Heading angle in degrees, SUMO: 0=north, 90=east
        length = float(vehicle.get('length', 4.5))  # Get vehicle length
        
        # Convert SUMO position (front bumper) to Waymo position (rear axle)
        x, y, z = self.convert_sumo_to_waymo_position(x, y, z, sumo_angle, length)
        
        # Create 4x4 transformation matrix
        pose = np.eye(4)
        
        # Convert SUMO heading angle to world coordinate system angle
        # SUMO: 0=north, 90=east, 180=south, 270=west
        # World: 0=east, 90=north, 180=west, 270=south
        heading = (90 - sumo_angle) % 360  # Convert to world coordinate system
        heading_rad = np.radians(heading)
        
        # Set rotation matrix (World: x=east, y=north)
        pose[0:2, 0:2] = np.array([
            [np.cos(heading_rad), -np.sin(heading_rad)],
            [np.sin(heading_rad), np.cos(heading_rad)]
        ])
        
        # Set translation (position)
        pose[0:2, 3] = [x, y]  # Keep SUMO coordinates (x=east, y=north)
        
        # Set z coordinate (now at rear axle height)
        pose[2, 3] = z
        return pose
    
    def __len__(self):
        """Return the number of timesteps in the dataset"""
        return len(self.timesteps)

    def convert_sumo_to_waymo_position(self, x: float, y: float, z: float, angle: float, length: float = 4.5) -> tuple:
        """
        Convert SUMO vehicle position (front bumper center) to Waymo position (rear axle center).
        
        Args:
            x: SUMO x coordinate (front bumper center)
            y: SUMO y coordinate (front bumper center)
            z: SUMO z coordinate (ground height)
            angle: Vehicle heading angle in degrees (SUMO convention: 0=north, 90=east)
            length: Vehicle length in meters (default: 4.5m for typical sedan)
            
        Returns:
            tuple: (x, y, z) coordinates in Waymo convention (rear axle center)
        """
        # Convert angle to radians
        heading = (90 - angle) % 360
        heading_rad = np.radians(heading)
        
        # Calculate offset from front bumper to rear axle
        # Typical sedan: wheelbase ~2.7m, front overhang ~0.9m
        # So rear axle is about (length/2 - front_overhang) meters behind front bumper
        # front_overhang = 0.9  # meters
        rear_axle_offset = 0.75 * length
        
        # Calculate rear axle position
        rear_x = x - rear_axle_offset * np.cos(heading_rad)
        rear_y = y - rear_axle_offset * np.sin(heading_rad)
        
        # Set rear axle height (typical sedan rear axle height)
        rear_z = z
        
        return rear_x, rear_y, rear_z
    
    def convert_sumo_to_waymo_bbox_center(self, x: float, y: float, z: float, sumo_angle: float, length: float, width: float, height: float) -> tuple:
        """
        Convert SUMO vehicle position (front bumper center) to Waymo position (rear axle center).
        
        Args:
            x: SUMO x coordinate (front bumper center)
            y: SUMO y coordinate (front bumper center)
            z: SUMO z coordinate (ground height)
            angle: Vehicle heading angle in degrees (SUMO convention: 0=north, 90=east)
            length: Vehicle length in meters (default: 4.5m for typical sedan)
            width: Vehicle width in meters (default: 2.0m for typical sedan)
            height: Vehicle height in meters (default: 1.5m for typical sedan)

        Returns:
            tuple: (x, y, z) coordinates in Waymo convention (rear axle center)
        """
        # Calculate offset from front bumper to vehicle center
        offset_x = length / 2
        heading = (90 - sumo_angle) % 360
        heading_rad = np.radians(heading)

        # Calculate vehicle center position
        center_x = x - offset_x * np.cos(heading_rad)
        center_y = y - offset_x * np.sin(heading_rad)
        center_z = z + height / 2

        return center_x, center_y, center_z
        

WaymoProto2SemanticLabel = {
    label_pb2.Label.Type.TYPE_UNKNOWN: "Unknown",
    label_pb2.Label.Type.TYPE_VEHICLE: "Car",
    label_pb2.Label.Type.TYPE_PEDESTRIAN: "Pedestrian",
    label_pb2.Label.Type.TYPE_SIGN: "Sign",
    label_pb2.Label.Type.TYPE_CYCLIST: "Cyclist",
}

SourceFps = 10 # waymo's recording fps
TargetFps = 30 # cosmos's expected fps
IndexScaleRatio = int(TargetFps / SourceFps)

if int(tf.__version__.split(".")[0]) < 2:
    tf.enable_eager_execution()

# make sure the GPU memory is not exhausted
physical_devices = tf.config.list_physical_devices('GPU')
if physical_devices:
    try:
        for device in physical_devices:
            tf.config.experimental.set_memory_growth(device, True)
    except Exception as e:
        print(e)

def convert_terasim_intrinsics(output_root: Path, clip_id: str, dataset: tf.data.TFRecordDataset):
    """
    Use the camera's intrinsics in Waymo openmotion dataset

    Minimal required format:
        sample['pinhole_intrinsic.{camera_name}.npy'] = np.ndarray with shape (4, 4)
    """
    sample = {'__key__': clip_id}

    example_id = "10017090168044687777_6380_000_6400_000"
    package_root = Path(__file__).parent
    npy_path = package_root / f"assets/waymo_pinhole_intrinsic_{example_id}.npy"
    example_sample = np.load(npy_path, allow_pickle=True).item()
    CameraNames = ['front', 'front_left', 'front_right', 'side_left', 'side_right']
    for camera_name in CameraNames:
        sample[f'pinhole_intrinsic.{camera_name}.npy'] = example_sample[f'pinhole_intrinsic.{camera_name}.npy']
    
    # sample[f'pinhole_intrinsic.front.npy'] = np.array([2044.50, 2044.50, 949.57, 633.24, 1920, 1280])
    # sample['pinhole_intrinsic.front_left.npy'] = np.array([2046.63, 2046.63, 975.06, 640.91, 1920, 1280])
    # sample['pinhole_intrinsic.front_right.npy'] = np.array([2053.54, 2053.54, 944.36, 630.65, 1920, 1280])
    # sample['pinhole_intrinsic.side_left.npy'] = np.array([2046.63, 2046.63, 975.06, 640.91, 1920, 1280])
    # sample['pinhole_intrinsic.side_right.npy'] = np.array([2050.25, 2050.25, 970.31, 248.14, 1920, 1280])
    write_to_tar(sample, output_root / 'pinhole_intrinsic' / f'{clip_id}.tar')
    return

def convert_terasim_hdmap(output_root: Path, clip_id: str, dataset: TeraSim_Dataset, map_clip_distance: float = 100.0):
    """
    Convert TeraSim map data to RDS-HQ format.
    
    Args:
        output_root: Output directory path
        clip_id: Clip ID
        dataset: TeraSim dataset object
    """
    def hump_to_underline(hump_str):
        import re
        return re.sub(r'([a-z])([A-Z])', r'\1_\2', hump_str).lower()
    
    sumo2waymo = SUMO2Waymo(dataset.sumo_net_path)
    sumo2waymo.parse(have_road_edges=True, have_road_lines=True)
    scenario = sumo2waymo.convert_to_scenario(scenario_id=clip_id)
    
    # Get AV position from the first frame
    dataset_iter = iter(dataset)
    first_frame = next(dataset_iter)
    av_position = first_frame['vehicle_pose'][:3, 3]  # Extract translation [x, y, z]

    # print(f"AV position in first frame: [{av_position[0]:.2f}, {av_position[1]:.2f}, {av_position[2]:.2f}]")

    hdmap_names_polyline = ["lane", "road_line", "road_edge"]
    hdmap_names_polygon = ["crosswalk", "speed_bump", "driveway"]
    
    hdmap_name_to_data = {}
    for hdmap_name in hdmap_names_polyline + hdmap_names_polygon:
        hdmap_name_to_data[hump_to_underline(hdmap_name)] = []


    map_features_list = json_format.MessageToDict(scenario)['mapFeatures']

    for hdmap_content in map_features_list:
        hdmap_name = list(hdmap_content.keys())
        hdmap_name.remove("id")
        hdmap_name = hdmap_name[0]
        hdmap_name_lower = hump_to_underline(hdmap_name)

        hdmap_data = hdmap_content[hdmap_name]
        if hdmap_name_lower in hdmap_names_polyline:
            try:
                hdmap_data = hdmap_data['polyline']
                polyline = [[point['x'], point['y'], point['z']] for point in hdmap_data]
                
                # Filter polyline based on distance to AV
                filtered_polyline = []
                for point in polyline:
                    distance = np.sqrt((point[0] - av_position[0])**2 + (point[1] - av_position[1])**2)
                    if distance <= map_clip_distance:
                        filtered_polyline.append(point)
                
                # Only add polyline if it has points within distance threshold
                if filtered_polyline:
                    hdmap_name_to_data[hdmap_name_lower].append(filtered_polyline)
            except:
                print(f"Unkown hdmap item name: {hdmap_name}, skip this item")
        elif hdmap_name_lower in hdmap_names_polygon:
            hdmap_data = hdmap_data['polygon']
            polygon = [[point['x'], point['y'], point['z']] for point in hdmap_data]
            
            # Filter polygon based on distance to AV
            filtered_polygon = []
            for point in polygon:
                distance = np.sqrt((point[0] - av_position[0])**2 + (point[1] - av_position[1])**2)
                if distance <= map_clip_distance:
                    filtered_polygon.append(point)
            
            # Only add polygon if it has points within distance threshold
            if filtered_polygon:
                hdmap_name_to_data[hdmap_name_lower].append(filtered_polygon)
        else:
            print(f"Unkown hdmap item name: {hdmap_name}, skip this item")

    # convert to cosmos's name convention for easier processing
    hdmap_name_to_cosmos = {
        'lane': 'lanes',
        'road_line': 'lanelines',
        'road_edge': 'road_boundaries',
        'crosswalk': 'crosswalks',
        'speed_bump': None,
        'driveway': None
    }

    for hdmap_name, hdmap_data in hdmap_name_to_data.items():
        hdmap_name_in_cosmos = hdmap_name_to_cosmos[hdmap_name]
        if hdmap_name_in_cosmos is None:
            continue

        if hdmap_name in hdmap_names_polyline:
            vertex_indicator = 'polyline3d'
        else:
            vertex_indicator = 'surface'

        # to match cosmos format, the easiest way is to add 'vertices' key for the polyline or polygon
        sample = {'__key__': clip_id, f'{hdmap_name_in_cosmos}.json': {'labels': []}}

        for each_polyline_or_polygon in hdmap_data:
            sample[f'{hdmap_name_in_cosmos}.json']['labels'].append({
                'labelData': {
                    'shape3d': {
                        vertex_indicator: {
                            'vertices': each_polyline_or_polygon
                        }
                    }
                }
            })

        write_to_tar(sample, output_root / f'3d_{hdmap_name_in_cosmos}' / f'{clip_id}.tar')

def convert_terasim_pose(output_root: Path, clip_id: str, dataset: TeraSim_Dataset, camera_setting_name: str = "default"):
    """
    read all frames and convert the pose to wds format. interpolate the pose to the target fps

    Minimal required format:
        sample_camera_to_world['{frame_idx:06d}.pose.{camera_name}.npy'] = np.ndarray with shape (4, 4). opencv convention
        sample_vehicle_to_world['{frame_idx:06d}.vehicle_pose.npy'] = np.ndarray with shape (4, 4). world coordinate system
    """
    sample_camera_to_world = {'__key__': clip_id}
    sample_vehicle_to_world = {'__key__': clip_id}

    if camera_setting_name == "waymo":
        package_root = Path(__file__).parent
        CameraNames = ['front', 'front_left', 'front_right', 'side_left', 'side_right']
        example_id = "10017090168044687777_6380_000_6400_000"
        npy_path = package_root / f"assets/waymo_camera_name_to_camera_to_vehicle_{example_id}.npy"
        camera_name_to_camera_to_vehicle = np.load(npy_path, allow_pickle=True).item()

    elif camera_setting_name == "default":
        import json
        camera_name_to_camera_to_vehicle = {}
        package_root = Path(__file__).parent
        settings = json.load(open(package_root / "config/dataset_rds_hq_mv_terasim.json", "r"))
        CameraNames = settings["CAMERAS"]
        example_id = "2d23a1f4-c269-46aa-8e7d-1bb595d1e421_2445376400000_2445396400000"
        time_step = 0
        path_to_pose = package_root / "assets/example/pose" / f"{example_id}.tar"
        path_to_vehicle_pose = package_root / "assets/example/vehicle_pose" / f"{example_id}.tar"
        pose = get_sample(path_to_pose)
        vehicle_pose = get_sample(path_to_vehicle_pose)
        vehicle_to_world = vehicle_pose[f"{time_step:06d}.vehicle_pose.npy"]
        for camera_name in CameraNames:
            camera_to_world = pose[f"{time_step:06d}.pose.{camera_name}.npy"]
            # camera to world is openCV convention, we need to convert it to FLU
            transform_matrix = np.array([
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1]])
            camera_to_world = camera_to_world @ transform_matrix
            world_to_vehicle = np.linalg.inv(vehicle_to_world)
            camera_to_vehicle = world_to_vehicle @ camera_to_world
            camera_name_to_camera_to_vehicle[camera_name] = camera_to_vehicle
            # print the yaw angle of the camera which is used for street view analysis
            # rotation = camera_to_vehicle[:3, :3]
            # yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
            # print(f"Yaw angle of {camera_name}: {np.rad2deg(yaw)}")
    else:
        raise ValueError(f"Invalid camera setting name: {camera_setting_name}")

    
    # Process each frame
    for frame_idx, frame_data in enumerate(dataset):
        # Get vehicle pose in world coordinate (x=east, y=north)
        vehicle_to_world = frame_data['vehicle_pose']
        for camera_name in CameraNames:
            camera_to_vehicle = camera_name_to_camera_to_vehicle[camera_name]
            camera_to_world = vehicle_to_world @ camera_to_vehicle
            # Convert camera pose to OpenCV coordinate convention (FLU)
            camera_to_world_opencv = np.concatenate(
                [-camera_to_world[:, 1:2], -camera_to_world[:, 2:3], camera_to_world[:, 0:1], camera_to_world[:, 3:4]],
                axis=1
            )
            
            # Store poses
            sample_camera_to_world[f"{frame_idx * IndexScaleRatio:06d}.pose.{camera_name}.npy"] = camera_to_world_opencv
            sample_vehicle_to_world[f"{frame_idx * IndexScaleRatio:06d}.vehicle_pose.npy"] = vehicle_to_world

    # interpolate the pose to the target fps
    # source index: 0,    1,    2,    3, ..., 10
    # target index: 0,1,2,3,4,5,6,7,8,9, ..., 30,31,32
    max_target_frame_idx = frame_idx * IndexScaleRatio

    # interpolate the vehicle pose to the target fps
    for target_frame_idx in range(max_target_frame_idx):
        if f"{target_frame_idx:06d}.vehicle_pose.npy" not in sample_vehicle_to_world:
            nearest_prev_frame_idx = target_frame_idx // IndexScaleRatio * IndexScaleRatio
            nearest_prev_frame_pose = sample_vehicle_to_world[f"{nearest_prev_frame_idx:06d}.vehicle_pose.npy"]
            nearest_next_frame_idx = (target_frame_idx // IndexScaleRatio + 1) * IndexScaleRatio
            nearest_next_frame_pose = sample_vehicle_to_world[f"{nearest_next_frame_idx:06d}.vehicle_pose.npy"]
            sample_vehicle_to_world[f"{target_frame_idx:06d}.vehicle_pose.npy"] = \
                interpolate_pose(nearest_prev_frame_pose, nearest_next_frame_pose, (target_frame_idx - nearest_prev_frame_idx) / IndexScaleRatio)

    # add the last two frames
    approx_motion = sample_vehicle_to_world[f"{max_target_frame_idx:06d}.vehicle_pose.npy"] - sample_vehicle_to_world[f"{max_target_frame_idx - 1:06d}.vehicle_pose.npy"]
    approx_motion[:3, :3] = 0
    sample_vehicle_to_world[f"{max_target_frame_idx + 1:06d}.vehicle_pose.npy"] = sample_vehicle_to_world[f"{max_target_frame_idx:06d}.vehicle_pose.npy"] + approx_motion
    sample_vehicle_to_world[f"{max_target_frame_idx + 2:06d}.vehicle_pose.npy"] = sample_vehicle_to_world[f"{max_target_frame_idx:06d}.vehicle_pose.npy"] + 2 * approx_motion

    # interpolate the camera pose to the target fps
    for camera_name in CameraNames:
        for target_frame_idx in range(max_target_frame_idx):
            if f"{target_frame_idx:06d}.pose.{camera_name}.npy" not in sample_camera_to_world:
                nearest_prev_frame_idx = target_frame_idx // IndexScaleRatio * IndexScaleRatio
                nearest_prev_frame_pose = sample_camera_to_world[f"{nearest_prev_frame_idx:06d}.pose.{camera_name}.npy"]
                nearest_next_frame_idx = (target_frame_idx // IndexScaleRatio + 1) * IndexScaleRatio
                nearest_next_frame_pose = sample_camera_to_world[f"{nearest_next_frame_idx:06d}.pose.{camera_name}.npy"]
                sample_camera_to_world[f"{target_frame_idx:06d}.pose.{camera_name}.npy"] = \
                    interpolate_pose(nearest_prev_frame_pose, nearest_next_frame_pose, (target_frame_idx - nearest_prev_frame_idx) / IndexScaleRatio)

        # add the last two frames
        approx_motion = sample_camera_to_world[f"{max_target_frame_idx:06d}.pose.{camera_name}.npy"] - sample_camera_to_world[f"{max_target_frame_idx - 1:06d}.pose.{camera_name}.npy"]
        approx_motion[:3, :3]  = 0
        sample_camera_to_world[f"{max_target_frame_idx + 1:06d}.pose.{camera_name}.npy"] = sample_camera_to_world[f"{max_target_frame_idx:06d}.pose.{camera_name}.npy"] + approx_motion
        sample_camera_to_world[f"{max_target_frame_idx + 2:06d}.pose.{camera_name}.npy"] = sample_camera_to_world[f"{max_target_frame_idx:06d}.pose.{camera_name}.npy"] + 2 * approx_motion

    write_to_tar(sample_camera_to_world, output_root / 'pose' / f'{clip_id}.tar')
    write_to_tar(sample_vehicle_to_world, output_root / 'vehicle_pose' / f'{clip_id}.tar')


def convert_terasim_bbox(output_root: Path, clip_id: str, dataset: TeraSim_Dataset):
    """
    Read all frames and convert the bbox to wds format for TeraSim dataset.

    Minimal required format:
        sample['{frame_idx:06d}.all_object_info.json'] = {
            'object_id 1' : {
                'object_to_world' : np.ndarray with shape (4, 4),
                'object_lwh' : np.ndarray with shape (3,),
                'object_is_moving' : bool,
                'object_type' : str
            },
            ...
        }
    """
    sample = {'__key__': clip_id}

    for frame_idx, frame_data in enumerate(dataset):
        # Get all agent bbox info for this frame
        all_agent_bbox = frame_data['all_agent_bbox']
        sample[f"{frame_idx * IndexScaleRatio:06d}.all_object_info.json"] = {}

        for object_id, bbox_info in all_agent_bbox.items():
            # Directly use the bbox_info dict from TeraSim_Dataset
            sample[f"{frame_idx * IndexScaleRatio:06d}.all_object_info.json"][object_id] = {
                'object_to_world': bbox_info['object_to_world'],
                'object_lwh': bbox_info['object_lwh'],
                'object_is_moving': bbox_info['object_is_moving'],
                'object_type': bbox_info['object_type']
            }

    # Write all results to tar file
    write_to_tar(sample, output_root / 'all_object_info' / f'{clip_id}.tar')

def convert_terasim_to_wds(
    terasim_record_root: Union[str, Path],
    path_to_fcd: Union[str, Path],
    path_to_map: Union[str, Path],
    output_wds_path: Union[str, Path],
    single_camera: bool = False,
    camera_setting_name: str = "default",
    av_id: str = "CAV",
    time_start: float = -12.0,
    time_end: float = -1,
    agent_clip_distance: float = 100.0,
    map_clip_distance: float = 100.0
):
    terasim_record_path = Path(terasim_record_root)
    clip_id = terasim_record_path.stem
    output_wds_path = Path(output_wds_path)

    if not terasim_record_path.exists():
        raise FileNotFoundError(f"Terasim record file not found: {terasim_record_path}")
    
    dataset = TeraSim_Dataset(terasim_record_root, path_to_fcd, path_to_map, time_start, time_end, av_id=av_id, agent_clip_distance=agent_clip_distance)

    convert_terasim_pose(output_wds_path, clip_id, dataset, camera_setting_name)
    convert_terasim_hdmap(output_wds_path, clip_id, dataset, map_clip_distance)
    convert_terasim_bbox(output_wds_path, clip_id, dataset)
    convert_terasim_intrinsics(output_wds_path, clip_id, dataset)
    

