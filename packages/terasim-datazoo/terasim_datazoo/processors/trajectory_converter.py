import os
import numpy as np
import xml.etree.ElementTree as ET
from waymo_open_dataset.protos import scenario_pb2



class TrajectoryConverter:
    """
    Class to convert trajectory output to Waymax format.

    """

    def __init__(self, traj_file: str):
        """
        Initialize the TrajectoryConverter with a trajectory file.

        Args:
            traj_file (str): Path to the trajectory file.
            start_at (int): The timestamp (in microseconds) to start parsing the trajectory.
            simulation_length (int): The length of the simulation (in microseconds).
        """
        assert os.path.exists(traj_file), f"Trajectory file {traj_file} does not exist."
        assert traj_file.endswith('.xml'), "Trajectory file must be in XML format."
        self.traj_file = traj_file
        self.obj_dict = {} # Dictionary to store object data
        self.ids = [] # List to store object ids
        # self.max_num_objects = max_num_objects


    def _load_trajectory(self):
        # Load the XML file
        try:
            self.trajectory_tree = ET.parse(self.traj_file)
        except ET.ParseError as e:
            raise ValueError(f"Error loading XML file {self.traj_file}: {e}")
        self.root = self.trajectory_tree.getroot()


    def parse(self, start_at=None, traj_length=None):
        # Load the trajectory file if not already loaded
        if not hasattr(self, 'trajectory_tree'):
            self._load_trajectory()
        
        self.start_at = start_at
        self.traj_length = traj_length
        self.end_at = self.start_at + self.traj_length
        # Init time & metadata: how many, their types, etc.
        # Construct a object_state.ObjectMetadata object
        # Shape should be (..., num_objects)
        """ Attributes to be extracted from XML tree:
            ids: A unique integer id for each object which is consistent over time of data type int32.
            object_types: An integer representing each different class of object 
            (Unset=0, Vehicle=1, Pedestrian=2, Cyclist=3, Other=4) of data type int32. 
            This definition is from Waymo Open Motion Dataset (WOMD).
            is_sdc: Binary mask of data type bool representing whether an object 
            represents the sdc or some other object.
            is_modeled: Whether a specific object is one designated by WOMD to be 
            predicted of data type bool.
            is_valid: Whether an object is valid at any part of the run segment of data type bool.
            objects_of_interest: A vector of type bool to indicate which objects in the 
            scene corresponding to the first dimension of the object tensors have 
            interactive behavior. Up to 2 objects will be selected. The objects in 
            this list form an interactive group.
            is_controlled: Whether an object will be controlled by external agents in an 
            environment.
        """

        time_steps = []        # find all timestep_micros
        object_index = 0
        for timestep in self.root.findall('timestep'):
            timestamp = float(timestep.get('time', -1))
            if timestamp < self.start_at:
                # print(f"skipping timestep {timestamp_micro} at {timestamp_micro} microseconds")
                continue
            elif timestamp == self.start_at:
                # scan for vehicle only when it is start_at
                for vehicle in timestep.findall('vehicle'):
                    # Extract attributes from the vehicle element
                    vehicle_id = vehicle.get('id', "n/a")

                    if vehicle_id == "n/a":
                        print("Warning: Vehicle without id found, skipping.")
                        continue

                    vehicle_type = vehicle.get('type', "UNSET")  # Default to UNSET if type not found
                    # Map vehicle type to ObjectTypeIds enum
                    # we need to convert the string to upper case to match the enum
                    if 'veh' in vehicle_type.lower():
                        waymo_obj_type = scenario_pb2.Track.ObjectType.TYPE_VEHICLE
                    elif 'ped' in vehicle_type.lower():
                        waymo_obj_type = scenario_pb2.Track.ObjectType.TYPE_PEDESTRIAN
                    elif 'cyc' in vehicle_type.lower():
                        waymo_obj_type = scenario_pb2.Track.ObjectType.TYPE_CYCLIST
                    else:
                        waymo_obj_type = scenario_pb2.Track.ObjectType.TYPE_OTHER
                        print(f"Warning: Unknown vehicle type {vehicle_type}, setting to TYPE_OTHER.")

                    if vehicle_id not in self.obj_dict:
                        print(f"{timestamp} adding vehicle {vehicle_id} to obj_dict")
                        self.obj_dict[vehicle_id] = {
                            'id': vehicle_id,
                            'index': object_index,
                            'object_type': waymo_obj_type,
                            'is_sdc': False,  # Default to False, can be updated later
                            'is_modeled': False,  # Default to False, can be updated later
                            'is_valid': False,  # Default to True, can be updated later
                            'objects_of_interest': np.zeros(1, dtype=bool),  # Default to no objects of interest
                            'is_controlled': False  # Default to False, can be updated later
                        }
                        object_index += 1
            elif timestamp >= self.end_at:
                # print(f"Reached max number of timesteps: {self.end_at}.")
                break

            time_steps.append(timestamp)

        # check if time_steps is empty
        if not time_steps:
            raise ValueError("No timesteps found in the trajectory file.")
        
        # sort time_steps, all timestamps should be in ascending order from the XML file
        time_steps = sorted(time_steps)

        # check and print the number of time steps, and range of timestamps
        num_timesteps = len(time_steps)
        # num_timesteps = self.max_num_timesteps
        if num_timesteps == 0:
            raise ValueError("No valid timesteps found in the trajectory file.")
        print(f"Number of timesteps: {num_timesteps}")
        print(f"Timestamp range: {time_steps[0]} - {time_steps[-1]}")

        # save timestamps in the trajectory data
        # reshape to (num_objects, num_timesteps)
        self.timestamps = np.tile(
            np.array(time_steps, dtype=np.float32)[np.newaxis, :],
            (len(self.obj_dict), 1)
        )

        # Check if any objects were found
        if not self.obj_dict:
            raise ValueError("No vehicles found in the trajectory file.")
        
        # print a summary of the objects found
        print(f"Found {len(self.obj_dict)} unique vehicles in the trajectory file.")
        
        # make sure ids are unique
        self.ids = list(self.obj_dict.keys())
        if len(self.ids) != len(set(self.ids)):
            raise ValueError("Duplicate vehicle ids found in the trajectory file.")

        # pre-initialize the trajectory data
        # set positions, vertices, and yaw of all objects at all time steps to -1
        # set all data points to invalid
        # size of objects set to default values: width=180 cm, length=550 cm, height=150 cm
        
        num_objects = len(self.obj_dict)
        # Initialize the trajectory data with default values
        self.trajectory_data = {
            'x': np.full((num_objects, num_timesteps), -float('inf'), dtype=np.float32),
            'y': np.full((num_objects, num_timesteps), -float('inf'), dtype=np.float32),
            'z': np.full((num_objects, num_timesteps), -float('inf'), dtype=np.float32),
            'vel_x': np.full((num_objects, num_timesteps), -float('inf'), dtype=np.float32),
            'vel_y': np.full((num_objects, num_timesteps), -float('inf'), dtype=np.float32),
            'yaw': np.full((num_objects, num_timesteps), -float('inf'), dtype=np.float32),
            'valid': np.full((num_objects, num_timesteps), False, dtype=bool), # Default to True, can be updated later
            'timestamps': self.timestamps,
            'length': np.full((num_objects, num_timesteps), 5.5, dtype=np.float32),  # Default length in meters
            'width': np.full((num_objects, num_timesteps), 1.8, dtype=np.float32),  # Default width in meters
            'height': np.full((num_objects, num_timesteps), 1.5, dtype=np.float32)  # Default height in meters
        }

        # Read x, y, z coordinates and other attributes from the XML
        # Construct a object_state.Trajectory object
        # Shape should be (..., num_objects, num_timesteps)
        """  Attributes to be extracted from XML tree:
            x: The x coordinate of each object at each time step of data type float32.
            y: The y coordinate of each object at each time step of data type float32.
            z: The z coordinate of each object at each time step of data type float32.
            vel_x: The x component of the object velocity at each time step of data type float32.
            vel_y: The y component of the object velocity at each time step of data type float32.
            yaw: Counter-clockwise yaw in top-down view (rotation about the Z axis 
                from a unit X vector to the object direction vector) of shape of data type float32.
            valid: Validity bit for all object at all times steps of data type bool.
            timestamps: A timestamp for each time step of data type float32.
            length: The length of each object at each time step of data type float32.
                    Note for each object, its length is fixed for all time steps.
            width: The width of each object at each time step of data type float32. Note
                    for each object, its width is fixed for all time steps.
            height: The height of each object at each time step of data type float32.
                    Note for each object, its height is fixed for all time steps.
        """
        # first locate the timestep elements
        for timestep in self.root.findall('timestep'):
            # Get the index of the current timestep
            timestamp = float(timestep.get('time', -1))
            try: 
                timestep_index = time_steps.index(timestamp)
            except:
                continue
            if timestamp >= self.end_at:
                print(f"Reached the length of the trajectory: {self.end_at}.")
                break

            # Iterate through each vehicle in the current timestep
            for vehicle in timestep.findall('vehicle'):
                vehicle_id = vehicle.get('id', "n/a")
                if vehicle_id not in self.obj_dict:
                    # print(f"Skipping {vehicle_id} at time {timestamp_micro} since it didn't appear at the beginning of trajectory.")
                    continue
                # Get the index of the current vehicle
                vehicle_index = self.obj_dict[vehicle_id]['index']
                # Extract position, velocity, and yaw from the vehicle element
                x = float(vehicle.get('x', -float('inf')))
                y = float(vehicle.get('y', -float('inf')))
                z = float(vehicle.get('z', -float('inf')))
                angle = float(vehicle.get('angle', -float('inf')))
                speed = float(vehicle.get('speed', -float('inf')))
                # acceleration = float(vehicle.get('acceleration', -float('inf')))

                # convert SUMO yaw to Waymax yaw
                # SUMO：0 degree on north, clockwise, degrees
                # Waymax: 0 degree on east, counter-clockwise, radians

                angle = (450 - angle) % 360
                angle_rad = np.radians(angle)
                
                # infer velocity from speed and angle
                if speed >= 0 and angle >= 0:
                    vel_x = speed * np.cos(angle_rad)
                    vel_y = speed * np.sin(angle_rad)
                else:
                    vel_x = -1.0
                    vel_y = -1.0
                # Update the trajectory data for the current timestep and vehicle
                self.trajectory_data['x'][vehicle_index, timestep_index] = x
                self.trajectory_data['y'][vehicle_index, timestep_index] = y
                self.trajectory_data['z'][vehicle_index, timestep_index] = z
                self.trajectory_data['vel_x'][vehicle_index, timestep_index] = vel_x
                self.trajectory_data['vel_y'][vehicle_index, timestep_index] = vel_y
                self.trajectory_data['yaw'][vehicle_index, timestep_index] = angle_rad
                self.trajectory_data['valid'][vehicle_index, timestep_index] = True

                
        # show a summary of the trajectory data
        print(f"Parsed trajectory data shape: {self.trajectory_data['x'].shape}")
    
    def create_waymo_trajectory(self, scenario: scenario_pb2.Scenario):
        """
        Creates a trajectory object and the corresponding object metadata from the parsed trajectory data.
        """
        for vehicle_id, vehicle_data in self.obj_dict.items():
            track = scenario.tracks.add()
            vehicle_index = vehicle_data['index']
            track.id = vehicle_index
            track.object_type = vehicle_data['object_type']

            for timestep in range(self.timestamps.shape[1]):
                ts = track.states.add()
                ts.center_x = self.trajectory_data['x'][vehicle_index, timestep]
                ts.center_y = self.trajectory_data['y'][vehicle_index, timestep]
                ts.center_z = self.trajectory_data['z'][vehicle_index, timestep]
                ts.velocity_x = self.trajectory_data['vel_x'][vehicle_index, timestep]
                ts.velocity_y = self.trajectory_data['vel_y'][vehicle_index, timestep]
                ts.heading = self.trajectory_data['yaw'][vehicle_index, timestep]
                ts.length = self.trajectory_data['length'][vehicle_index, timestep]
                ts.width = self.trajectory_data['width'][vehicle_index, timestep]
                ts.height = self.trajectory_data['height'][vehicle_index, timestep]
                ts.valid = self.trajectory_data['valid'][vehicle_index, timestep]
        return scenario
    
    def create_waymax_trajectory(self):
        """
        Creates a trajectory object and the corresponding object metadata from the parsed trajectory data.

        Inputs:
            start_at (int): Number of warmup steps to ignore.
            simulation_length (int): Length of the output trajectory.
        Returns:
            waymax.datatypes.trajectory.Trajectory: A trajectory object containing the parsed data.
            waymax.datatypes.object_state.ObjectMetadata: An object metadata object containing the parsed data.
        """
        from waymax.datatypes import object_state, roadgraph, traffic_lights
        from waymax.datatypes.object_state import ObjectTypeIds
        import jax.numpy as jnp

        # Check if trajectory data is available
        if not hasattr(self, 'trajectory_data'):
            raise ValueError("Trajectory data not parsed. Please call _parse_trajectory() first.")

        # Limit the number of objects
        # self.max_num_objects = min(len(self.obj_dict), self.max_num_objects)

        # Apply start_at to slice the trajectory data
        # start = self.start_at
        # end = self.max_num_timesteps
        # sliced_data = {key: value[:, start:end] for key, value in self.trajectory_data.items()}
        sliced_data = self.trajectory_data.copy()
        # ensure sliced timesteps starts from 0
        sliced_data['timestamp_micros'] = sliced_data['timestamp_micros'] - sliced_data['timestamp_micros'][0]
        # if filter_sdc_neighbors and sdc_id in self.ids:
        #     sdc_idx = self.ids.index(sdc_id)
        #     # Use distance to filter neighbors, calculate distance at initial step
        #     distance = np.sqrt(
        #         (sliced_data['x'][:, init_steps-1] - sliced_data['x'][sdc_idx, init_steps-1]) ** 2 +
        #         (sliced_data['y'][:, init_steps-1] - sliced_data['y'][sdc_idx, init_steps-1]) ** 2
        #     )
        #     # Filter objects based on distance
        #     sorted_indices = np.argsort(distance, axis=0)
        #     idx_selection_range = sorted_indices[:self.max_num_objects] # Sorting by distance. we can always keep sdc in the first place
        #     selected_indices = np.where(distance < neighbor_radius)[0]

        # else:
        #     idx_selection_range = np.arange(self.max_num_objects)
        #     selected_indices = np.arange(self.max_num_objects)
        #     # if sdc is not in the selected indices, replace the first index with sdc
        #     if sdc_id in self.ids:
        #         sdc_idx = self.ids.index(sdc_id)
        #         if sdc_idx not in selected_indices:
        #             selected_indices[0] = sdc_idx
        #             idx_selection_range[0] = sdc_idx

        # print(selected_indices)
        # print(distance)
        # selected_indices = np.arange(self.max_num_objects)
        # idx_selection_range = np.arange(self.max_num_objects)
        # print(idx_selection_range)

        # Create a trajectory object with the modified data
        traj = object_state.Trajectory(
            x=sliced_data['x'],
            y=sliced_data['y'],
            z=sliced_data['z'],
            vel_x=sliced_data['vel_x'],
            vel_y=sliced_data['vel_y'],
            yaw=sliced_data['yaw'],
            valid=sliced_data['valid'],
            timestamp_micros=sliced_data['timestamp_micros'],
            length=sliced_data['length'],
            width=sliced_data['width'],
            height=sliced_data['height'],
        )

        # scan Trajectory: if an veh has trajectory data, set is_valid to True
        for idx, id in enumerate(self.ids):
            # idx = idx_selection_range[i]
            if np.any(traj.valid[idx, :] == True):
                self.obj_dict[self.ids[idx]]['is_valid'] = True
                # print(f"vehicle {self.ids[idx]} has invalid datapoint in traj")
            else:
                self.obj_dict[self.ids[idx]]['is_valid'] = False
                # print(traj.valid[idx, :])
                # traj.valid[idx, :] = False  # mark the trajectory as invalid, since it is not what we are interested in

        # Directly iterate over selected items from obj_dict using idx_selection_range
        # selected_objs = [self.obj_dict[self.ids[i]] for i in idx_selection_range]
        obj_metadata = object_state.ObjectMetadata(
            # ids=np.array([obj['id'] for obj in selected_objs], dtype=np.int32),
            ids=np.arange(len(self.ids), dtype=np.int32),  # Temporary ids from 0 to max_num_objects-1
            object_types=np.array([obj['object_type'] for obj in self.obj_dict.values()], dtype=np.int32),
            is_sdc=np.array([obj['is_sdc'] for obj in self.obj_dict.values()], dtype=bool),
            is_modeled=np.array([obj['is_modeled'] for obj in self.obj_dict.values()], dtype=bool),
            is_valid=np.array([obj['is_valid'] for obj in self.obj_dict.values()], dtype=bool),
            objects_of_interest=np.array([obj['objects_of_interest'] for obj in self.obj_dict.values()], dtype=bool),
            is_controlled=np.array([obj['is_controlled'] for obj in self.obj_dict.values()], dtype=bool)
        )
        # if sdc_id in self.ids:
            # obj_metadata.is_sdc[0] = True  # Set SDC index to True - we always have SDC in the first place
        # print(f"Object metadata shape: {obj_metadata.ids.shape}")

        return traj, obj_metadata
    
    def create_timestamp_micros(self):
        """
        Creates a timestamp array from the parsed trajectory data.

        Inputs:
            None

        Returns:
            np.ndarray: A numpy array containing the timestamps in microseconds.
        """
        # Check if trajectory data is available
        if not hasattr(self, 'trajectory_data'):
            raise ValueError("Trajectory data not parsed. Please call _parse_trajectory() first.")

        # Create a timestamp array
        return self.timestamp_micros

    def create_traffic_light(self):
        """
        Creates a traffic light object from the parsed trajectory data.

        Inputs:
            None

        Returns:
            waymax.datatypes.roadgraph.TrafficLight: A traffic light object containing the parsed data.
        """

        length = self.traj_length
        # Create a traffic light object
        # Shape should be (..., num_objects, num_timesteps)
        xs = jnp.zeros((1, length), dtype=jnp.float32)
        ys = jnp.zeros((1, length), dtype=jnp.float32)
        zs = jnp.zeros((1, length), dtype=jnp.float32)
        states = jnp.zeros((1, length), dtype=jnp.int32)
        lane_ids = jnp.zeros((1, length), dtype=jnp.int32)
        valid = jnp.zeros((1, length), dtype=bool)
        # Set the traffic light data
        traffic_light = traffic_lights.TrafficLights(
            x=xs,
            y=ys,
            z=zs,
            state=states,
            lane_ids=lane_ids,
            valid=valid
        )

        return traffic_light



    def print_trajectory_tree(self):
        """
        Prints the trajectory tree structure for debugging purposes.

        Inputs:
            None
        Returns:
            None
        """

        if not hasattr(self, 'trajectory_tree'):
            self._load_trajectory()

        text = ""
        for elem in self.root.iter():
            if elem.tag == "timestep":
                text += f"Tag: {elem.tag}, Attributes: {elem.attrib}, Text: {elem.text}\n"
            if elem.tag == "vehicle":
                text += f"Tag: {elem.tag}, Attributes: {elem.attrib}, Text: {elem.text}\n"
            # break

        print(text)

    def id_to_index(self, id):
        """
        Converts an object id to its index in the trajectory data.

        Inputs:
            id (int): The object id.

        Returns:
            int: The index of the object in the trajectory data.
        """
        if id not in self.ids:
            raise ValueError(f"Object id {id} not found in the trajectory data.")
        return self.ids.index(id)
    
    def index_to_id(self, index):
        """
        Converts an index to its corresponding object id in the trajectory data.

        Inputs:
            index (int): The index of the object.

        Returns:
            int: The object id.
        """
        if index < 0 or index >= len(self.ids):
            raise ValueError(f"Index {index} out of range.")
        return self.ids[index]


if __name__ == "__main__":
    # create a scenario object or from a map file using sumo2waymo
    scenario = scenario_pb2.Scenario()
    converter = TrajectoryConverter('fcd_all.xml')
    converter.parse(start_at=100.0, traj_length=8)
    scenario = converter.create_waymo_trajectory(scenario)
    # save the scenario to a file
    with open('test_traj.pb', 'wb') as f:
        f.write(scenario.SerializeToString())