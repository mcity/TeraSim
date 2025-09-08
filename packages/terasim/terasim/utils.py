import math
import uuid

from terasim.overlay import traci


def center_coordinate_to_sumo_coordinate(x, y, heading, length=5):
    """Convert the center coordinate to the SUMO coordinate. the input will be a list of {x, y, heading, length}.
    
    Args:
        x (float): x coordinate.
        y (float): y coordinate.
        heading (float): heading.
        length (int, optional): length. Defaults to 5.

    Returns:
        tuple: x and y coordinates.
    """
    x = x + math.cos(heading) * 0.5 * length
    y = y + math.sin(heading) * 0.5 * length
    return x, y


def sumo_coordinate_to_center_coordinate(x, y, heading, length=5):
    """Convert the SUMO coordinate to the center coordinate. the input will be a list of {x, y, heading, length}.
    
    Args:
        x (float): x coordinate.
        y (float): y coordinate.
        heading (float): heading.
        length (int, optional): length. Defaults to 5.

    Returns:
        tuple: x and y coordinates.
    """
    x = x - math.cos(heading) * 0.5 * length
    y = y - math.sin(heading) * 0.5 * length
    return x, y


def sumo_heading_to_orientation(sumo_heading):
    """Convert the SUMO heading to orientation.

    Args:
        sumo_heading (float): SUMO heading.

    Returns:
        float: Orientation.
    """
    radians = math.radians(90 - sumo_heading)
    return math.atan2(math.sin(radians), math.cos(radians))


def orientation_to_sumo_heading(orientation):
    """Convert the orientation to SUMO heading.

    Args:
        orientation (float): Orientation.

    Returns:
        float: SUMO heading.
    """
    degrees = math.degrees(orientation)
    degrees = (degrees + 360) % 360
    degrees = (90 - degrees) % 360
    return degrees


def getLoadedIDList():
    """Get the list of loaded vehicle IDs.

    Returns:
        list(str): A list of vehicle IDs.
    """
    return traci.simulation.getLoadedIDList()


def get_vehicle_route(vehID):
    """Get route of the vehicle.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        list(str): A list of edge ID.
    """
    return traci.vehicle.getRoute(vehID)


def get_vehicle_length(vehID):
    """Get vehicle length.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Vehicle length in m.
    """
    return traci.vehicle.getLength(vehID)


def get_vehicle_width(vehID):
    """Get vehicle width.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Vehicle width in m.
    """
    return traci.vehicle.getWidth(vehID)


def generate_unique_bv_id():
    """Randomly generate an ID of the background vehicle.

    Returns:
        str: ID of the background vehicle.
    """
    return "BV_" + str(uuid.uuid4())


def remap(v, x, y):
    """Remap the value v from the range of x to the range of y.

    Args:
        v (float): Value to be remapped.
        x (list): Original range.
        y (list): Target range.

    Returns:
        float: Remapped value.
    """
    return y[0] + (v - x[0]) * (y[1] - y[0]) / (x[1] - x[0])


def check_equal(x, y, error):
    """Check if x is approximately equal to y considering the given error.

    Args:
        x (float): Parameter 1.
        y (float): Parameter 2.
        error (float): Specified error.

    Returns:
        bool: True is x and y are close enough. Otherwise, False.
    """
    if abs(x - y) <= error:
        return True
    else:
        return False


def get_step_size():
    """Get the step size of the simulation.

    Returns:
        float: Step size in s.
    """
    return traci.simulation.getDeltaT()


def get_time():
    """Get current simulation time in SUMO.

    Returns:
        float: Simulation time in s.
    """
    return traci.simulation.getTime()


def get_vehicle_lateral_lane_position(vehID):
    """Get the lateral offset of the vehicle.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Lateral offset related to the lane's center.
    """
    return traci.vehicle.getLateralLanePosition(vehID)


def cal_dis_with_start_end_speed(v_start, v_end, acc, time_interval=1.0, v_low=20, v_high=40):
    """Calculate the travel distance with start and end speed and acceleration.

    Args:
        v_start (float): Start speed [m/s].
        v_end (float): End speed [m/s].
        acc (float): Acceleration [m/s^2].
        time_interval (float, optional): Time interval [s]. Defaults to 1.0.
        v_low (int, optional): Low speed [m/s]. Defaults to 20.
        v_high (int, optional): High speed [m/s]. Defaults to 40.

    Returns:
        float: Travel distance in the time interval.
    """
    if v_end == v_low or v_end == v_high:
        t_1 = (v_end - v_start) / acc if acc != 0 else 0
        t_2 = time_interval - t_1
        dis = v_start * t_1 + 0.5 * (acc) * (t_1**2) + v_end * t_2
    else:
        dis = ((v_start + v_end) / 2) * time_interval
    return dis


def cal_euclidean_dist(veh1_position=None, veh2_position=None):
    """Calculate Euclidean distance between two vehicles.

    Args:
        veh1_position (tuple, optional): Position of Vehicle 1 [m]. Defaults to None.
        veh2_position (tuple, optional): Position of Vehicle 2 [m]. Defaults to None.

    Raises:
        ValueError: If the position of fewer than two vehicles are provided, raise error.

    Returns:
        float: Euclidean distance between two vehicles [m].
    """
    if veh1_position is None or veh2_position is None:
        raise ValueError("Fewer than two vehicles are provided!")
    veh1_x, veh1_y = veh1_position[0], veh1_position[1]
    veh2_x, veh2_y = veh2_position[0], veh2_position[1]
    return math.sqrt(pow(veh1_x - veh2_x, 2) + pow(veh1_y - veh2_y, 2))


def get_leading_vehicle(vehID, obs_range):
    """Get the information of the leading vehicle.

    Args:
        vehID (str): ID of the ego vehicle.
        obs_range (float): Observation range.

    Returns:
        dict: necessary information of the leading vehicle, including:
            str: ID of the leading vehicle(accessed by 'veh_id'),
            float: Leading vehicle speed (accessed by 'velocity'),
            tuple(float, float): Leading vehicle position in X and Y (accessed by 'position'),
            int: Leading vehicle lane index (accessed by 'lane_index')
            float: Distance between the ego vehicle and the leading vehicle (accessed by 'distance').
    """
    # get leading vehicle information: a list:
    # first element: leader id
    # second element: distance from leading vehicle to ego vehicle
    # (it does not include the minGap of the ego vehicle)
    leader_info = traci.vehicle.getLeader(vehID, dist=obs_range)  # empty leader: None
    if not leader_info:
        return None
    else:
        r = leader_info[1] + traci.vehicle.getMinGap(vehID)
        return get_ego_vehicle(vehID=leader_info[0], obs_range=obs_range, dist=r)


def get_following_vehicle(vehID, obs_range):
    """Get the information of the following vehicle.

    Args:
        vehID (str): ID of the ego vehicle.
        obs_range (float): Observation range.

    Returns:
        dict: necessary information of the following vehicle, including:
            str: ID of the following vehicle(accessed by 'veh_id'),
            float: Following vehicle speed (accessed by 'velocity'),
            tuple(float, float): Following vehicle position in X and Y (accessed by 'position'),
            int: Following vehicle lane index (accessed by 'lane_index')
            float: Distance between the ego vehicle and the following vehicle (accessed by 'distance').
    """
    # get following vehicle information: a list:
    # first element: follower id
    # second element: distance from ego vehicle to following vehicle
    # (it does not include the minGap of the following vehicle)
    follower_info = traci.vehicle.getFollower(vehID, dist=obs_range)  # empty follower: ('',-1)
    if follower_info[1] == -1:
        return None
    else:
        r = follower_info[1] + traci.vehicle.getMinGap(follower_info[0])
        return get_ego_vehicle(vehID=follower_info[0], obs_range=obs_range, dist=r)


def get_neighboring_leading_vehicle(vehID, obs_range, dir):
    """Get the information of the neighboring leading vehicle.

    Args:
        vehID (str): ID of the ego vehicle.
        obs_range (float): Observation range.
        dir (str): Choose from "left" and "right".

    Returns:
        dict: necessary information of the neighboring leading vehicle, including:
            str: ID of the neighboring leading vehicle(accessed by 'veh_id'),
            float: Neighboring leading vehicle speed (accessed by 'velocity'),
            tuple(float, float): Neighboring leading vehicle position in X and Y (accessed by 'position'),
            int: Neighboring leading vehicle lane index (accessed by 'lane_index')
            float: Distance between the ego vehicle and the neighboring leading vehicle (accessed by 'distance').
    """
    # get neighboring leading vehicle information: a list of tuple:
    # first element: leader id
    # second element: distance from leading vehicle to ego vehicle
    # (it does not include the minGap of the ego vehicle)
    if dir == "left":
        leader_info = traci.vehicle.getNeighbors(vehID, 2)  # empty leftleader: len=0
    elif dir == "right":
        leader_info = traci.vehicle.getNeighbors(vehID, 3)  # empty rightleader: len=0
    else:
        raise ValueError("NotKnownDirection when fetching adjacent vehicle information")
    if len(leader_info) == 0:
        return None
    else:
        leader_info_list = [list(item) for item in leader_info]
        for i in range(len(leader_info)):
            leader_info_list[i][1] += traci.vehicle.getMinGap(vehID)
        sorted_leader = sorted(leader_info_list, key=lambda l: l[1])
        closest_leader = sorted_leader[0]
        return get_ego_vehicle(vehID=closest_leader[0], obs_range=obs_range, dist=closest_leader[1])


def get_neighboring_following_vehicle(vehID, obs_range, dir):
    """Get the information of the neighboring following vehicle.

    Args:
        vehID (str): ID of the ego vehicle.
        obs_range (float): Observation range.
        dir (str): Choose from "left" and "right".

    Returns:
        dict: necessary information of the neighboring following vehicle, including:
            str: ID of the neighboring following vehicle(accessed by 'veh_id'),
            float: Neighboring following vehicle speed (accessed by 'velocity'),
            tuple(float, float): Neighboring following vehicle position in X and Y (accessed by 'position'),
            int: Neighboring following vehicle lane index (accessed by 'lane_index')
            float: Distance between the ego vehicle and the neighboring following vehicle (accessed by 'distance').
    """
    # get neighboring following vehicle information: a list of tuple:
    # first element: follower id
    # second element: distance from ego vehicle to following vehicle
    # (it does not include the minGap of the following vehicle)
    if dir == "left":
        follower_info = traci.vehicle.getNeighbors(vehID, 0)  # empty leftfollower: len=0
    elif dir == "right":
        follower_info = traci.vehicle.getNeighbors(vehID, 1)  # empty rightfollower: len=0
    else:
        raise ValueError("NotKnownDirection when fetching adjacent vehicle information")
    if len(follower_info) == 0:
        return None
    else:
        follower_info_list = [list(item) for item in follower_info]
        for i in range(len(follower_info)):
            follower_info_list[i][1] += traci.vehicle.getMinGap(follower_info_list[i][0])
        sorted_follower = sorted(follower_info_list, key=lambda l: l[1])
        closest_follower = sorted_follower[0]
        return get_ego_vehicle(
            vehID=closest_follower[0], obs_range=obs_range, dist=closest_follower[1]
        )


def get_ego_vehicle(vehID, obs_range, dist=0.0):
    """Get the information of the ego vehicle.

    Args:
        vehID (str): ID of the ego vehicle.
        obs_range (float): Observation range.
        dist (float, optional): Distance between two vehicles. Defaults to 0.0.

    Returns:
        dict: Necessary information of the ego vehicle, including:
            str: Vehicle ID (accessed by 'veh_id'),
            float: Vehicle speed (accessed by 'velocity'),
            tuple(float, float): Vehicle position in X and Y (accessed by 'position'),
            int: Vehicle lane index (accessed by 'lane_index')
            float: Distance between the ego vehicle and another vehicle (accessed by 'distance').
    """
    ego_veh = None
    if dist <= obs_range:
        ego_veh = {"veh_id": vehID}
        ego_veh["distance"] = dist
        ego_veh["velocity"] = traci.vehicle.getSpeed(vehID)
        ego_veh["position"] = traci.vehicle.getPosition(vehID)
        ego_veh["position3d"] = traci.vehicle.getPosition3D(vehID)
        ego_veh["heading"] = traci.vehicle.getAngle(vehID)
        ego_veh["edge_id"] = traci.vehicle.getRoadID(vehID)
        ego_veh["lane_id"] = traci.vehicle.getLaneID(vehID)
        ego_veh["lane_index"] = traci.vehicle.getLaneIndex(vehID)
        ego_veh["acceleration"] = traci.vehicle.getAcceleration(vehID)
    return ego_veh


def set_vehicle_speedmode(vehID, speedmode=31):
    """Set the speed mode of the vehicle. This command controls how speeds set with the command setSpeed and slowDown are used. Per default, the vehicle may only drive slower than the speed that is deemed safe by the car following model and it may not exceed the bounds on acceleration and deceleration. Furthermore, vehicles follow the right-of-way rules when approaching an intersection and if necessary they brake hard to avoid driving across a red light.

    Args:
        vehID (str): Vehicle ID.
        speedmode (int, optional): This integer is a bitset (bit0 is the least significant bit) with the following fields. Defaults to 31.
            bit0: Regard safe speed.
            bit1: Regard maximum acceleration.
            bit2: Regard maximum deceleration.
            bit3: Regard right of way at intersections.
            bit4: Brake hard to avoid passing a red light.
    """
    traci.vehicle.setSpeedMode(vehID, speedmode)


def set_vehicle_lanechangemode(vehID, lanechangemode=1621):
    """Sets how lane changing in general and lane changing requests by TraCI are performed.

    Args:
        vehID (str): Vehicle ID.
        lanechangemode (int, optional): If an external change lane command (0x13) command is in conflict with the internal request this is resolved by the current value of the vehicles lane change mode. The given integer is interpreted as a bitset (bit0 is the least significant bit) with the following fields. Defaults to 1621.
        The default lane change mode is 0b011001010101 = 1621 which means that the laneChangeModel may execute all changes unless in conflict with TraCI. Requests from TraCI are handled urgently (with cooperative speed adaptations by the ego vehicle and surrounding traffic) but with full consideration for safety constraints.
        The lane change mode when controlling BV is 0b010001010101 = 1109 which means that the laneChangeModel may execute all changes unless in conflict with TraCI. Requests from TraCI are handled urgently without consideration for safety constraints.
            - bit1, bit0: 00 = do no strategic changes; 01 = do strategic changes if not in conflict with a TraCI request; 10 = do strategic change even if - overriding TraCI request.
            - bit3, bit2: 00 = do no cooperative changes; 01 = do cooperative changes if not in conflict with a TraCI request; 10 = do cooperative change even if overriding TraCI request.
            - bit5, bit4: 00 = do no speed gain changes; 01 = do speed gain changes if not in conflict with a TraCI request; 10 = do speed gain change even if overriding TraCI request.
            - bit7, bit6: 00 = do no right drive changes; 01 = do right drive changes if not in conflict with a TraCI request; 10 = do right drive change even if overriding TraCI request.
            - bit9, bit8:
                00 = do not respect other drivers when following TraCI requests, adapt speed to fulfill request;
                01 = avoid immediate collisions when following a TraCI request, adapt speed to fulfill request;
                10 = respect the speed / brake gaps of others when changing lanes, adapt speed to fulfill request;
                11 = respect the speed / brake gaps of others when changing lanes, no speed adaption.
            - bit11, bit10: 00 = do no sublane changes; 01 = do sublane changes if not in conflict with a TraCI request; 10 = do sublane change even if overriding TraCI request.
    """
    traci.vehicle.setLaneChangeMode(vehID, lanechangemode)


def get_speed_without_traCI(vehID):
    """Get the speed of the vehicle without using TraCI.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Speed of the vehicle without TraCI.
    """
    return traci.vehicle.getSpeedWithoutTraCI(vehID)


def get_speed(vehID):
    """Get the (longitudinal) speed in m/s of the named vehicle within the last step.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Speed of the vehicle within the last step.
    """
    return traci.vehicle.getSpeed(vehID)


def get_next_traffic_light(vehID):
    """Returns the information of the next traffic light the vehicle will encounter, which
    will be a list of upcoming traffic lights [(tlsID, tlsIndex, distance, state), ...].

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: The distance to the first upcoming traffic light (tlsID, tlsIndex, distance, state).
    """
    tls = traci.vehicle.getNextTLS(vehID)
    if len(tls) == 0:
        return ()
    else:
        return tls[0]


def get_vehicle_speedmode(vehID):
    """Retrieves how the values set by speed (0x40) and slowdown (0x14) shall be treated.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        int: This integer is a bitset (bit0 is the least significant bit) with the following fields.
    """
    return traci.vehicle.getSpeedMode(vehID)


def highlight_vehicle(vehID, duration=-1, color=(255, 0, 0, 255)):
    """Highlight the vehicle with the given ID in the GUI.

    Args:
        vehID (str): Vehicle ID.
        duration (int, optional): Duration of the highlight. Defaults to -1.
        color (tuple, optional): Color of the highlight. Defaults to (255, 0, 0, 255).
    """
    traci.vehicle.highlight(vehID, duration=duration, color=color, alphaMax=0.1)


def get_waiting_time(vehID):
    """Returns the waiting time of the named vehicle in seconds.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Waiting time of the vehicle in seconds.
    """
    return traci.vehicle.getWaitingTime(vehID)


def get_vehicle_lanechangemode(vehID):
    """Returns the lane change mode of the named vehicle.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        int: This integer is a bitset (bit0 is the least significant bit) with the following fields.
    """
    return traci.vehicle.getLaneChangeMode(vehID)


def get_distance(vehID):
    """Returns the distance (in m) to the vehicle ahead.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Distance to the vehicle ahead.
    """
    return traci.vehicle.getDistance(vehID)


def get_vehicle_angle(vehID):
    """Returns the angle in degrees of the named vehicle within the last step.

    Args:
        vehID (str): Vehicle ID.

    Returns:
        float: Angle of the vehicle within the last step.
    """
    return traci.vehicle.getAngle(vehID)
