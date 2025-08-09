import traci.constants as tc
from addict import Dict

from terasim import utils
from terasim.agent.agent_sensor import AgentSensor
from terasim.overlay import traci
from terasim.simulator import Simulator


class LocalSensor(AgentSensor):
    """
    LocalSensor is a basic sensor that subscribe to some SUMO variables of a vehicle.

    A LocalSensor will maintain a observation, which is a nested dictionary observation.time_stamp
    observation: a dictionary{
        'Ego': {'veh_id': vehicle ID, 'speed': vehicle velocity [m/s], 'position': tuple of X,Y coordinates [m], 'heading': vehicle angle [degree], 'lane_index': lane index of vehicle, 'distance': 0 [m], 'acceleration': m/s^2},
        'Lead'
        'Foll'
        'LeftLead'
        'RightLead'
        'LeftFoll'
        'RightFoll'
    }
    """

    def __init__(self, name="local", **params):
        """Initialize the local sensor for the vehicle.

        Args:
            name (str, optional): The name of the sensor. Defaults to "local".
            params (dict, optional): The parameters of the sensor.
        """
        super().__init__(name, **params)

    def fetch(self):
        """Fetch the vehicle information.

        Returns:
            dict: The vehicle information.
        """
        common = dict(vehID=self._agent.id, obs_range=self._params.obs_range)

        return Dict(
            Ego=LocalSensor.get_ego_vehicle_info(veh_id=self._agent.id),
            Lead=utils.get_leading_vehicle(**common),
            LeftLead=utils.get_neighboring_leading_vehicle(dir="left", **common),
            RightLead=utils.get_neighboring_leading_vehicle(dir="right", **common),
            Foll=utils.get_following_vehicle(**common),
            LeftFoll=utils.get_neighboring_following_vehicle(dir="left", **common),
            RightFoll=utils.get_neighboring_following_vehicle(dir="right", **common),
        )

    @staticmethod
    def get_ego_vehicle_info(veh_id, distance=0.0):
        """Modify the vehicle information into a standard form.

        Args:
            veh_id (str, optional): Vehicle ID. Defaults to None.
            distance (float, optional): Distance from the ego vehicle [m]. Defaults to 0.0.

        Returns:
            dict: Standard form of vehicle information.
        """
        veh_info = Dict(
            veh_id=veh_id,
            velocity=traci.vehicle.getSpeed(veh_id),
            position=traci.vehicle.getPosition(veh_id),
            position3d=traci.vehicle.getPosition3D(veh_id),
            heading=traci.vehicle.getAngle(veh_id),
            edge_id=traci.vehicle.getRoadID(veh_id),
            lane_id=traci.vehicle.getLaneID(veh_id),
            lane_index=traci.vehicle.getLaneIndex(veh_id),
            acceleration=traci.vehicle.getAcceleration(veh_id),
            could_drive_adjacent_lane_left=Simulator.get_vehicle_lane_adjacent(veh_id, 1),
            could_drive_adjacent_lane_right=Simulator.get_vehicle_lane_adjacent(veh_id, -1),
            distance=distance,
            lateral_speed=traci.vehicle.getLateralSpeed(veh_id),
            lateral_offset=traci.vehicle.getLateralLanePosition(veh_id),
        )
        return veh_info
