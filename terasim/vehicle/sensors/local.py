import addict
from terasim.agent.agent_sensor import AgentSensor
from terasim.simulator import Simulator
from terasim.overlay import traci
from terasim import utils
import traci.constants as tc

class LocalSensor(AgentSensor):
    """
    LocalSensor is a basic sensor that subscribe to some SUMO variables of a vehicle.
    
    A LocalSensor will maintain a observation, which is a nested dictionary observation.data.time_stamp
    observation.data: a dictionary{
        'Ego': {'veh_id': vehicle ID, 'speed': vehicle velocity [m/s], 'position': tuple of X,Y coordinates [m], 'heading': vehicle angle [degree], 'lane_index': lane index of vehicle, 'distance': 0 [m], 'acceleration': m/s^2},
        'Lead'
        'Foll'
        'LeftLead'
        'RightLead'
        'LeftFoll'
        'RightFoll'
    }
    """
    DEFAULT_PARAMS = dict(
        obs_range = 120,
        no_opposite = True, 
        lane_filter_list = [-2,-1, 0, 1, 2],
        subscription_component = [tc.VAR_LENGTH, tc.VAR_POSITION, tc.VAR_SPEED, tc.VAR_LANE_INDEX, tc.VAR_ANGLE, tc.VAR_POSITION3D, tc.VAR_EDGES, tc.VAR_LANEPOSITION, tc.VAR_LANEPOSITION_LAT, tc.VAR_SPEED_LAT, tc.VAR_ROAD_ID, tc.VAR_ACCELERATION, tc.VAR_LANE_ID]
    )

    def __init__(self, name = "local", **params):
        super().__init__(name, **params)

    def subscribe(self):
        traci.vehicle.subscribeContext(self._agent.id,
                                       tc.CMD_GET_VEHICLE_VARIABLE,
                                       self.params.obs_range,
                                       self.params.subscription_component)

    def unsubscribe(self):
        traci.vehicle.unsubscribeContext(self._agent.id,
                                         tc.CMD_GET_VEHICLE_VARIABLE,
                                         self.params.obs_range)

    def fetch(self):
        common = dict(
            vehID = self._agent.id,
            obs_range = self._params.obs_range
        )

        return addict.Dict(
            Ego=LocalSensor.pre_process_subscription(veh_id=self._agent.id),
            Lead=utils.get_leading_vehicle(**common),
            LeftLead=utils.get_neighboring_leading_vehicle(dir="left", **common),
            RightLead=utils.get_neighboring_leading_vehicle(dir="right", **common),
            Foll=utils.get_following_vehicle(**common),
            LeftFoll=utils.get_neighboring_following_vehicle(dir="left", **common),
            RightFoll=utils.get_neighboring_following_vehicle(dir="right", **common)
        )

    @staticmethod
    def pre_process_subscription(veh_id, distance=0.0):
        """Modify the subscription results into a standard form.

        Args:
            veh_id (str, optional): Vehicle ID. Defaults to None.
            distance (float, optional): Distance from the ego vehicle [m]. Defaults to 0.0.

        Returns:
            dict: Standard for of vehicle information.
        """
        subscription = traci.vehicle.getContextSubscriptionResults(veh_id)
        veh_info = addict.Dict(
            veh_id=veh_id,
            could_drive_adjacent_lane_left=Simulator.get_vehicle_lane_adjacent(veh_id, 1),
            could_drive_adjacent_lane_right=Simulator.get_vehicle_lane_adjacent(veh_id, -1),
            distance=distance,
            heading=subscription[veh_id][67],
            lane_index=subscription[veh_id][82],
            lateral_speed=subscription[veh_id][50],
            lateral_offset=subscription[veh_id][184],
            position=subscription[veh_id][66],
            position3D=subscription[veh_id][57],
            velocity=subscription[veh_id][64],
            road_id=subscription[veh_id][80]
        )
        return veh_info

