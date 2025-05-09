from __future__ import division, print_function

import numpy as np
from scipy import stats

# Longitudinal policy parameters
from terasim.overlay import traci
from terasim.vehicle.decision_models.highway_base_decision_model import HighwayBaseDecisionModel

# Lateral policy parameters
POLITENESS = 0.0  # in [0, 1]
LANE_CHANGE_MIN_ACC_GAIN = 0.1  # [m/s2]
LANE_CHANGE_MAX_BRAKING_IMPOSED = 4.0  # [m/s2]

# NDD Vehicle IDM parameters
COMFORT_ACC_MAX = 2  # [m/s2]
COMFORT_ACC_MIN = -4.0  # [m/s2]
DISTANCE_WANTED = 5.0  # [m]
TIME_WANTED = 1.5  # [s]
DESIRED_VELOCITY = 35  # [m/s]
DELTA = 4.0  # []

acc_low = -4
acc_high = 2
stochastic_IDM_resolution = 0.2
stochastic_IDM_prob_threshold = 1e-10
LENGTH = 5


class IDMModel(HighwayBaseDecisionModel):
    """A vehicle using both a longitudinal and a lateral decision policies.

    - Longitudinal: the IDM model computes an acceleration given the preceding vehicle's distance and velocity.
    - Lateral: the MOBIL model decides when to change lane by maximizing the acceleration of nearby vehicles.
    """

    def __init__(
        self,
        MOBIL_lc_flag=True,
        stochastic_acc_flag=False,
        IDM_parameters=None,
        MOBIL_parameters=None,
    ):
        """Initialization of the IDM mdoel

        Args:
            MOBIL_lc_flag (bool, optional): whether to include MOBIL lane change model, if not, will call SUMO lane change model. Defaults to True.
            stochastic_acc_flag (bool, optional): Whether the acceleration will be stochastic and random sampled. Defaults to False.
        """
        super().__init__()
        self.MOBIL_lc_flag = MOBIL_lc_flag
        self.stochastic_acc_flag = stochastic_acc_flag
        self.load_parameters(IDM_parameters, MOBIL_parameters)

    def load_parameters(self, IDM_parameters, MOBIL_parameters):
        """Load the IDM and MOBIL parameters.

        Args:
            IDM_parameters (dict, optional): IDM parameters. Defaults to None.
            MOBIL_parameters (dict, optional): MOBIL parameters. Defaults to None.
        """
        IDM_parameters = IDM_parameters if IDM_parameters else {}

        self.COMFORT_ACC_MAX = (
            IDM_parameters["COMFORT_ACC_MAX"]
            if "COMFORT_ACC_MAX" in IDM_parameters
            else COMFORT_ACC_MAX
        )
        self.COMFORT_ACC_MIN = (
            IDM_parameters["COMFORT_ACC_MIN"]
            if "COMFORT_ACC_MIN" in IDM_parameters
            else COMFORT_ACC_MIN
        )
        self.DISTANCE_WANTED = (
            IDM_parameters["DISTANCE_WANTED"]
            if "DISTANCE_WANTED" in IDM_parameters
            else DISTANCE_WANTED
        )
        self.TIME_WANTED = (
            IDM_parameters["TIME_WANTED"] if "TIME_WANTED" in IDM_parameters else TIME_WANTED
        )
        self.DESIRED_VELOCITY = (
            IDM_parameters["DESIRED_VELOCITY"]
            if "DESIRED_VELOCITY" in IDM_parameters
            else DESIRED_VELOCITY
        )
        self.DELTA = IDM_parameters["DELTA"] if "DELTA" in IDM_parameters else DELTA
        self.stochastic_IDM_resolution = (
            IDM_parameters["stochastic_IDM_resolution"]
            if "stochastic_IDM_resolution" in IDM_parameters
            else stochastic_IDM_resolution
        )
        self.stochastic_IDM_prob_threshold = (
            IDM_parameters["stochastic_IDM_prob_threshold"]
            if "stochastic_IDM_prob_threshold" in IDM_parameters
            else stochastic_IDM_prob_threshold
        )
        self.LENGTH = IDM_parameters["LENGTH"] if "LENGTH" in IDM_parameters else LENGTH
        self.acc_low = IDM_parameters["acc_low"] if "acc_low" in IDM_parameters else acc_low
        self.acc_high = IDM_parameters["acc_high"] if "acc_high" in IDM_parameters else acc_high

        MOBIL_parameters = MOBIL_parameters if MOBIL_parameters else {}
        self.POLITENESS = (
            MOBIL_parameters["POLITENESS"] if "POLITENESS" in MOBIL_parameters else POLITENESS
        )
        self.LANE_CHANGE_MIN_ACC_GAIN = (
            MOBIL_parameters["LANE_CHANGE_MIN_ACC_GAIN"]
            if "LANE_CHANGE_MIN_ACC_GAIN" in MOBIL_parameters
            else LANE_CHANGE_MIN_ACC_GAIN
        )
        self.LANE_CHANGE_MAX_BRAKING_IMPOSED = (
            MOBIL_parameters["LANE_CHANGE_MAX_BRAKING_IMPOSED"]
            if "LANE_CHANGE_MAX_BRAKING_IMPOSED" in MOBIL_parameters
            else LANE_CHANGE_MAX_BRAKING_IMPOSED
        )

    def derive_control_command_from_observation(self, obs_dict):
        """Derive control command from observation.

        Args:
            obs_dict (dict): The observation dictionary.

        Returns:
            dict: The control command.
        """
        if "local" not in obs_dict:
            raise ValueError("No local observation")
        control_command, mode = self.decision(obs_dict["local"])
        return control_command, None

    def decision(self, observation):
        """Vehicle decides next action based on IDM model.

        Args:
            observation (dict): Observation of the vehicle.

        Returns:
            dict, str: The control command. The mode of the decision.
        """
        action = None
        mode = None
        ego_vehicle = observation["Ego"]
        front_vehicle = observation["Lead"]
        # Lateral: MOBIL
        mode = "MOBIL"
        left_gain, right_gain = 0, 0
        left_LC_flag, right_LC_flag = False, False

        if self.MOBIL_lc_flag:
            # see if the vehicle can change lane
            possible_lane_change = []
            (
                possible_lane_change.append(-1)
                if observation["Ego"]["could_drive_adjacent_lane_right"]
                else None
            )
            (
                possible_lane_change.append(1)
                if observation["Ego"]["could_drive_adjacent_lane_left"]
                else None
            )
            # calculate the gain of changing lane and the lane change flag
            for lane_index in possible_lane_change:
                LC_flag, gain = self.mobil_gain(lane_index, observation)
                if LC_flag and gain:
                    if lane_index < 0:
                        right_gain, right_LC_flag = np.clip(gain, 0.0, None), LC_flag
                    elif lane_index > 0:
                        left_gain, left_LC_flag = np.clip(gain, 0.0, None), LC_flag

        if left_LC_flag or right_LC_flag:
            if right_gain > left_gain:
                action = {"lateral": "right", "longitudinal": 0}
                assert right_gain >= self.LANE_CHANGE_MIN_ACC_GAIN
            else:
                action = {"lateral": "left", "longitudinal": 0}
                assert left_gain >= self.LANE_CHANGE_MIN_ACC_GAIN
        # Longitudinal: IDM
        else:
            mode = "IDM"
            if not self.stochastic_acc_flag:
                tmp_acc = self.IDM_acceleration(
                    ego_vehicle=ego_vehicle, front_vehicle=front_vehicle
                )
            else:
                tmp_acc = self.stochastic_IDM_acceleration(
                    ego_vehicle=ego_vehicle, front_vehicle=front_vehicle
                )
            tmp_acc = np.clip(tmp_acc, self.acc_low, self.acc_high)
            if not self.MOBIL_lc_flag:
                action = {"lateral": "SUMO", "longitudinal": tmp_acc}
            else:
                action = {"lateral": "central", "longitudinal": tmp_acc}
        action["type"] = "lon_lat"
        return action, mode

    def IDM_acceleration(self, ego_vehicle=None, front_vehicle=None, distance=None):
        """Compute an acceleration command with the Intelligent Driver Model. The acceleration is chosen so as to:
            - reach a target velocity;
            - maintain a minimum safety distance (and safety time) w.r.t the front vehicle.

        Args:
            ego_vehicle (dict, optional): Information of the vehicle whose desired acceleration is to be computed. It does not have to be an IDM vehicle, which is why this method is a class method. This allows an IDM vehicle to reason about other vehicles behaviors even though they may not IDMs. Defaults to None.
            front_vehicle (dict, optional): Information of the vehicle preceding the ego-vehicle. Defaults to None.
            mode (str, optional): Difference IDM parameters for BV and AV. Defaults to None.

        Returns:
            float: Acceleration command for the ego-vehicle in m/s^2.
        """
        if not ego_vehicle:
            return 0

        a0 = self.COMFORT_ACC_MAX
        v0 = self.DESIRED_VELOCITY
        delt = self.DELTA
        acceleration = a0 * (1 - np.power(ego_vehicle["velocity"] / v0, delt))
        if front_vehicle is not None:
            r = front_vehicle["distance"] if distance is None else distance
            d = max(1e-5, r)
            acceleration -= a0 * np.power(self.desired_gap(ego_vehicle, front_vehicle) / d, 2)
        return acceleration

    def stochastic_IDM_acceleration(self, ego_vehicle, front_vehicle):
        """Compute an acceleration command with the Intelligent Driver Model. The acceleration is chosen so as to:
            - reach a target velocity;
            - maintain a minimum safety distance (and safety time) w.r.t the front vehicle.
            The acceleration is stochastic and randomly sampled.

        Args:
            ego_vehicle (dict): Information of the vehicle whose desired acceleration is to be computed.
            front_vehicle (dict): Information of the vehicle preceding the ego-vehicle.

        Returns:
            float: Acceleration command for the ego-vehicle in m/s^2.
        """
        tmp_acc = self.IDM_acceleration(ego_vehicle=ego_vehicle, front_vehicle=front_vehicle)
        tmp_acc = np.clip(tmp_acc, self.acc_low, self.acc_high)
        stochastic_IDM_num = (
            int((self.acc_high - self.acc_low) / self.stochastic_IDM_resolution) + 1
        )
        acc_list = np.linspace(self.acc_low, self.acc_high, stochastic_IDM_num)
        acc_possi_list = stats.norm.pdf(acc_list, tmp_acc, 1)
        # Delete possi if smaller than certain threshold
        acc_possi_list = [
            val if val > self.stochastic_IDM_prob_threshold else 0 for val in acc_possi_list
        ]
        assert sum(acc_possi_list) > 0, "The sum of the probability is 0"
        acc_possi_list = acc_possi_list / (sum(acc_possi_list))
        final_acc_idx = np.random.choice(len(acc_list), 1, replace=False, p=acc_possi_list).item()
        final_acc = acc_list[final_acc_idx]
        return final_acc

    def desired_gap(self, ego_vehicle, front_vehicle=None):
        """Compute the desired distance between a vehicle and its leading vehicle.

        Args:
            ego_vehicle (dict): Information of the controlled vehicle.
            front_vehicle (dict, optional): Information of the leading vehicle. Defaults to None.

        Returns:
            float: Desired distance between the two vehicles in m.
        """
        d0 = self.DISTANCE_WANTED
        tau = self.TIME_WANTED
        ab = -self.COMFORT_ACC_MAX * self.COMFORT_ACC_MIN
        dv = ego_vehicle["velocity"] - front_vehicle["velocity"]
        d_star = d0 + max(
            0,
            ego_vehicle["velocity"] * tau + ego_vehicle["velocity"] * dv / (2 * np.sqrt(ab)),
        )
        return d_star

    def mobil_gain(self, lane_index, observation):
        """MOBIL model for the vehicle.

        Args:
            lane_index (int): Lane change direction, -1 represents right, 1 represents left.
            observation (dict): Observation of the vehicle.

        Returns:
            bool, float: Whether it is feasible to do the lane change maneuver. Lane change gain.
        """
        try:
            mingap = traci.vehicle.getMinGap(observation["Ego"]["veh_id"])
        except:
            mingap = 3.28
        gain = None
        ego = observation["Ego"]

        # Is the maneuver unsafe for the new following vehicle?
        if lane_index == 1:
            new_preceding, new_following = (
                observation["LeftLead"],
                observation["LeftFoll"],
            )
        if lane_index == -1:
            new_preceding, new_following = (
                observation["RightLead"],
                observation["RightFoll"],
            )

        # Check whether will crash immediately
        r_new_preceding, r_new_following = 99999, 99999
        if new_preceding:
            r_new_preceding = new_preceding["distance"]
        if new_following:
            r_new_following = new_following["distance"]
        if r_new_preceding <= 0 or r_new_following <= 0:
            return False, gain

        new_following_distance = (
            new_following["distance"] + new_preceding["distance"] + 2 * mingap + self.LENGTH
            if (new_following and new_preceding)
            else 99999
        )
        new_following_a = self.IDM_acceleration(
            ego_vehicle=new_following,
            front_vehicle=new_preceding,
            distance=new_following_distance,
        )
        new_following_pred_a = self.IDM_acceleration(
            ego_vehicle=new_following,
            front_vehicle=ego,
            distance=new_following["distance"] if new_following else 99999,
        )

        old_preceding, old_following = observation["Lead"], observation["Foll"]
        self_pred_a = self.IDM_acceleration(ego_vehicle=ego, front_vehicle=new_preceding)

        # The deceleration of the new following vehicle after the the LC should not be too big (negative)
        if new_following_pred_a < -self.LANE_CHANGE_MAX_BRAKING_IMPOSED:
            return False, gain

        # Is there an acceleration advantage for me and/or my followers to change lane?
        self_a = self.IDM_acceleration(ego_vehicle=ego, front_vehicle=old_preceding)
        old_following_a = self.IDM_acceleration(
            ego_vehicle=old_following,
            front_vehicle=ego,
            distance=old_following["distance"] if old_following else 99999,
        )
        old_following_pred_a_distance = (
            old_following["distance"] + old_preceding["distance"] + 2 * mingap + self.LENGTH
            if (old_following and old_preceding)
            else 99999
        )
        old_following_pred_a = self.IDM_acceleration(
            ego_vehicle=old_following,
            front_vehicle=old_preceding,
            distance=old_following_pred_a_distance,
        )
        gain = (
            self_pred_a
            - self_a
            + self.POLITENESS
            * (new_following_pred_a - new_following_a + old_following_pred_a - old_following_a)
        )
        if gain <= self.LANE_CHANGE_MIN_ACC_GAIN:
            gain = None
            return False, gain

        # All clear, let's go!
        return True, gain
