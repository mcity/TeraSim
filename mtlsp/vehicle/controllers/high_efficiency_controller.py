from terasim.vehicle.controllers.base_controller import BaseController
import logging
import terasim.utils as utils
from terasim.simulator import traci

class HighEfficiencyController(BaseController):
    params = {
            "v_high": 40,
            "v_low": 20,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
        }
    
    def __init__(self, simulator, params=None):
        super().__init__(simulator, params)
        self.is_busy = False
        self.controlled_duration = 0
        self.step_size = utils.get_step_size()
        self.lc_step_num = int(self.params["lc_duration"]/self.step_size)

    def _is_command_legal(self, veh_id, control_command):
        """Check if the control command is legal.
        """
        if control_command["type"] != "lon_lat" or "longitudinal" not in control_command or "lateral" not in control_command:
            raise ValueError("The control command should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.")
        
        if self.is_busy:
            assert self.controlled_duration >= 0
            if self.controlled_duration == 0:
                self.is_busy = False
            else:
                self.controlled_duration -= 1
                self.controlled_duration = max(self.controlled_duration, 0)
            logging.info("Control command assigned while lane change maneuver not finished")
            return False
        else:
            if control_command["lateral"] == "left" and not self.simulator.get_vehicle_lane_adjacent(veh_id, 1):
                logging.info("Left lane change to a invalid lane")
                return False
            if control_command["lateral"] == "right" and not self.simulator.get_vehicle_lane_adjacent(veh_id, -1):
                logging.info("Right lane change to a invalid lane")
                return False
            return True

    def execute_control_command(self, veh_id, control_command, obs_dict):
        """Vehicle acts based on the input action.

        Args:
            action (dict): Lonitudinal and lateral actions. It should have the format: {'longitudinal': float, 'lateral': str}. The longitudinal action is the longitudinal acceleration, which should be a float. The lateral action should be the lane change direction. 'central' represents no lane change. 'left' represents left lane change, and 'right' represents right lane change.
        """ 
        # Remove all control limits from SUMO
        utils.set_vehicle_speedmode(veh_id, 0)
        
        # Longitudinal control
        controlled_acc = control_command["longitudinal"]
        current_velocity = obs_dict["ego"].data["speed"]
        if current_velocity + controlled_acc > self.params["v_high"]:
            controlled_acc = self.params["v_high"] - current_velocity
        elif current_velocity + controlled_acc < self.params["v_low"]:
            controlled_acc = self.params["v_low"] - current_velocity
        # Lateral control
        if control_command["lateral"] == "SUMO":
            utils.set_vehicle_lanechangemode(veh_id)
            self.simulator.change_vehicle_speed(veh_id, controlled_acc, self.params["acc_duration"])
        else:
            utils.set_vehicle_lanechangemode(veh_id, 0)
            if control_command["lateral"] == "central":
                current_lane_offset = utils.get_vehicle_lateral_lane_position(veh_id)
                self.simulator.change_vehicle_sublane_dist(veh_id, -current_lane_offset, self.step_size)
                self.simulator.change_vehicle_speed(veh_id, controlled_acc, self.params["acc_duration"])
            else:
                self.simulator.change_vehicle_lane(veh_id, control_command["lateral"], self.params["lc_duration"])
                self.simulator.change_vehicle_speed(veh_id, controlled_acc, self.params["lc_duration"])
                self.controlled_duration = self.lc_step_num  # begin counting the lane change maneuver timesteps
                self.is_busy = True