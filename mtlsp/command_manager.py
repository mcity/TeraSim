from mtlsp.overlay import traci
from traci import constants as tc
from typing import List

class CommandPresets:
    SET_SUMO_TF = "SetSumoTransform"
    SET_CARLA_TF = "SetCarlaTransform"

class Executor:
    def __init__(self) -> None:
        pass
    
    @property
    def support_commands(self) -> List[CommandPresets]:
        raise NotImplementedError

    def execute(self, veh_id, control_command, obs_dict):
        raise NotImplementedError

class CarlaExecutor(Executor):
    @property
    def support_commands(self) -> List[CommandPresets]:
        return [CommandPresets.SET_CARLA_TF]

    def execute(self, veh_id, control_command, obs_dict):
        pass
    
    def set_transform_carla(self, veh_id, position, angle, velocity):
        pass

class SumoExecutor(Executor):
    @property
    def support_commands(self) -> List[CommandPresets]:
        return [CommandPresets.SET_SUMO_TF]

    def execute(self, veh_id, control_command, obs_dict):
        keepRoute = control_command.get("keepRoute", 2)
        self.set_transform_sumo(veh_id, control_command["position"], control_command["velocity"], control_command["angle"], keepRoute)

    def set_transform_sumo(self, veh_id, position, velocity, angle, keepRoute, speedmode=0):
        """Apply the SUMO movePosition command to the vehicle.
        
        Args:
            veh_id (str): ID of the vehicle.
            position (tuple): Position of the vehicle.
            velocity (float): Velocity of the vehicle.
            angle (float): Angle of the vehicle.
        """
        # Move vehicle to specific position, speed, and angle without specifying the lane and the edge
        traci.vehicle.setSpeedMode(veh_id, speedmode)
        traci.vehicle.moveToXY(veh_id, edgeID="", laneIndex=-1, x=position[0], y=position[1], angle=angle, keepRoute=keepRoute)
        if velocity is not None:
            traci.vehicle.setSpeed(veh_id, velocity)

class CommandManager(object):
    
    def __init__(self, simulator):
        self.simulator = simulator
        self.sim_executor_list = []
        self.register(SumoExecutor())

    def _install(self, vehicle):
        pass

    def register(self, sim_executor):
        self.sim_executor_list.append(sim_executor)

    def _is_command_legal(self, veh_id, control_command):
        """Check if the control command is legal.

        Args:
            veh_id (str): ID of the vehicle.
            control_command (dict): Control command.

        Returns:
            bool: True if the control command is legal, False otherwise.
        """
        if len(self.sim_executor_list) == 0:
            raise ValueError("No sim executor registered.")
        for sim_executor in self.sim_executor_list:
            if control_command["type"] in sim_executor.support_commands:
                return True
        print(f"Unknown control command type: {control_command['type']} for vehicle {veh_id}")
        return False

    def execute_control_command(self, veh_id, control_command, obs_dict=None):
        """Execute the control command of a vehicle.
        
        Args:
            veh_id (str): ID of the vehicle.
            control_command (dict): Control command.
            obs_dict (dict, optional): Observation dict. Defaults to None.
        """
        for sim_executor in self.sim_executor_list:
            if control_command["type"] in sim_executor.support_commands:
                sim_executor.execute(veh_id, control_command, obs_dict)
                return True
        raise ValueError("Unknown control command type: {}".format(control_command["type"]))