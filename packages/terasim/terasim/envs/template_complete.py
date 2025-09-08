import addict

from terasim.envs.base_complete import BaseEnvComplete
from terasim.params import AgentType


class EnvTemplateComplete(BaseEnvComplete):
    """This Env provides a basic Env implementation.

    Env developers can derived from this class or build their own implementations directly on BaseEnv
    """

    def on_step(self, ctx):
        """Functions to be called at each simulation step.

        Args:
            ctx (dict): The context information.
        """
        # Make decisions and execute commands
        control_cmds, infos = self.make_decisions(ctx)
        self.execute_control_commands(control_cmds)

        # Simulation stop check
        return self.should_continue_simulation()

    def make_decisions(self, ctx):
        """Make decisions for all vehicles.

        Args:
            ctx (dict): The context information.
        """
        # You can also make decisions for specific vehicles, e.g., only let vehicles near the AV make decisions
        # Cooperative decision making is also possible, e.g., let the AV and the BV make decisions together

        # by default, all vehicles in the vehicle list will make decisions
        if "terasim_controlled_vehicle_ids" in ctx:
            control_command_and_info_veh = {
                veh.id: veh.make_decision()
                for veh in self.vehicle_list
                if veh.id in ctx["terasim_controlled_vehicle_ids"]
            }
        else:
            control_command_and_info_veh = {
                veh.id: veh.make_decision() for veh in self.vehicle_list
            }
        if "terasim_controlled_vulnerable_road_user_ids" in ctx:
            control_command_and_info_vru = {
                vru.id: vru.make_decision()
                for vru in self.vulnerable_road_user_list
                if vru.id in ctx["terasim_controlled_vulnerable_road_user_ids"]
            }
        else:
            control_command_and_info_vru = {
                vru.id: vru.make_decision() for vru in self.vulnerable_road_user_list
            }
        control_command_dict = addict.Dict()
        control_command_dict[AgentType.VEHICLE] = addict.Dict(
            {
                veh_id: command_and_info[0]
                for veh_id, command_and_info in control_command_and_info_veh.items()
            }
        )
        control_command_dict[AgentType.VULNERABLE_ROAD_USER] = addict.Dict(
            {
                vru_id: command_and_info[0]
                for vru_id, command_and_info in control_command_and_info_vru.items()
            }
        )
        info_dict = addict.Dict()
        info_dict[AgentType.VEHICLE] = addict.Dict(
            {
                veh_id: command_and_info[1]
                for veh_id, command_and_info in control_command_and_info_veh.items()
            }
        )
        info_dict[AgentType.VULNERABLE_ROAD_USER] = addict.Dict(
            {
                vru_id: command_and_info[1]
                for vru_id, command_and_info in control_command_and_info_vru.items()
            }
        )
        return control_command_dict, info_dict

    def execute_control_commands(self, control_commands: dict):
        """Execute the control commands of all vehicles and vulnerable road users.
        
        Args:
            control_commands (dict): The control commands for all vehicles and vulnerable road users.
        """
        # Execute vehicle commands - only for vehicles that exist in both command list and environment
        vehicle_commands = control_commands[AgentType.VEHICLE]
        valid_vehicle_ids = set(vehicle_commands.keys()) & set(self.vehicle_list.keys())
        for veh_id in valid_vehicle_ids:
            self.vehicle_list[veh_id].apply_control(vehicle_commands[veh_id])
        
        # Execute VRU commands - only for VRUs that exist in both command list and environment  
        vru_commands = control_commands[AgentType.VULNERABLE_ROAD_USER]
        valid_vru_ids = set(vru_commands.keys()) & set(self.vulnerable_road_user_list.keys())
        for vru_id in valid_vru_ids:
            self.vulnerable_road_user_list[vru_id].apply_control(vru_commands[vru_id])

    def should_continue_simulation(self):
        """Check whether the simulation has ends, return False or Dict (including reason and info) to stop the simulation. Or return True to continue the simulation.

        Returns:
            bool or dict: True if the simulation should continue, False or dict if the simulation should stop
        """
        # By default, the simulation will stop when all vehicles leave the network
        if self.simulator.get_vehicle_min_expected_number() == 0:
            return False

        # You can also define your own termination condition, e.g., when the AV reaches the destination,
        # when collisions between AV and BV happen, etc.
        some_condition = False
        if some_condition:
            return dict(reason="All Vehicles Left", info={})

        # Otherwise return True to continue simulation
        return True
