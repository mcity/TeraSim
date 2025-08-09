import addict

from terasim.envs.base import BaseEnv


class EnvTemplate(BaseEnv):
    """This Env provides a basic Env implementation.

    Env developers can derived from this class or build their own implementations directly on BaseEnv
    """
    def on_start(self, ctx):
        """Function to be called when the simulation starts.

        Args:
            ctx (dict): The context information.
        """
        # your initialization (vehicle position, etc.), for example:
        # `self.add_vehicle(veh_id="AV", route="route_0", lane_id="0to1_0", position=100, speed=10)`
        pass

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
            control_command_and_info = {
                veh.id: veh.make_decision()
                for veh in self.vehicle_list
                if veh.id in ctx["terasim_controlled_vehicle_ids"]
            }
        else:
            control_command_and_info = {veh.id: veh.make_decision() for veh in self.vehicle_list}
        control_command_dict = addict.Dict(
            {
                veh_id: command_and_info[0]
                for veh_id, command_and_info in control_command_and_info.items()
            }
        )
        info_dict = addict.Dict(
            {
                veh_id: command_and_info[1]
                for veh_id, command_and_info in control_command_and_info.items()
            }
        )
        return control_command_dict, info_dict

    def execute_control_commands(self, control_commands: dict):
        """Execute the control commands of all vehicles.
        
        Args:
            control_commands (dict): The control commands of all vehicles.
        """
        for veh_id, command in control_commands.items():
            self.vehicle_list[veh_id].apply_control(command)

    def on_stop(self, ctx) -> bool:
        """Function to be called when the simulation stops.

        Args:
            ctx (dict): The context information.
        """
        pass

    def should_continue_simulation(self):
        """Check whether the simulation has ends, return False or Dict (including reason and info) to stop the simulation. Or return True to continue the simulation.

        Returns:
            bool: True if the simulation should continue.
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
