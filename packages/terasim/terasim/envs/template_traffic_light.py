from typing import Union

from terasim.envs.base import BaseEnv
from terasim.traffic_light.traffic_light import TrafficLightList


class EnvTrafficLightTemplate(BaseEnv):
    def __init__(self, vehicle_factory, tls_factory, info_extractor):
        """Initialize the base testing environment with control of traffic lights.

        Args:
            vehicle_factory (VehicleFactory): The vehicle factory.
            tls_factory (TrafficLightFactory): The traffic light factory.
            info_extractor (InfoExtractor): The info extractor.
        """
        self.tls_list = TrafficLightList({})
        self.tls_factory = tls_factory
        super().__init__(vehicle_factory, info_extractor)

    def on_start(self, ctx) -> bool:
        """Function to be called when the simulation starts.
        """
        pass

    def on_step(self, ctx) -> Union[bool, dict]:
        """Function to be called at each simulation step.

        Args:
            ctx (dict): The context information.
        """
        # Make decisions and execute commands
        control_cmds, infos = self.make_decisions()
        self.execute_control_commands(control_cmds)

        # Simulation stop c`heck
        return self.should_continue_simulation()

    def on_stop(self, ctx) -> bool:
        """Function to be called when the simulation stops.
        """
        pass

    def make_decisions(self):
        """Make decisions for all vehicles and traffic lights.
        """
        # You can also make decisions for specific vehicles, e.g., only let vehicles near the AV make decisions
        # Cooperative decision making is also possible, e.g., let the AV and the BV make decisions together

        # by default, all vehicles in the vehicle list will make decisions
        control_command_and_info = dict()
        control_command_and_info["veh"] = {veh.id: veh.make_decision() for veh in self.vehicle_list}
        control_command_and_info["tls"] = {tls.id: tls.make_decision() for tls in self.tls_list}
        control_command_dict = dict()
        control_command_dict["veh"] = {
            veh_id: command_and_info[0]
            for veh_id, command_and_info in control_command_and_info["veh"].items()
        }
        control_command_dict["tls"] = {
            tls_id: command_and_info[0]
            for tls_id, command_and_info in control_command_and_info["tls"].items()
        }
        info_dict = dict()
        info_dict["veh"] = {
            veh_id: command_and_info[1]
            for veh_id, command_and_info in control_command_and_info["veh"].items()
        }
        info_dict["tls"] = {
            tls_id: command_and_info[1]
            for tls_id, command_and_info in control_command_and_info["tls"].items()
        }
        return control_command_dict, info_dict

    def execute_control_commands(self, control_commands: dict):
        """Execute the control commands of all vehicles and traffic lights.
        """
        for veh_id, command in control_commands["veh"].items():
            self.vehicle_list[veh_id].apply_control(command)
        for tls_id, command in control_commands["tls"].items():
            self.tls_list[tls_id].apply_control(command)

    def should_continue_simulation(self):
        """Check whether the simulation should end, return False or Dict (including reason and info) to stop the simulation. Or return True to continue the simulation.

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

    ########## These methods are those hooked into the pipelines ##########

    # TODO: remove the simulator arguments in these hooks
    def _step(self, simulator, ctx) -> bool:
        """The step function that will be called by the simulator.

        Args:
            simulator (Simulator): The simulator object.
            ctx (dict): The context information.

        Returns:
            bool: True if the simulation should continue.
        """
        self._maintain_all_vehicles(ctx)
        self._maintain_all_tls(ctx)
        # Then call custom env defined step
        step_result = self.on_step(ctx)

        # If custom env requested to stop, log some of the information
        if isinstance(step_result, bool):
            if step_result:
                return True
            else:
                self._request_termination("Simulation ends normally", None)
                return False
        elif isinstance(step_result, dict):
            self._request_termination(step_result["reason"], step_result["info"])
            return False
        else:
            raise TypeError("The output of a step should be a boolean or a dictionary")

    ########## Other private utility functions that should not be directly called by custom env

    def _maintain_all_tls(self, ctx):
        """Maintain the traffic light list.
        
        Args:
            ctx (dict): The context information.
        """
        if "terasim_controlled_traffic_light_ids" in ctx:
            realtime_tlsID_set = set(ctx["terasim_controlled_traffic_light_ids"])
        else:
            realtime_tlsID_set = set(self.simulator.get_tlsID_list())

        tlsID_set = set(self.tls_list.keys())
        if tlsID_set != realtime_tlsID_set:
            for tlsID in realtime_tlsID_set:
                if tlsID not in tlsID_set:
                    tls = self._add_tls_to_env(tlsID)
                    tls._install()
            for tlsID in tlsID_set:
                if tlsID not in realtime_tlsID_set:
                    self._remove_tls_from_env(tlsID)

    def _add_tls_to_env(self, tls_id_list):
        """Add traffic lights from tls_id_list.

        Args:
            tls_id_list (list(str)): List of traffic lights IDs needed to be inserted.

        Returns:
            TrafficLight: The traffic light object.
        """
        single_input = not isinstance(tls_id_list, list)
        if single_input:
            tls_id_list = [tls_id_list]

        output = []
        for tls_id in tls_id_list:
            tls = self.tls_factory.create_traffic_light(tls_id, self.simulator)
            self.tls_list.add_trafficlight(tls)
            output.append(tls)
        return output[0] if single_input else output

    def _remove_tls_from_env(self, tls_id_list):
        """Delete traffic lights in tls_id_list.

        Args:
            tls_id_list (list(str)): List of traffic light IDs needed to be deleted.
        """
        if not isinstance(tls_id_list, list):
            tls_id_list = [tls_id_list]
        for tls_id in tls_id_list:
            if tls_id in self.tls_list:
                self.tls_list.pop(tls_id)._uninstall()
