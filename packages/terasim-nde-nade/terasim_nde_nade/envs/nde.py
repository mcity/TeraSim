from addict import Dict
from collections import deque
from loguru import logger
import numpy as np
import os

from terasim.envs.template_complete import EnvTemplateComplete
from terasim.overlay import traci
from terasim.params import AgentType
import terasim.utils as utils


class NDE(EnvTemplateComplete):
    def __init__(
        self,
        vehicle_factory,
        vru_factory,
        info_extractor,
        warmup_time_lb=900,
        warmup_time_ub=1200,
        run_time=300,
        log_flag=False,
        log_dir=None,
        configuration=None,
        *args,
        **kwargs,
    ):
        """Initialize the NDE environment.

        Args:
            vehicle_factory (VehicleFactory): Vehicle factory.
            vru_factory (VulnerableRoadUserFactory): Vulnerable road user factory.
            info_extractor (InfoExtractor): Information extractor.
            warmup_time_lb (int, optional): Lower bound of warmup time. Defaults to 900.
            warmup_time_ub (int, optional): Upper bound of warmup time. Defaults to 1200.
            run_time (int, optional): Running time. Defaults to 300.
            log_flag (bool, optional): Log flag. Defaults to False.
            log_dir (str, optional): Log directory. Defaults to None.
            configuration (dict, optional): Configuration. Defaults to None.
        """
        rng = np.random.default_rng()
        self.warmup_time = int(rng.integers(low=warmup_time_lb, high=warmup_time_ub))
        self.run_time = run_time
        logger.info(f"warmup_time: {self.warmup_time}, run_time: {self.run_time}")
        self.final_log = None
        self.log_dir = log_dir
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
        self.log_flag = log_flag
        self.tls_info_cache = {}
        self.history_length = 10
        self.record = Dict()
        self.step_epsilon = 1.0
        self.step_weight = 1.0
        self.excluded_agent_set = set()
        self.configuration = configuration
        super().__init__(vehicle_factory, vru_factory, info_extractor, *args, **kwargs)

    def on_start(self, ctx):
        """Start the simulation of NDE including simulation warm up.

        Args:
            ctx (dict): Simulation context information.

        Returns:
            bool: Flag to indicate if the simulation is started successfully.
        """
        self.sumo_warmup(self.warmup_time)
        return super().on_start(ctx)

    def get_env_observation(self, ctx):
        """Get the observation of the environment.

        Returns:
            dict: Observation of the environment.
        """
        env_observation = {
            AgentType.VEHICLE: {},
            AgentType.VULNERABLE_ROAD_USER: {},
        }
        if "terasim_controlled_vehicle_ids" in ctx:
            env_observation[AgentType.VEHICLE] = {
                veh.id: veh.observation
                for veh in self.vehicle_list.values()
                if veh.id in ctx["terasim_controlled_vehicle_ids"]
            }
        else:
            env_observation[AgentType.VEHICLE] = {
                veh.id: veh.observation for veh in self.vehicle_list.values()
            }
        if "terasim_controlled_vulnerable_road_user_ids" in ctx:
            env_observation[AgentType.VULNERABLE_ROAD_USER] = {
                vru.id: vru.observation
                for vru in self.vulnerable_road_user_list.values()
                if vru.id in ctx["terasim_controlled_vulnerable_road_user_ids"]
            }
        else:
            env_observation[AgentType.VULNERABLE_ROAD_USER] = {
                vru.id: vru.observation for vru in self.vulnerable_road_user_list.values()
            }
        return env_observation

    def executeMove(self, ctx, env_command_information=None, env_observation=None):
        """Execute the move of the environment, i.e., moving forward the SUMO simulation by half step and updating all the vehicles and vrus.
        Some of the vehicles and vrus may leave or enter the simulation.

        Args:
            ctx (dict): Simulation context information.
            env_command_information (dict, optional): Command information of the environment. Defaults to None.
            env_observation (dict, optional): Observation of the environment. Defaults to None.

        Returns:
            dict: Updated command information of the environment.
            dict: Updated observation of the environment.
            bool: Flag to indicate if the simulation should continue.
        """
        # Move half step forward, update all vehicles and vrus (some of them may leave or enter the simulation)
        # for veh_id in traci.vehicle.getIDList():
        #     traci.vehicle.setSpeed(veh_id, -1)
        traci.simulation.executeMove()
        self._maintain_all_vehicles(ctx)
        self._maintain_all_vulnerable_road_users(ctx)
        existing_vehicle_list = traci.vehicle.getIDList()
        existing_vru_list = traci.person.getIDList()

        env_command_information = {
            AgentType.VEHICLE: (
                {
                    veh_id: env_command_information[AgentType.VEHICLE][veh_id]
                    for veh_id in env_command_information[AgentType.VEHICLE]
                    if veh_id in existing_vehicle_list
                }
                if env_command_information[AgentType.VEHICLE]
                else {}
            ),
            AgentType.VULNERABLE_ROAD_USER: (
                {
                    vru_id: env_command_information[AgentType.VULNERABLE_ROAD_USER][vru_id]
                    for vru_id in env_command_information[AgentType.VULNERABLE_ROAD_USER]
                    if vru_id in existing_vru_list
                }
                if env_command_information[AgentType.VULNERABLE_ROAD_USER]
                else {}
            ),
        }
        env_observation = {
            AgentType.VEHICLE: (
                {
                    veh_id: env_observation[AgentType.VEHICLE][veh_id]
                    for veh_id in env_observation[AgentType.VEHICLE]
                    if veh_id in existing_vehicle_list
                }
                if env_observation[AgentType.VEHICLE]
                else {}
            ),
            AgentType.VULNERABLE_ROAD_USER: (
                {
                    vru_id: env_observation[AgentType.VULNERABLE_ROAD_USER][vru_id]
                    for vru_id in env_observation[AgentType.VULNERABLE_ROAD_USER]
                    if vru_id in existing_vru_list
                }
                if env_observation[AgentType.VULNERABLE_ROAD_USER]
                else {}
            ),
        }
        return env_command_information, env_observation, self.should_continue_simulation()

    def cache_history_tls_data(self, focus_tls_ids=None):
        """Cache the history traffic light state data.

        Args:
            focus_tls_ids (list, optional): List of traffic light ids to focus on. Defaults to None.
        """
        if not focus_tls_ids:
            focus_tls_ids = traci.trafficlight.getIDList()
        for tls_id in focus_tls_ids:
            if tls_id not in self.tls_info_cache:
                self.tls_info_cache[tls_id] = deque(maxlen=self.history_length + 1)
            current_time = traci.simulation.getTime()
            tls_state = traci.trafficlight.getRedYellowGreenState(tls_id)
            self.tls_info_cache[tls_id].append((current_time, tls_state))

    def sumo_warmup(self, warmup_time):
        """Warm up the SUMO simulation.

        Args:
            warmup_time (float): Warm up time.
        """
        # TODO: change vehicle type during the warmup time (might make warmup time longer)
        while True:
            while True:
                traci.simulationStep()
                if traci.simulation.getTime() > warmup_time:
                    break
            break
            # if traci.vehicle.getIDCount() > 2500:
            #     logger.warning(
            #         f"Too many vehicles in the simulation: {traci.vehicle.getIDCount()}, Restarting..."
            #     )
            #     traci.load(self.simulator.sumo_cmd[1:])
            # else:
            #     break
        self.record.warmup_vehicle_num = traci.vehicle.getIDCount()
        self._vehicle_in_env_distance("before")

    def on_step(self, ctx):
        """Step the simulation of NDE.

        Args:
            ctx (dict): Simulation context information.

        Returns:
            bool: Flag to indicate if the simulation should continue.
        """
        control_cmds, infos = self.make_decisions(ctx)
        self.refresh_control_commands_state()
        self.execute_control_commands(control_cmds)
        return self.should_continue_simulation()

    def refresh_control_commands_state(self):
        """Refresh the controlling status of all agents.
        """
        current_time = traci.simulation.getTime()
        for veh_id in self.vehicle_list.keys():
            self.vehicle_list[veh_id].controller._update_controller_status(
                veh_id, current_time
            )
        
        # Handle VRU removal - need to check before updating status to avoid dict modification during iteration
        vrus_to_remove = []
        for vru_id in self.vulnerable_road_user_list.keys():
            controller = self.vulnerable_road_user_list[vru_id].controller
            controller._update_controller_status(vru_id, current_time)
            
            # Check if VRU should be removed after adversity control ends
            if controller.used_to_be_busy:
                try:
                    traci.person.remove(vru_id)
                    vrus_to_remove.append(vru_id)
                    print(f"VRU {vru_id} removed after adversity completion")
                except Exception as e:
                    print(f"Failed to remove VRU {vru_id}: {e}")
        
        # Remove VRUs from environment list
        for vru_id in vrus_to_remove:
            if vru_id in self.vulnerable_road_user_list:
                self.vulnerable_road_user_list.pop(vru_id)._uninstall()

    def _vehicle_in_env_distance(self, mode):
        """Record the distance of all vehicles in the environment.

        Args:
            mode (str): Mode of the recording.
        """
        veh_id_list = traci.vehicle.getIDList()
        distance_dist = self._get_distance(veh_id_list)

    def _get_distance(self, veh_id_list):
        """Get the distance of all vehicles in the environment.

        Args:
            veh_id_list (list): List of vehicle ids.

        Returns:
            dict: Distance of all vehicles in the environment.
        """
        distance_dist = {veh_id: utils.get_distance(veh_id) for veh_id in veh_id_list}
        return distance_dist

    def should_continue_simulation(self):
        """Determine if the simulation should continue. The simulation should stop when collision happens or the running time exceeds the limit.

        Returns:
            bool: Flag to indicate if the simulation should continue.
        """
        return True
        num_colliding_vehicles = self.simulator.get_colliding_vehicle_number()
        self._vehicle_in_env_distance("after")
        if num_colliding_vehicles >= 2: # collision happens between two vehicles.
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            veh_2_id = colliding_vehicles[1]
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "veh_2_id": veh_2_id,
                    "veh_2_obs": self.vehicle_list[veh_2_id].observation,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False
        elif num_colliding_vehicles == 1: # collision happens between a vehicle and a vru.
            colliding_vehicles = self.simulator.get_colliding_vehicles()
            veh_1_id = colliding_vehicles[0]
            collision_objects = traci.simulation.getCollisions()
            if veh_1_id == collision_objects[0].collider:
                vru_1_id = collision_objects[0].victim
            elif veh_1_id == collision_objects[0].victim:
                vru_1_id = collision_objects[0].collider
            else:
                vru_1_id = None
            self.record.update(
                {
                    "veh_1_id": veh_1_id,
                    "veh_1_obs": self.vehicle_list[veh_1_id].observation,
                    "vru_1_id": vru_1_id,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "collision",
                }
            )
            return False
        elif utils.get_time() >= self.warmup_time + self.run_time: # running time exceeds the limit.
            self.record.update(
                {
                    "veh_1_id": None,
                    "veh_2_id": None,
                    "warmup_time": self.warmup_time,
                    "run_time": self.run_time,
                    "finish_reason": "timeout",
                }
            )
            return False
        return True
