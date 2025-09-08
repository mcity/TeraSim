from addict import Dict
import json
from loguru import logger
import numpy as np
import os

from terasim.overlay import profile, traci
from terasim.params import AgentType
import terasim.utils as utils

from .nde import NDE

from ..utils import (
    add_avoid_accept_collision_command,
    adversarial_hook,
    apply_collision_avoidance,
    CommandType, 
    get_environment_avoidability,
    get_environment_criticality,
    get_environment_maneuver_challenge,
    get_nde_cmd_from_cmd_info,
    modify_nde_cmd_veh_using_avoidability,
    predict_environment_future_trajectory, 
    remove_collision_avoidance_command_using_avoidability,
    StaticAdversityManager,
    update_control_cmds_from_predicted_trajectory,
    update_nde_cmd_to_vehicle_cmd_info,
)

IS_MAGNITUDE_DEFAULT = 20
IS_MAGNITUDE_MULTIPLIER = 40
IS_MAGNITUDE_MAPPING = {
    "roundabout": "IS_MAGNITUDE_ROUNDABOUT",
    "highway": "IS_MAGNITUDE_HIGHWAY",
    "intersection": "IS_MAGNITUDE_INTERSECTION",
}


BaseEnv = NDE  # SafeTestNDE


class NADE(BaseEnv):
    def on_start(self, ctx):
        """Start the Naturalistic and Adversarial Driving Environment (NADE).

        Args:
            ctx (dict): Context dictionary. 

        Returns:
            bool: Flag to indicate if the simulation should continue.
        """
        self.importance_sampling_weight = 1.0
        self.max_importance_sampling_prob = 5e-2
        self.unavoidable_collision_prob_factor = (
            3e-4  # the factor to reduce the probability of the anavoidable collision
        )
        self.log = Dict()
        # initialize the static adversities
        if "adversity_cfg" in self.configuration and "static" in self.configuration.adversity_cfg:
            self.static_adversity = StaticAdversityManager(self.configuration.adversity_cfg.static)
            # self.static_adversity.initialize(time=utils.get_time())
        else:
            self.static_adversity = None
        self.distance_info = Dict({"before": self.update_distance(), "after": Dict()})
        self.allow_NADE_IS = True
        self.latest_IS_time = -1
        self.centered_veh_id = None
        on_start_result = super().on_start(ctx)
        return on_start_result
    
    def preparation(self):
        """Prepare for the NADE step."""
        if self.static_adversity is not None:
            self.static_adversity.update(time=utils.get_time())
        self.distance_info.after.update(self.update_distance())
        self.record.final_time = utils.get_time()
        self.cache_history_tls_data()
    
    @profile
    def NDE_decision(self, ctx):
        """Make NDE decisions for all vehicles and VRUs.

        Args:
            ctx (dict): Context dictionary.

        Returns:
            dict: Environment command information.
            dict: Environment observation.
            bool: Flag to indicate if the simulation should continue.
        """  

        _, env_command_information = super().make_decisions(ctx)
        env_observation = self.get_env_observation(ctx) # get the observation of the environment
        (
            env_command_information,
            env_observation,
            should_continue_simulation_flag,
        ) = self.executeMove(ctx, env_command_information, env_observation) # move the simulation half step forward and update useful information
        return env_command_information, env_observation, should_continue_simulation_flag
    
    def respond_to_emergency_vehicle(self):
        """
        Respond to the emergency vehicle.
        """
        # get all vehicles that has the vclass emergency
        emergency_vehicle_list = traci.vehicle.getIDList()
        for vehicle_id in emergency_vehicle_list:
            if traci.vehicle.getVehicleClass(vehicle_id) == "emergency":
                # get the lane id of the vehicle
                edge_id = traci.vehicle.getRoadID(vehicle_id)
                # get the lane position of the vehicle
                emergency_lane_position = traci.vehicle.getLanePosition(vehicle_id)
                emergency_lane_index = traci.vehicle.getLaneIndex(vehicle_id)
                # get all vehicles that are on the same edge
                same_edge_vehicle_list = traci.edge.getLastStepVehicleIDs(edge_id)
                for same_edge_vehicle_id in same_edge_vehicle_list:
                    same_edge_vehicle_lane_position = traci.vehicle.getLanePosition(same_edge_vehicle_id)
                    same_edge_vehicle_lane_index = traci.vehicle.getLaneIndex(same_edge_vehicle_id)
                    if emergency_lane_position < same_edge_vehicle_lane_position: # the vehicle is in front of the emergency vehicle
                        # Set lane change parameters to be very cooperative for emergency vehicle
                        traci.vehicle.setLateralAlignment(same_edge_vehicle_id, "center")
                        # traci.vehicle.setLaneChangeMode(same_edge_vehicle_id, 0b000000000000) # Disable all autonomous changes
                        traci.vehicle.setParameter(same_edge_vehicle_id, "lcSpeedGain", "0") # No speed gain from lane changes  
                        traci.vehicle.setParameter(same_edge_vehicle_id, "lcCooperative", "1.0") # Maximum cooperation
                        traci.vehicle.setParameter(same_edge_vehicle_id, "lcKeepRight", "0") # Disable keep right
                        if same_edge_vehicle_lane_index == emergency_lane_index:
                            can_change_left = traci.vehicle.couldChangeLane(same_edge_vehicle_id, 1) 
                            can_change_right = traci.vehicle.couldChangeLane(same_edge_vehicle_id, -1)
                            if can_change_right:
                                traci.vehicle.changeLaneRelative(same_edge_vehicle_id, -1, 1)
                            elif can_change_left:
                                traci.vehicle.changeLaneRelative(same_edge_vehicle_id, 1, 1)
                            else:
                                # make the vehicle change sublane to the center at next time step
                                offset = traci.vehicle.getLateralLanePosition(same_edge_vehicle_id)
                                traci.vehicle.changeSublane(same_edge_vehicle_id, -offset)
                        elif same_edge_vehicle_lane_index > emergency_lane_index: # the vehicle is in the left lane of the emergency vehicle
                            traci.vehicle.changeLaneRelative(same_edge_vehicle_id, 1, 1) # change to the left lane
                        else: # the vehicle is in the right lane of the emergency vehicle
                            traci.vehicle.changeLaneRelative(same_edge_vehicle_id, -1, 1) # change to the right lane
            else:
                traci.vehicle.setLateralAlignment(vehicle_id, "center")
        return True
    
    @profile
    def NADE_decision_and_control(self, env_command_information, env_observation):
        """
        Make NADE decision, includes the modification of NDD distribution according to avoidability, and excute the control commands.

        Args:
            env_command_information (dict): Environment command information.
            env_observation (dict): Environment observation.
        """
        # Make NADE decision, includes the modification of NDD distribution according to avoidability
        (
            nade_control_commands,
            env_command_information,
            weight,
            env_future_trajectory,
            _,
            _,
        ) = self.NADE_decision(
            env_command_information, env_observation
        )
        self.importance_sampling_weight *= weight  # update weight by adversity
        nade_control_commands, env_command_information, weight, self.record = apply_collision_avoidance(
            env_future_trajectory, env_command_information, nade_control_commands, self.record
        ) # consider the collision avoidance
        self.importance_sampling_weight *= (
            weight  
        ) # update weight by collision avoidance
        nade_control_commands = update_control_cmds_from_predicted_trajectory(
            nade_control_commands, env_future_trajectory
        ) # update the control commands according to the predicted trajectory
        if hasattr(self, "nnde_make_decisions"): # leave the API for the NeuralNDE
            nnde_control_commands, _ = self.nnde_make_decisions()
            nade_control_commands = self.merge_NADE_NeuralNDE_control_commands(
                nade_control_commands, nnde_control_commands
            )
        
        # Execute the control commands
        self.refresh_control_commands_state() # refresh the control status of all agents
        self.execute_control_commands(nade_control_commands) # execute the control commands
        self.record_step_data(env_command_information) # record the step data

    @profile
    def on_step(self, ctx):
        """Step NADE.

        Args:
            ctx (dict): Context dictionary.

        Returns:
            bool: Flag to indicate if the simulation should continue.
        """
        # Preparation
        self.preparation()

        # NADE Step 1. Make NDE decisions for all vehicles and vrus
        env_command_information, env_observation, should_continue_simulation_flag = self.NDE_decision(ctx)
        
        # NADE Step 2. Make NADE decision, includes the modification of NDD distribution according to avoidability, and excute the control commands
        self.NADE_decision_and_control(env_command_information, env_observation)
        self.try_insert_emergency_vehicle()
        # self.respond_to_emergency_vehicle()
        return should_continue_simulation_flag

    def try_insert_emergency_vehicle(self):
        """
        Try to insert an emergency vehicle into the simulation.
        """
        if "AV" in traci.vehicle.getIDList():
            if traci.vehicle.getLaneID("AV") == "EG_16_23_1_1" and traci.vehicle.getNextTLS("AV")[0][3] == 'G' and traci.vehicle.getLeader("AV") is None and "BV_police_1" not in traci.vehicle.getIDList():
                from terasim_nde_nade.adversity.static.stalled_object import create_emergency_police_type
                emergency_vehicle_typeid = create_emergency_police_type("POLICE")
                emergency_vehicle_id = "BV_police_1"
                emergency_vehicle_route_name = "emergency_vehicle_route"
                emergency_vehicle_route = ["EG_12_1_1", "EG_12_1_2", "EG_26_12_1"]
                traci.route.add(emergency_vehicle_route_name, emergency_vehicle_route)
                traci.vehicle.add(emergency_vehicle_id, routeID=emergency_vehicle_route_name, typeID=emergency_vehicle_typeid)
                traci.vehicle.setRouteID(emergency_vehicle_id, emergency_vehicle_route_name)
                traci.vehicle.setSpeed(emergency_vehicle_id, 10)
                traci.vehicle.setParameter(emergency_vehicle_id, "device.bluelight.reactiondist", str(90))
                traci.vehicle.setMaxSpeed(emergency_vehicle_id, 33)
                traci.vehicle.setSpeedMode(emergency_vehicle_id, 96)
                traci.vehicle.setLaneChangeMode(emergency_vehicle_id, 0)
                traci.vehicle.setSpeedFactor(emergency_vehicle_id, 1.5)
                
    def on_stop(self, ctx) -> bool:
        """Stop the NADE.

        Args:
            ctx (dict): Context dictionary.

        Returns:
            bool: Flag to indicate if the simulation is successfully stopped.
        """
        if self.log_flag:
            self.distance_info.after.update(self.update_distance())
            self.record.weight = self.importance_sampling_weight
            self.record.total_distance = self.calculate_total_distance()
            logger.info(f"total distance: {self.record.total_distance}")
            self.align_record_event_with_collision()
            moniotr_json_path = self.log_dir / "monitor.json"
            with open(moniotr_json_path, "w") as f:
                json.dump(self.record, f, indent=4, default=str)
        return super().on_stop(ctx)

    def calculate_total_distance(self):
        """Calculate the total distance of all vehicles in the simulation.

        Returns:
            float: Total distance of all vehicles in the simulation.
        """
        total_distance = 0
        for veh_id in self.distance_info.after:
            if veh_id not in self.distance_info.before:
                total_distance += self.distance_info.after[veh_id]
            else:
                total_distance += (
                    self.distance_info.after[veh_id] - self.distance_info.before[veh_id]
                )
        return total_distance

    # find the corresponding event that lead to the final result (e.g., collisions)
    def align_record_event_with_collision(self):
        """Align the record event with the collision event.
        """
        if self.record["finish_reason"] == "collision":
            agent_1_id = self.record["veh_1_id"]
            agent_2_id = self.record["veh_2_id"] if self.record["veh_2_id"] else self.record["vru_1_id"]
            # find if one of veh_1_id or veh_2_id is in the record.event_info adversarial_pair_dict
            for timestep in self.record.event_info:
                adversarial_pair_dict = self.record.event_info[
                    timestep
                ].adversarial_pair_dict
                adversarial_related_agent_set = set()
                for adversarial_agent_id in adversarial_pair_dict:
                    adversarial_related_agent_set.add(adversarial_agent_id)
                    adversarial_related_agent_set.update(
                        adversarial_pair_dict[adversarial_agent_id]
                    )
                if set([agent_1_id, agent_2_id]).issubset(adversarial_related_agent_set):
                    self.record.adversarial_event_time = timestep
                    self.record.adversarial_event_info = self.record.event_info[timestep]

    def merge_NADE_NeuralNDE_control_commands(
        self, nade_control_commands, NeuralNDE_control_commands
    ):
        """Merge the control commands from NADE and NeuralNDE.

        Args:
            nade_control_commands (dict): control commands from NADE.
            NeuralNDE_control_commands (dict): control commands from NeuralNDE.

        Returns:
            dict: Merged control commands.
        """
        # only replace the control commands that command_type is DEFAULT
        for veh_id in NeuralNDE_control_commands:
            if (
                veh_id in nade_control_commands[AgentType.VEHICLE]
                and nade_control_commands[AgentType.VEHICLE][veh_id].command_type == CommandType.DEFAULT
            ):
                nade_control_commands[AgentType.VEHICLE][veh_id] = NeuralNDE_control_commands[veh_id]
                nade_control_commands[AgentType.VEHICLE][veh_id] = NeuralNDE_control_commands[veh_id]
        return nade_control_commands

    def record_step_data(self, env_command_information):
        """Record the step data for all agents.

        Args:
            env_command_information (dict): Environment command information.
        """
        step_log = Dict()
        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for agent_id, agent_command_info in env_command_information[agent_type].items():
                maneuver_challenge = agent_command_info.get("maneuver_challenge", None)
                if maneuver_challenge and maneuver_challenge.get("adversarial", None):
                    step_log[agent_id]["maneuver_challenge"] = maneuver_challenge

                keys = ["avoidable", "conflict_vehicle_list", "mode"]
                step_log[agent_id].update(
                    {key: agent_command_info[key] for key in keys if agent_command_info.get(key)}
                )
                if step_log[agent_id].get("avoidable"):
                    step_log[agent_id].pop(
                        "avoidable"
                    )  # remove the avoidable key if it is True
        # pop the empty dict
        step_log = {k: v for k, v in step_log.items() if v}
        step_log = {
            "weight": self.importance_sampling_weight,
            "agent_log": step_log,
        }
        self.record.step_info[utils.get_time()] = step_log
        return step_log

    def update_distance(self):
        """Update the distance information for all vehicles.

        Returns:
            dict: Distance information for all vehicles.
        """
        distance_info_dict = Dict()
        for veh_id in traci.vehicle.getIDList():
            distance_info_dict[veh_id] = traci.vehicle.getDistance(veh_id)
        return distance_info_dict

    @profile
    def NADE_decision(self, env_command_information, env_observation):
        """NADE decision here.

        Args:
            env_command_information (dict): Environment command information.
            env_observation (dict): Environment observation.

        Returns:
            dict: NADE control commands.
            dict: Environment command information.
            float: Importance sampling weight.
            dict: Environment future trajectory.
            dict: Environment maneuver challenge.
            dict: Environment criticality information.
        """
        # Step 1. Predict the future trajectory for all agents.
        env_future_trajectory = predict_environment_future_trajectory(
            env_command_information, env_observation, self.simulator.sumo_net
        )

        # Step 2. Get maneuver challenge and mark the conflict vehicles and vrus.
        env_maneuver_challenge, env_command_information = get_environment_maneuver_challenge(
            env_future_trajectory,
            env_observation,
            env_command_information,
            centered_agent_set=self.excluded_agent_set,
        )

        # Step 3. Add collision avoidance and acceptance command for the victim vehicles.
        env_future_trajectory, env_command_information = add_avoid_accept_collision_command(
            env_future_trajectory,
            env_maneuver_challenge,
            env_observation,
            env_command_information,
            self.simulator.sumo_net,
        )

        # Step 4. Get collision avoidability by checking if the collision avoidance command is effective.
        env_avoidability, env_command_information = get_environment_avoidability(
            env_maneuver_challenge,
            env_future_trajectory,
            env_observation,
            env_command_information,
            centered_agent_set=self.excluded_agent_set,
        )
        # If collision avoidance is not effective, remove the collision avoidance command.
        env_command_information = remove_collision_avoidance_command_using_avoidability(
            env_observation, env_future_trajectory, env_command_information
        )
        # Update the NDD distribution for the unavoidable collision.
        (
            tmp_nde_control_commands_veh,
            env_command_information,
        ) = modify_nde_cmd_veh_using_avoidability(
            self.unavoidable_collision_prob_factor, env_maneuver_challenge, env_command_information
        )
        env_command_information = update_nde_cmd_to_vehicle_cmd_info(
            env_command_information, tmp_nde_control_commands_veh
        )
        # Extract the criticality information for each vehicle.
        env_criticality, env_command_information = get_environment_criticality(
            env_maneuver_challenge, env_command_information
        )

        # Step 5. Get the NDD distribution for each agent (after the collision avoidance command is added and the ndd probability is adjusted) and conduct importance sampling.
        nde_control_commands = {
            AgentType.VEHICLE: get_nde_cmd_from_cmd_info(
                env_command_information, AgentType.VEHICLE
            ),
            AgentType.VULNERABLE_ROAD_USER: get_nde_cmd_from_cmd_info(
                env_command_information, AgentType.VULNERABLE_ROAD_USER
            ),
        }
        self.step_epsilon = 1.0
        self.step_weight = 1.0
        if self.allow_NADE_IS:
            (
                nade_control_commands,
                env_command_information,
                weight,
                adversarial_flag,
            ) = self.NADE_importance_sampling(
                nde_control_commands, env_maneuver_challenge, env_command_information
            )
            if adversarial_flag:
                self.latest_IS_time = utils.get_time()
                self.allow_NADE_IS = False
        else:
            weight = 1.0
            nade_control_commands = Dict(
                {
                    AgentType.VEHICLE: {
                        veh_id: nde_control_commands[AgentType.VEHICLE][veh_id][
                            "normal"
                        ]
                        for veh_id in nde_control_commands[AgentType.VEHICLE]
                    },
                    AgentType.VULNERABLE_ROAD_USER: {
                        vru_id: nde_control_commands[
                            AgentType.VULNERABLE_ROAD_USER
                        ][vru_id]["normal"]
                        for vru_id in nde_control_commands[
                            AgentType.VULNERABLE_ROAD_USER
                        ]
                    },
                }
            )
            if utils.get_time() - self.latest_IS_time >= 5.5:
                self.allow_NADE_IS = True

        return (
            nade_control_commands,
            env_command_information,
            weight,
            env_future_trajectory,
            env_maneuver_challenge,
            env_criticality,
        )

    def NADE_importance_sampling(
        self,
        nde_control_commands,
        env_maneuver_challenge,
        env_command_information,
    ):
        """Select the NADE control commands based on the importance sampling theory.

        Args:
            nde_control_commands (dict): NDD distribution for each agent, including the normal, adversarial, collision_accept, and collision_avoid control commands, together with their probability.
            env_maneuver_challenge (dict): Environment maneuver challenge.
            env_command_information (dict): Environment command information.

        Returns:
            dict: NADE control commands.
            dict: Environment command information.
            float: Importance sampling.
            bool: Flag to indicate if the adversarial control command is selected.
        """
        # Initialize the importance sampling weight and epsilon.
        weight = 1.0
        epsilon = 1.0
        # Initialize the NADE control commands with the normal control commands.
        nade_control_commands = Dict(
            {
                AgentType.VEHICLE: {
                    veh_id: nde_control_commands[AgentType.VEHICLE][veh_id][
                        "normal"
                    ]
                    for veh_id in nde_control_commands[AgentType.VEHICLE]
                },
                AgentType.VULNERABLE_ROAD_USER: {
                    vru_id: nde_control_commands[AgentType.VULNERABLE_ROAD_USER][
                        vru_id
                    ]["normal"]
                    for vru_id in nde_control_commands[
                        AgentType.VULNERABLE_ROAD_USER
                    ]
                },
            }
        )
        adversarial_flag = False

        for agent_type in [AgentType.VEHICLE, AgentType.VULNERABLE_ROAD_USER]:
            for agent_id in env_maneuver_challenge[agent_type]:
                if agent_id in self.excluded_agent_set:
                    continue
                if adversarial_flag:
                    continue
                if env_maneuver_challenge[agent_type][agent_id].get("adversarial"):
                    ndd_normal_prob = nde_control_commands[agent_type][
                        agent_id
                    ].normal.prob
                    ndd_adversarial_prob = nde_control_commands[agent_type][
                        agent_id
                    ].adversarial.prob
                    assert (
                        ndd_normal_prob + ndd_adversarial_prob == 1
                    ), "The sum of the probabilities of the normal and adversarial control commands should be 1."

                    # get the importance sampling probability
                    IS_prob = self.get_IS_prob(
                        agent_id,
                        nde_control_commands[agent_type],
                        env_maneuver_challenge[agent_type],
                        env_command_information[agent_type],
                    )
                    epsilon = 1 - IS_prob

                    # update the importance sampling weight and the ITE control command
                    sampled_prob = np.random.uniform(0, 1)
                    if sampled_prob < IS_prob:  # select the negligece control command
                        weight *= ndd_adversarial_prob / IS_prob
                        nade_control_commands[agent_type][
                            agent_id
                        ] = nde_control_commands[agent_type][agent_id].adversarial
                        env_command_information[agent_type][agent_id]["mode"] = "adversarial"
                        if agent_type == AgentType.VEHICLE:
                            adversarial_hook(agent_id)
                        logger.info(
                            f"time: {utils.get_time()}, agent_id: {agent_id} select adversarial control command, IS_prob: {IS_prob}, ndd_prob: {ndd_adversarial_prob}, weight: {self.importance_sampling_weight}"
                        )
                        adversarial_flag = True
                    else:
                        weight *= ndd_normal_prob / (1 - IS_prob)
                        nade_control_commands[agent_type][
                            agent_id
                        ] = nde_control_commands[agent_type][agent_id]["normal"]
                        logger.trace(
                            f"time: {utils.get_time()}, agent_id: {agent_id} select normal control command, IS_prob: {IS_prob}, weight: {self.importance_sampling_weight}"
                        )
        self.step_epsilon = epsilon
        self.step_weight = weight
        return nade_control_commands, env_command_information, weight, adversarial_flag

    def get_IS_prob(
        self, agent_id, nde_control_commands, env_maneuver_challenge, env_command_information
    ):
        """Get the importance sampling probability.

        Args:
            agent_id (str): Agent id.
            nde_control_commands (dict): NDD distribution for each agent.
            env_maneuver_challenge (dict): Environment maneuver challenge.
            env_command_information (dict): Environment command information.

        Returns:
            float: Importance sampling probability.
        """
        if not env_maneuver_challenge[agent_id].get("adversarial"):
            raise ValueError("The vehicle is not in the adversarial mode.")
        
        if hasattr(self.configuration, "adversity_sampling_probability") and float(self.configuration.adversity_sampling_probability) > 0:
            return float(self.configuration.adversity_sampling_probability)
        
        IS_magnitude = IS_MAGNITUDE_DEFAULT
        try:
            predicted_collision_type = nde_control_commands[
                agent_id
            ].adversarial.info["predicted_collision_type"]

            # get the importance sampling magnitude according to the predicted collision type
            for collision_type, env_var in IS_MAGNITUDE_MAPPING.items():
                if collision_type in predicted_collision_type:
                    IS_magnitude = float(os.getenv(env_var, IS_magnitude))
                    break

            # if the vehicle is not avoidable, increase the importance sampling magnitude
            if not env_command_information[agent_id].get("avoidable", True):
                IS_magnitude *= IS_MAGNITUDE_MULTIPLIER
            # logger.trace(f"IS_magnitude: {IS_magnitude} for {collision_type}")
            # logger.trace(f"Original prob: {nde_control_commands[veh_id]["adversarial"].prob}")
            # final_is_prob = np.clip(
            #     nde_control_commands[veh_id]["adversarial"].prob * IS_magnitude,
            #     0,
            #     self.max_importance_sampling_prob,
            # )
            # logger.trace(f"final IS prob for veh_id: {final_is_prob}")

        except Exception as e:
            logger.critical(f"Error in getting the importance sampling magnitude: {e}")

        return np.clip(
            nde_control_commands[agent_id]["adversarial"].prob * IS_magnitude,
            0,
            self.max_importance_sampling_prob,
        )