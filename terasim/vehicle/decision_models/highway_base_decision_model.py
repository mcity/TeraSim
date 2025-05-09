from bidict import bidict
import numpy as np

from terasim.agent.agent_decision_model import AgentDecisionModel


class HighwayBaseDecisionModel(AgentDecisionModel):
    longi_safety_buffer, lateral_safety_buffer = 2, 2
    v_low, v_high, r_low, r_high, rr_low, rr_high, acc_low, acc_high = (
        20,
        40,
        0,
        115,
        -10,
        8,
        -4,
        2,
    )
    acc_resolution = 0.2
    LENGTH = 5
    ACTION_STEP = 1.0
    num_acc = int(1 + ((acc_high - acc_low) / acc_resolution))
    AV_acc_low, AV_acc_high, AV_acc_step = -4, 2, 0.2
    num_AV_acc = int((AV_acc_high - AV_acc_low) / AV_acc_step + 1)
    AV_acc_to_idx_dic = bidict()
    for i in range(num_AV_acc):
        AV_acc_to_idx_dic[
            list(np.arange(AV_acc_low, AV_acc_high + AV_acc_step, AV_acc_step))[i]
        ] = i
    acc_to_idx_dic = bidict()
    for m in range(num_acc):
        acc_to_idx_dic[list(np.linspace(acc_low, acc_high, num=num_acc))[m]] = m

    def step(self):
        """Store ego vehicle information.
        """
        self.ego_info = self.vehicle.observation.information["Ego"]

    @staticmethod
    def _check_longitudinal_safety(obs, pdf_array, lateral_result=None, AV_flag=False):
        """Check longitudinal safety for vehicle.

        Args:
            obs (dict): Processed observation of the vehicle.
            pdf_array (list(float)): Old possibility distribution of the maneuvers.
            lateral_result (list(float), optional): Possibility distribution of the lateral maneuvers. Defaults to None.
            AV_flag (bool, optional): Check whether the vehicle is the AV. Defaults to False.

        Returns:
            list(float): New possibility distribution of the maneuvers after checking the longitudinal direction.
        """
        ego_info = obs["Ego"]
        f_veh_info = obs["Lead"]
        safety_buffer = HighwayBaseDecisionModel.longi_safety_buffer
        for i in range(len(pdf_array) - 1, -1, -1):
            if AV_flag:
                acc = HighwayBaseDecisionModel.AV_acc_to_idx_dic.inverse[i]
            else:
                acc = HighwayBaseDecisionModel.acc_to_idx_dic.inverse[i]
            if f_veh_info is not None:
                rr = f_veh_info["velocity"] - ego_info["velocity"]
                r = f_veh_info["distance"]
                criterion_1 = rr + r + 0.5 * (HighwayBaseDecisionModel.acc_low - acc)
                self_v_2, f_v_2 = max(
                    ego_info["velocity"] + acc, HighwayBaseDecisionModel.v_low
                ), max(
                    (f_veh_info["velocity"] + HighwayBaseDecisionModel.acc_low),
                    HighwayBaseDecisionModel.v_low,
                )
                dist_r = (self_v_2**2 - HighwayBaseDecisionModel.v_low**2) / (
                    2 * abs(HighwayBaseDecisionModel.acc_low)
                )
                dist_f = (f_v_2**2 - HighwayBaseDecisionModel.v_low**2) / (
                    2 * abs(HighwayBaseDecisionModel.acc_low)
                ) + HighwayBaseDecisionModel.v_low * (
                    f_v_2 - self_v_2
                ) / HighwayBaseDecisionModel.acc_low
                criterion_2 = criterion_1 - dist_r + dist_f
                if criterion_1 <= safety_buffer or criterion_2 <= safety_buffer:
                    pdf_array[i] = 0
                else:
                    break

        # Only set the decelerate most when none of lateral is OK.
        if lateral_result is not None:
            lateral_feasible = lateral_result[0] or lateral_result[2]
        else:
            lateral_feasible = False
        if np.sum(pdf_array) == 0 and not lateral_feasible:
            pdf_array[0] = 1 if not AV_flag else np.exp(-2)
            return pdf_array

        if AV_flag:
            new_pdf_array = pdf_array
        else:
            new_pdf_array = pdf_array / np.sum(pdf_array)
        return new_pdf_array

    @staticmethod
    def _check_lateral_safety(obs, pdf_array, AV_flag=False):
        """Check the lateral safety of the vehicle.

        Args:
            obs (dict): Processed information of vehicle observation.
            pdf_array (list): Old possibility distribution of the maneuvers.
            AV_flag (bool, optional): Check whether the vehicle is the AV. Defaults to False.

        Returns:
            list: New possibility distribution of the maneuvers after checking the lateral direction.
        """
        AV_observation = obs
        f0, r0 = AV_observation["LeftLead"], AV_observation["LeftFoll"]
        f2, r2 = AV_observation["RightLead"], AV_observation["RightFoll"]
        AV_info = AV_observation["Ego"]
        lane_change_dir = [0, 2]
        nearby_vehs = [[f0, r0], [f2, r2]]
        safety_buffer = HighwayBaseDecisionModel.lateral_safety_buffer
        ### need to change when considering more than 3 lanes
        if not obs["Ego"]["could_drive_adjacent_lane_right"]:
            pdf_array[2] = 0
        elif not obs["Ego"]["could_drive_adjacent_lane_left"]:
            pdf_array[0] = 0
        for lane_index, nearby_veh in zip(lane_change_dir, nearby_vehs):
            if pdf_array[lane_index] != 0:
                f_veh, r_veh = nearby_veh[0], nearby_veh[1]
                if f_veh is not None:
                    rr = f_veh["velocity"] - AV_info["velocity"]
                    r = f_veh["distance"]
                    dis_change = (
                        rr * HighwayBaseDecisionModel.ACTION_STEP
                        + 0.5
                        * HighwayBaseDecisionModel.acc_low
                        * (HighwayBaseDecisionModel.ACTION_STEP**2)
                    )
                    r_1 = r + dis_change  # 1s
                    rr_1 = (
                        rr + HighwayBaseDecisionModel.acc_low * HighwayBaseDecisionModel.ACTION_STEP
                    )

                    if r_1 <= safety_buffer or r <= safety_buffer:
                        pdf_array[lane_index] = 0
                    elif rr_1 < 0:
                        self_v_2, f_v_2 = max(
                            AV_info["velocity"], HighwayBaseDecisionModel.v_low
                        ), max(
                            (f_veh["velocity"] + HighwayBaseDecisionModel.acc_low),
                            HighwayBaseDecisionModel.v_low,
                        )
                        dist_r = (self_v_2**2 - HighwayBaseDecisionModel.v_low**2) / (
                            2 * abs(HighwayBaseDecisionModel.acc_low)
                        )
                        dist_f = (f_v_2**2 - HighwayBaseDecisionModel.v_low**2) / (
                            2 * abs(HighwayBaseDecisionModel.acc_low)
                        ) + HighwayBaseDecisionModel.v_low * (
                            f_v_2 - self_v_2
                        ) / HighwayBaseDecisionModel.acc_low
                        r_2 = r_1 - dist_r + dist_f
                        if r_2 <= safety_buffer:
                            pdf_array[lane_index] = 0
                if r_veh is not None:
                    rr = AV_info["velocity"] - r_veh["velocity"]
                    r = r_veh["distance"]
                    dis_change = (
                        rr * HighwayBaseDecisionModel.ACTION_STEP
                        - 0.5
                        * HighwayBaseDecisionModel.acc_high
                        * (HighwayBaseDecisionModel.ACTION_STEP**2)
                    )
                    r_1 = r + dis_change
                    rr_1 = (
                        rr
                        - HighwayBaseDecisionModel.acc_high * HighwayBaseDecisionModel.ACTION_STEP
                    )
                    if r_1 <= safety_buffer or r <= safety_buffer:
                        pdf_array[lane_index] = 0
                    elif rr_1 < 0:
                        self_v_2, r_v_2 = min(
                            AV_info["velocity"], HighwayBaseDecisionModel.v_high
                        ), min(
                            (r_veh["velocity"] + HighwayBaseDecisionModel.acc_high),
                            HighwayBaseDecisionModel.v_high,
                        )
                        dist_r = (r_v_2**2 - HighwayBaseDecisionModel.v_low**2) / (
                            2 * abs(HighwayBaseDecisionModel.acc_low)
                        )
                        dist_f = (self_v_2**2 - HighwayBaseDecisionModel.v_low**2) / (
                            2 * abs(HighwayBaseDecisionModel.acc_low)
                        ) + HighwayBaseDecisionModel.v_low * (
                            -r_v_2 + self_v_2
                        ) / HighwayBaseDecisionModel.acc_low
                        r_2 = r_1 - dist_r + dist_f
                        if r_2 <= safety_buffer:
                            pdf_array[lane_index] = 0
        if np.sum(pdf_array) == 0:
            return np.array([0, 1, 0])

        if AV_flag:
            new_pdf_array = pdf_array
        else:
            new_pdf_array = pdf_array / np.sum(pdf_array)
        return new_pdf_array
