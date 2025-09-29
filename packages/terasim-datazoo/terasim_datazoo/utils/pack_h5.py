# Licensed under the CC BY-NC 4.0 license (https://creativecommons.org/licenses/by-nc/4.0/)
from typing import Dict, List, Tuple
import numpy as np
from scipy.interpolate import interp1d
from . import transform_utils

# "agent/cmd"
# STATIONARY = 0;
# STRAIGHT = 1;
# STRAIGHT_LEFT = 2;
# STRAIGHT_RIGHT = 3;
# LEFT_U_TURN = 4;
# LEFT_TURN = 5;
# RIGHT_U_TURN = 6;
# RIGHT_TURN = 7;
N_AG_CMD = 8


def pack_episode_map(
    episode: Dict[str, np.ndarray],
    mp_id: List[int],
    mp_xyz: List[List[List[float]]],
    mp_type: List[int],
    mp_edge: List[List[int]],
    n_mp_data: int,
    n_nodes: int,
) -> int:
    """
    Args:
        mp_id: [polyline]
        mp_xyz: [polyline, points, xyz]
        mp_type: [polyline]
        mp_edge: [edge, 2]
    """
    episode["map/valid"] = np.zeros([n_mp_data, n_nodes], dtype=bool)
    episode["map/id"] = np.zeros([n_mp_data], dtype=np.int64) - 1
    episode["map/type"] = np.zeros([n_mp_data], dtype=np.int64)
    episode["map/pos"] = np.zeros([n_mp_data, n_nodes, 3], dtype=np.float32)
    episode["map/dir"] = np.zeros([n_mp_data, n_nodes, 3], dtype=np.float32)
    episode["map/edge"] = np.array(mp_edge)

    mp_counter = 0
    for i_pl in range(len(mp_id)):
        pl_pos = np.array(mp_xyz[i_pl])
        pl_dir = np.diff(pl_pos, axis=0)
        polyline_len = pl_dir.shape[0]
        polyline_cuts = np.linspace(0, polyline_len, polyline_len // n_nodes + 1, dtype=int, endpoint=False)
        num_cuts = len(polyline_cuts)
        for idx_cut in range(num_cuts):
            idx_start = polyline_cuts[idx_cut]
            if idx_cut + 1 == num_cuts:
                # last slice
                idx_end = polyline_len
            else:
                idx_end = polyline_cuts[idx_cut + 1]

            episode["map/valid"][mp_counter, : idx_end - idx_start] = True
            episode["map/pos"][mp_counter, : idx_end - idx_start] = pl_pos[idx_start:idx_end]
            episode["map/dir"][mp_counter, : idx_end - idx_start] = pl_dir[idx_start:idx_end]
            episode["map/type"][mp_counter] = mp_type[i_pl]
            episode["map/id"][mp_counter] = mp_id[i_pl]
            mp_counter += 1
    return mp_counter


def pack_episode_traffic_lights(
    episode: Dict[str, np.ndarray],
    step_current: int,
    tl_lane_state: List[List[float]],
    tl_lane_id: List[List[int]],
    tl_stop_point: List[List[List[float]]],
    pack_all: bool,
    pack_history: bool,
    n_tl_data: int,
) -> int:
    """
    Convert untracked traffic lights to tracks using unique lane_id.
    Args:
        tl_lane_state: [step, tl_lane]
        tl_lane_id: [step, tl_lane]
        tl_stop_point: [step, tl_lane, 3], x,y,z
    """
    data_tl_lane_id = np.zeros([n_tl_data], dtype=np.int64) - 1
    id2k = {}
    for k, _lane_id in enumerate(np.unique([x for l in tl_lane_id for x in l])):
        data_tl_lane_id[k] = _lane_id
        id2k[_lane_id] = k

    n_step = len(tl_lane_state)
    data_tl_lane_valid = np.zeros([n_tl_data, n_step], dtype=bool)
    data_tl_lane_state = np.zeros([n_tl_data, n_step], dtype=np.int64)
    data_tl_stop_pos = np.zeros([n_tl_data, 3], dtype=np.float32)

    for _step in range(n_step):
        for i in range(len(tl_lane_id[_step])):
            k = id2k[tl_lane_id[_step][i]]
            # assert not data_tl_lane_valid[
            #     k, _step
            # ]  # at each time step, each lane idx should appear only once.
            data_tl_lane_valid[k, _step] = True
            data_tl_lane_state[k, _step] = tl_lane_state[_step][i]
            if (data_tl_stop_pos[k] == 0).all():  # not set
                data_tl_stop_pos[k] = np.array(tl_stop_point[_step][i])
            else:  # already set
                assert np.isclose(data_tl_stop_pos[k], np.array(tl_stop_point[_step][i])).all()

    if pack_all:
        episode["tl_lane/id"] = data_tl_lane_id.copy()
        episode["tl_stop/pos"] = data_tl_stop_pos.copy()
        episode["tl_lane/valid"] = data_tl_lane_valid.copy()
        episode["tl_lane/state"] = data_tl_lane_state.copy()
    if pack_history:
        episode["history/tl_lane/id"] = data_tl_lane_id.copy()
        episode["history/tl_stop/pos"] = data_tl_stop_pos.copy()
        episode["history/tl_lane/valid"] = data_tl_lane_valid[:, : step_current + 1].copy()
        episode["history/tl_lane/state"] = data_tl_lane_state[:, : step_current + 1].copy()
    return len(id2k)


def pack_episode_agents(
    episode: Dict[str, np.ndarray],
    step_current: int,
    ag_id: List[int],
    ag_type: List[int],
    ag_state: List[List[List[float]]],
    ag_role: List[List[bool]],
    pack_all: bool,
    pack_history: bool,
    n_ag_data: int,
    n_ag_type: int = 3,
) -> None:
    """
    Args:
        ag_id: [agents]
        ag_type: [agents]
        ag_state: [agents, step, 10]; x,y,z,l,w,h,heading,vx,vy,valid
        ag_role: [agents, :], bool, [sdc=0]
    """
    n_step = len(ag_state[0])
    data_ag_valid = np.zeros([n_ag_data, n_step], dtype=bool)
    data_ag_pos = np.zeros([n_ag_data, n_step, 3], dtype=np.float32)
    data_ag_vel = np.zeros([n_ag_data, n_step, 2], dtype=np.float32)
    data_ag_spd = np.zeros([n_ag_data, n_step, 1], dtype=np.float32)
    data_ag_yaw_bbox = np.zeros([n_ag_data, n_step, 1], dtype=np.float32)

    data_ag_type = np.zeros([n_ag_data, n_ag_type], dtype=bool)
    data_ag_cmd = np.zeros([n_ag_data, N_AG_CMD], dtype=bool)
    data_ag_role = np.zeros([n_ag_data, len(ag_role[0])], dtype=bool)
    data_ag_size = np.zeros([n_ag_data, 3], dtype=np.float32)
    data_ag_goal = np.zeros([n_ag_data, 4], dtype=np.float32)
    data_ag_object_id = np.zeros([n_ag_data], dtype=np.int64) - 1

    for i in range(len(ag_id)):
        data_ag_type[i, ag_type[i]] = True
        data_ag_object_id[i] = ag_id[i]
        data_ag_role[i] = ag_role[i]

        length, width, height, count = 0.0, 0.0, 0.0, 0.0
        for k in range(n_step):
            if ag_state[i][k][9]:
                data_ag_pos[i, k] = np.array(ag_state[i][k][0:3])
                length += ag_state[i][k][3]
                width += ag_state[i][k][4]
                height += ag_state[i][k][5]
                data_ag_yaw_bbox[i, k, 0] = ag_state[i][k][6]
                data_ag_vel[i, k] = np.array(ag_state[i][k][7:9])
                data_ag_valid[i, k] = True
                count += 1

                spd_sign = np.sign(
                    np.cos(data_ag_yaw_bbox[i, k, 0]) * data_ag_vel[i, k, 0]
                    + np.sin(data_ag_yaw_bbox[i, k, 0]) * data_ag_vel[i, k, 1]
                )
                data_ag_spd[i, k, 0] = spd_sign * np.linalg.norm(data_ag_vel[i, k], axis=-1)

                # set goal as the last valid state [x,y,theta,v]
                data_ag_goal[i, 0] = data_ag_pos[i, k, 0]
                data_ag_goal[i, 1] = data_ag_pos[i, k, 1]
                data_ag_goal[i, 2] = data_ag_yaw_bbox[i, k, 0]
                data_ag_goal[i, 3] = data_ag_spd[i, k, 0]

        cmd = _classify_track(
            data_ag_valid[i, step_current:],
            data_ag_pos[i, step_current:, :2],
            data_ag_yaw_bbox[i, step_current:, 0],
            data_ag_spd[i, step_current:, 0],
        )
        data_ag_cmd[i, cmd] = True

        data_ag_size[i, 0] = length / count if count > 0 else 0.0
        data_ag_size[i, 1] = width / count if count > 0 else 0.0
        data_ag_size[i, 2] = height / count if count > 0 else 0.0

    # swap sdc to be the first agent
    sdc_track_index = np.where(data_ag_role[:, 0])[0][0]
    data_ag_valid[[0, sdc_track_index]] = data_ag_valid[[sdc_track_index, 0]]
    data_ag_pos[[0, sdc_track_index]] = data_ag_pos[[sdc_track_index, 0]]
    data_ag_vel[[0, sdc_track_index]] = data_ag_vel[[sdc_track_index, 0]]
    data_ag_spd[[0, sdc_track_index]] = data_ag_spd[[sdc_track_index, 0]]
    data_ag_yaw_bbox[[0, sdc_track_index]] = data_ag_yaw_bbox[[sdc_track_index, 0]]
    data_ag_object_id[[0, sdc_track_index]] = data_ag_object_id[[sdc_track_index, 0]]
    data_ag_type[[0, sdc_track_index]] = data_ag_type[[sdc_track_index, 0]]
    data_ag_role[[0, sdc_track_index]] = data_ag_role[[sdc_track_index, 0]]
    data_ag_size[[0, sdc_track_index]] = data_ag_size[[sdc_track_index, 0]]
    data_ag_cmd[[0, sdc_track_index]] = data_ag_cmd[[sdc_track_index, 0]]
    data_ag_goal[[0, sdc_track_index]] = data_ag_goal[[sdc_track_index, 0]]

    if pack_all:
        episode["agent/valid"] = data_ag_valid.copy()
        episode["agent/pos"] = data_ag_pos.copy()
        episode["agent/vel"] = data_ag_vel.copy()
        episode["agent/spd"] = data_ag_spd.copy()
        episode["agent/yaw_bbox"] = data_ag_yaw_bbox.copy()
        episode["agent/object_id"] = data_ag_object_id.copy()
        episode["agent/type"] = data_ag_type.copy()
        episode["agent/role"] = data_ag_role.copy()
        episode["agent/size"] = data_ag_size.copy()
        episode["agent/cmd"] = data_ag_cmd.copy()
        episode["agent/goal"] = data_ag_goal.copy()
    if pack_history:
        episode["history/agent/valid"] = data_ag_valid[:, : step_current + 1].copy()
        episode["history/agent/pos"] = data_ag_pos[:, : step_current + 1].copy()
        episode["history/agent/vel"] = data_ag_vel[:, : step_current + 1].copy()
        episode["history/agent/spd"] = data_ag_spd[:, : step_current + 1].copy()
        episode["history/agent/yaw_bbox"] = data_ag_yaw_bbox[:, : step_current + 1].copy()
        episode["history/agent/object_id"] = data_ag_object_id.copy()
        episode["history/agent/type"] = data_ag_type.copy()
        episode["history/agent/role"] = data_ag_role.copy()
        episode["history/agent/size"] = data_ag_size.copy()
        # no goal, no cmd
        invalid = ~(episode["history/agent/valid"].any(1))
        episode["history/agent/object_id"][invalid] = -1
        episode["history/agent/type"][invalid] = False
        episode["history/agent/size"][invalid] = 0
    return len(ag_id)


def center_at_sdc(
    episode: Dict[str, np.ndarray], step_current: int, rand_pos: float, rand_yaw: float
) -> Tuple[np.ndarray, float]:
    """episode
    # agent state
    "agent/valid": [N_AG_DATA, n_step],  # bool,
    "agent/pos": [N_AG_DATA, n_step, 3],  # float32
    "agent/vel": [N_AG_DATA, n_step, 2],  # float32, v_x, v_y
    "agent/yaw_bbox": [N_AG_DATA, n_step, 1],  # float32, yaw of the bbox heading
    "agent/goal": [N_AG_DATA, 4],  # float32: [x, y, theta, v]
    # map
    "map/valid": [N_MP_DATA, N_MP_PL_NODE],  # bool
    "map/pos": [N_MP_DATA, N_MP_PL_NODE, 3],  # float32
    "map/dir": [N_MP_DATA, N_MP_PL_NODE, 3],  # float32
    # traffic light
    "tl_lane/valid: [N_TL_DATA, n_step], bool
    "tl_stop/pos: [N_TL_DATA, 3], x,y,z
    """
    prefix = []
    if "agent/pos" in episode:
        prefix.append("")
    if "history/agent/valid" in episode:
        prefix.append("history/")

    sdc_center = episode[prefix[0] + "agent/pos"][0, step_current, :2].copy()
    sdc_yaw = episode[prefix[0] + "agent/yaw_bbox"][0, step_current, 0].copy()

    if rand_pos > 0:
        sdc_center[0] += np.random.uniform(-rand_pos, rand_pos)
        sdc_center[1] += np.random.uniform(-rand_pos, rand_pos)
    if rand_yaw > 0:
        sdc_yaw += np.random.uniform(-rand_yaw, rand_yaw)

    to_sdc_se3 = transform_utils.get_transformation_matrix(sdc_center, sdc_yaw)  # for points
    to_sdc_yaw = transform_utils.get_yaw_from_se2(to_sdc_se3)  # for vector
    to_sdc_so2 = transform_utils.get_so2_from_se2(to_sdc_se3)  # for angle

    # map
    episode["map/pos"][..., :2][episode["map/valid"]] = transform_utils.transform_points(
        episode["map/pos"][..., :2][episode["map/valid"]], to_sdc_se3
    )
    episode["map/dir"][..., :2][episode["map/valid"]] = transform_utils.transform_points(
        episode["map/dir"][..., :2][episode["map/valid"]], to_sdc_so2
    )

    for pf in prefix:
        # agent: pos, vel, yaw_bbox
        episode[pf + "agent/pos"][..., :2][episode[pf + "agent/valid"]] = transform_utils.transform_points(
            episode[pf + "agent/pos"][..., :2][episode[pf + "agent/valid"]], to_sdc_se3
        )
        episode[pf + "agent/vel"][episode[pf + "agent/valid"]] = transform_utils.transform_points(
            episode[pf + "agent/vel"][episode[pf + "agent/valid"]], to_sdc_so2
        )
        episode[pf + "agent/yaw_bbox"][episode[pf + "agent/valid"]] += to_sdc_yaw
        # traffic light: [step, tl, 3]
        key_tl = pf + "tl_stop/pos"
        if key_tl in episode:
            episode[key_tl][..., :2][episode[pf + "tl_lane/valid"].any(1)] = transform_utils.transform_points(
                episode[key_tl][..., :2][episode[pf + "tl_lane/valid"].any(1)], to_sdc_se3
            )
        if pf == "":
            # goal: x, y, theta
            goal_valid = episode["agent/valid"].any(axis=1)
            episode["agent/goal"][..., :2][goal_valid] = transform_utils.transform_points(
                episode["agent/goal"][..., :2][goal_valid], to_sdc_se3
            )
            episode["agent/goal"][..., 2][goal_valid] += to_sdc_yaw

    return sdc_center, sdc_yaw


def filter_episode_map(
    episode: Dict[str, np.ndarray], step_current: int, n_mp_h5: int, dist_thresh_mp: float, thresh_z: float
) -> None:
    """
    Args: episode
        "agent/valid": [N_AG_DATA, n_step], bool,
        "agent/pos": [N_AG_DATA, n_step, 3], float32
        "agent/role": [N_AG_DATA, :], bool, one hot
        "map/valid": [N_MP_DATA, N_MP_PL_NODE], bool
        "map/id": [N_MP_DATA], int, with -1
        "map/type": [N_MP_DATA], int, >= 0
        "map/pos": [N_MP_DATA, N_MP_PL_NODE, 3]
        "map/dir": [N_MP_DATA, N_MP_PL_NODE, 3]
    """
    # ! filter "map/" based on distance to relevant agents, based on the history only
    if "agent/valid" in episode:
        relevant_agent = episode["agent/role"].any(-1)
        agent_valid_relevant = episode["agent/valid"][relevant_agent, : step_current + 1]
        agent_pos_relevant = episode["agent/pos"][relevant_agent, : step_current + 1]
    elif "history/agent/valid" in episode:
        relevant_agent = episode["history/agent/role"].any(-1)
        agent_valid_relevant = episode["history/agent/valid"][relevant_agent]
        agent_pos_relevant = episode["history/agent/pos"][relevant_agent]
    agent_pos_relevant = agent_pos_relevant[agent_valid_relevant]  # [N, 3]

    xmin = agent_pos_relevant[:, 0].min()
    xmax = agent_pos_relevant[:, 0].max()
    ymin = agent_pos_relevant[:, 1].min()
    ymax = agent_pos_relevant[:, 1].max()
    x_thresh = max(xmax - xmin, dist_thresh_mp)
    y_thresh = max(ymax - ymin, dist_thresh_mp)

    old_map_valid = episode["map/valid"].copy()
    episode["map/valid"] = episode["map/valid"] & (episode["map/pos"][..., 0] > xmin - x_thresh).any(
        -1, keepdims=True
    )
    episode["map/valid"] = episode["map/valid"] & (episode["map/pos"][..., 0] < xmax + x_thresh).any(
        -1, keepdims=True
    )
    episode["map/valid"] = episode["map/valid"] & (episode["map/pos"][..., 1] > ymin - y_thresh).any(
        -1, keepdims=True
    )
    episode["map/valid"] = episode["map/valid"] & (episode["map/pos"][..., 1] < ymax + y_thresh).any(
        -1, keepdims=True
    )
    if thresh_z > 0:
        zmin = agent_pos_relevant[:, 2].min()
        zmax = agent_pos_relevant[:, 2].max()
        z_thresh = max(zmax - zmin, thresh_z)
        episode["map/valid"] = episode["map/valid"] & (episode["map/pos"][..., 2] > zmin - z_thresh).any(
            -1, keepdims=True
        )
        episode["map/valid"] = episode["map/valid"] & (episode["map/pos"][..., 2] < zmax + z_thresh).any(
            -1, keepdims=True
        )

    if episode["map/valid"].any(1).sum() < 10:
        # in waymo training 83954, agent and map z axis is off, filter using zmin/zmax will remove all polylines.
        print("something is wrong, check this episode out!")
        episode["map/valid"] = old_map_valid

    # ! filter "map/" that has too little valid entries
    mask_map_short = episode["map/valid"].sum(1) <= 3
    episode["map/valid"][mask_map_short] = False

    # ! still too many polylines: filter polylines based on distance to relevant agents
    while episode["map/valid"].any(1).sum() > n_mp_h5:
        mask_map_remain = episode["map/valid"].any(1)
        for i in range(episode["map/valid"].shape[0]):
            if mask_map_remain[i]:
                pl_pos = episode["map/pos"][i][episode["map/valid"][i]]
                close_to_agent = (
                    min(
                        np.linalg.norm(agent_pos_relevant - pl_pos[0], axis=1).min(),
                        np.linalg.norm(agent_pos_relevant - pl_pos[-1], axis=1).min(),
                    )
                    < dist_thresh_mp
                )
                if not close_to_agent:
                    episode["map/valid"][i] = False
                if episode["map/valid"].any(1).sum() == n_mp_h5:
                    break
        dist_thresh_mp = dist_thresh_mp * 0.5


def repack_episode_map(
    episode: Dict[str, np.ndarray], episode_reduced: Dict[str, np.ndarray], n_mp_h5: int, n_mp_type: int
) -> int:
    """
    Args: episode
        # map
        "map/valid": [N_MP_DATA, 20],  # bool
        "map/id": [N_MP_DATA],  # int, with -1
        "map/type": [N_MP_DATA],  # int, >= 0
        "map/pos": [N_MP_DATA, 20, 3]
        "map/dir": [N_MP_DATA, 20, 3]
    """
    n_mp_pl_node = episode["map/valid"].shape[1]
    episode_reduced["map/valid"] = np.zeros([n_mp_h5, n_mp_pl_node], dtype=bool)  # bool
    episode_reduced["map/type"] = np.zeros([n_mp_h5], dtype=np.int64)  # will be one_hot
    episode_reduced["map/pos"] = np.zeros([n_mp_h5, n_mp_pl_node, 3], dtype=np.float32)  # x,y,z
    episode_reduced["map/dir"] = np.zeros([n_mp_h5, n_mp_pl_node, 3], dtype=np.float32)  # x,y,z
    episode_reduced["map/id"] = np.zeros([n_mp_h5], dtype=np.int64) - 1

    map_valid_mask = episode["map/valid"].any(1)
    n_mp_valid = map_valid_mask.sum()
    episode_reduced["map/valid"][:n_mp_valid] = episode["map/valid"][map_valid_mask]
    episode_reduced["map/type"][:n_mp_valid] = episode["map/type"][map_valid_mask]
    episode_reduced["map/pos"][:n_mp_valid] = episode["map/pos"][map_valid_mask]
    episode_reduced["map/dir"][:n_mp_valid] = episode["map/dir"][map_valid_mask]
    episode_reduced["map/id"][:n_mp_valid] = episode["map/id"][map_valid_mask]
    # one_hot "map/type": [N_MP, N_MP_TYPE], bool
    episode_reduced["map/type"] = np.eye(n_mp_type, dtype=bool)[episode_reduced["map/type"]]
    episode_reduced["map/type"] = episode_reduced["map/type"] & episode_reduced["map/valid"].any(
        -1, keepdims=True
    )


def filter_episode_traffic_lights(episode: Dict[str, np.ndarray]) -> None:
    """Filter traffic light based on map valid
    Args: episode
        "map/valid": [N_MP_DATA, 20],  # bool
        "map/id": [N_MP_DATA],  # int, with -1
        "tl_lane/id": [N_TL_DATA],  # with -1
        "tl_lane/valid": [N_TL_DATA, n_step],  # bool
    """
    prefix = []
    if "tl_lane/valid" in episode:
        prefix.append("")
    if "history/tl_lane/valid" in episode:
        prefix.append("history/")
    for pf in prefix:
        n_tl_data, n_step = episode[pf + "tl_lane/valid"].shape
        for tl_step in range(n_step):
            for tl_idx in range(n_tl_data):
                if episode[pf + "tl_lane/valid"][tl_idx, tl_step]:
                    tl_lane_map_id = episode["map/id"] == episode[pf + "tl_lane/id"][tl_idx]
                    if episode["map/valid"][tl_lane_map_id].sum() == 0:
                        episode[pf + "tl_lane/valid"][tl_idx, tl_step] = False


def repack_episode_traffic_lights(
    episode: Dict[str, np.ndarray], episode_reduced: Dict[str, np.ndarray], n_tl_lane_h5: int, n_tl_state: int
) -> int:
    """
    Args:
        # episode_reduced: map
        "map/id": [N_MP_H5],  # int, with -1
        "map/dir": [N_MP_H5, 20, 3]
        # episode: traffic light
        "tl_lane/id": [N_TL_DATA],  # with -1
        "tl_lane/valid": [N_TL_DATA, n_step],  # bool
        "tl_lane/state": [N_TL_DATA, n_step],  # >= 0
        "tl_stop/pos": [N_TL_DATA, 3], # x,y,z
    Returns:
        "tl_lane/idx": [n_tl_lane_h5],  # with -1
        "tl_lane/valid": [n_tl_lane_h5, n_step],  # bool
        "tl_lane/state": [n_tl_lane_h5, n_step, n_tl_state], one hot
        "tl_stop/valid": [n_tl_stop_h5=n_tl_data, n_step], bool
        "tl_stop/pos": [n_tl_stop_h5=n_tl_data, 3], x,y,z
        "tl_stop/dir": [n_tl_stop_h5=n_tl_data, 3], x,y,z
        "tl_stop/state": [n_tl_stop_h5=n_tl_data, n_step, n_tl_state], one hot
    """
    prefix = []
    if "tl_lane/valid" in episode:
        prefix.append("")
    if "history/tl_lane/valid" in episode:
        prefix.append("history/")
    n_tl_lane_max = 0
    for pf in prefix:
        n_tl_data, n_step = episode[pf + "tl_lane/valid"].shape
        # tl_lane
        episode_reduced[pf + "tl_lane/idx"] = (
            np.zeros([n_tl_lane_h5], dtype=np.int64) - 1
        )  # int, -1 means not valid
        episode_reduced[pf + "tl_lane/valid"] = np.zeros([n_tl_lane_h5, n_step], dtype=bool)  # bool
        episode_reduced[pf + "tl_lane/state"] = np.zeros(
            [n_tl_lane_h5, n_step], dtype=np.int64
        )  # will be one_hot
        # tl_stop
        episode_reduced[pf + "tl_stop/valid"] = np.zeros([n_tl_data, n_step], dtype=bool)  # bool
        episode_reduced[pf + "tl_stop/state"] = np.zeros(
            [n_tl_data, n_step], dtype=np.int64
        )  # will be one_hot
        episode_reduced[pf + "tl_stop/pos"] = np.zeros([n_tl_data, 3], dtype=np.float32)
        episode_reduced[pf + "tl_stop/dir"] = np.zeros([n_tl_data, 3], dtype=np.float32)

        counter_tl_lane = 0
        counter_tl_stop = 0
        for i in range(n_tl_data):
            # tl_lane
            lane_idx = np.where(episode_reduced["map/id"] == episode[pf + "tl_lane/id"][i])[0]
            if episode[pf + "tl_lane/valid"][i].any():
                n_lanes = lane_idx.shape[0]
                assert counter_tl_lane + n_lanes <= n_tl_lane_h5, print(
                    "counter_tl_lane, n_lanes:", counter_tl_lane, n_lanes
                )
                episode_reduced[pf + "tl_lane/valid"][counter_tl_lane : counter_tl_lane + n_lanes] = episode[
                    pf + "tl_lane/valid"
                ][i]
                episode_reduced[pf + "tl_lane/state"][counter_tl_lane : counter_tl_lane + n_lanes] = episode[
                    pf + "tl_lane/state"
                ][i]
                episode_reduced[pf + "tl_lane/idx"][counter_tl_lane : counter_tl_lane + n_lanes] = lane_idx
                counter_tl_lane += n_lanes

            # tl_stop
            if episode[pf + "tl_lane/valid"][i].any():
                episode_reduced[pf + "tl_stop/valid"][counter_tl_stop] = episode[pf + "tl_lane/valid"][i]
                episode_reduced[pf + "tl_stop/state"][counter_tl_stop] = episode[pf + "tl_lane/state"][i]
                episode_reduced[pf + "tl_stop/pos"][counter_tl_stop] = episode[pf + "tl_stop/pos"][i]
                episode_reduced[pf + "tl_stop/dir"][counter_tl_stop] = episode_reduced["map/dir"][
                    lane_idx[0], 0
                ]
                counter_tl_stop += 1

        # one_hot "tl_lane/state" and "tl_stop/state": [N_AGENT, N_AGENT_TYPE], bool
        episode_reduced[pf + "tl_lane/state"] = np.eye(n_tl_state, dtype=bool)[
            episode_reduced[pf + "tl_lane/state"]
        ]
        episode_reduced[pf + "tl_stop/state"] = np.eye(n_tl_state, dtype=bool)[
            episode_reduced[pf + "tl_stop/state"]
        ]

        episode_reduced[pf + "tl_lane/state"] = (
            episode_reduced[pf + "tl_lane/state"] & episode_reduced[pf + "tl_lane/valid"][:, :, None]
        )
        episode_reduced[pf + "tl_stop/state"] = (
            episode_reduced[pf + "tl_stop/state"] & episode_reduced[pf + "tl_stop/valid"][:, :, None]
        )
        n_tl_lane_max = max(n_tl_lane_max, counter_tl_lane)
    return n_tl_lane_max


def filter_episode_agents(
    episode: Dict[str, np.ndarray],
    episode_reduced: Dict[str, np.ndarray],
    step_current: int,
    n_ag_h5_sim: int,
    dist_thresh_ag: float,
    dim_veh_lanes: List[int],
    prefix: str = "",
) -> Tuple[float, np.ndarray, np.ndarray]:
    """
    Args: episode
        # agent state
        "agent/valid": [N_AG_DATA, n_step], bool,
        "agent/pos": [N_AG_DATA, n_step, 3], float32
        "agent/vel": [N_AG_DATA, n_step, 2], float32, v_x, v_y
        "agent/spd": [N_AG_DATA, n_step, 1], float32
        "agent/yaw_bbox": [N_AG_DATA, n_step, 1], float32, yaw of the bbox heading
        # agent attribute
        "agent/type": [N_AG_DATA, 3], bool, one hot [Vehicle=0, Pedestrian=1, Cyclist=2]
        "agent/cmd": [N_AG_DATA, N_AG_CMD], bool, one hot
        "agent/role": [N_AG_DATA, :], bool, one hot
        "agent/size": [N_AG_DATA, 3], float32: [length, width, height]
        "agent/goal": [N_AG_DATA, 4], float32: [x, y, theta, v]
        # map
        "map/valid": [N_MP_DATA, N_MP_PL_NODE], bool
        "map/id": [N_MP_DATA], int, with -1
        "map/type": [N_MP_DATA], int, >= 0
        "map/pos": [N_MP_DATA, N_MP_PL_NODE, 3]
        "map/dir": [N_MP_DATA, N_MP_PL_NODE, 3]
    """
    n_ag_data = episode[prefix + "agent/valid"].shape[0]
    ag_valid = episode[prefix + "agent/valid"].copy()
    # relevant_agent = episode[prefix + "agent/role"].any(-1)  # [N_AG_DATA]
    # ag_valid_rel = ag_valid[relevant_agent]  # [N, n_step]
    # ag_pos_rel = episode[prefix + "agent/pos"][relevant_agent]  # [N, n_step, 3]
    # ag_pos_rel = ag_pos_rel[ag_valid_rel][:, :2]  # [N, 2]
    # thresh_spd = 2 if prefix == "" else 0.5

    # # ! filter agents not seen in the history.
    # mask_agent_history_not_seen = (~relevant_agent) & ~(ag_valid[:, : step_current + 1].any(axis=1))
    # ag_valid = ag_valid & (~mask_agent_history_not_seen[:, None])

    # # ! filter agents tracked for short period.
    # if prefix == "":
    #     mask_agent_short = (~relevant_agent) & (ag_valid.sum(axis=1) < 20)
    #     ag_valid = ag_valid & (~mask_agent_short[:, None])

    # # ! filter agents that have small displacement and large distance to relevant agents or map polylines
    # mask_agent_still = (
    #     (episode[prefix + "agent/spd"][..., 0].sum(axis=1) * 0.1 < thresh_spd)
    #     & (~relevant_agent)
    #     & (ag_valid.any(axis=1))
    # )
    # lane_pos = episode_reduced["map/pos"][episode_reduced["map/valid"], :2]
    # for i in range(n_ag_data):
    #     if mask_agent_still[i] and (ag_valid.any(axis=1).sum() > n_ag_h5_sim):
    #         agent_poses = episode[prefix + "agent/pos"][i, :, :2][ag_valid[i, :]]
    #         start_pos, end_pos = agent_poses[0], agent_poses[-1]
    #         far_to_relevant_agent = (np.linalg.norm(ag_pos_rel - start_pos, axis=-1).min() > 20) & (
    #             np.linalg.norm(ag_pos_rel - end_pos, axis=-1).min() > 20
    #         )
    #         far_to_lane = (np.linalg.norm(lane_pos - start_pos, axis=-1).min() > 20) & (
    #             np.linalg.norm(lane_pos - end_pos, axis=-1).min() > 20
    #         )
    #         if far_to_relevant_agent & far_to_lane:
    #             ag_valid[i] = False

    # # ! filter parking vehicles that have large distance to relevant agents and lanes.
    # mask_veh_lane = (
    #     episode_reduced["map/type"][:, dim_veh_lanes].any(axis=-1, keepdims=True)
    # ) & episode_reduced["map/valid"]
    # pos_veh_lane = episode_reduced["map/pos"][mask_veh_lane, :2]  # [?, 2]
    # dir_veh_lane = episode_reduced["map/dir"][mask_veh_lane, :2]  # [?, 2]
    # dir_veh_lane = dir_veh_lane / np.linalg.norm(dir_veh_lane, axis=-1, keepdims=True)

    # mask_vehicle_still = (
    #     (episode[prefix + "agent/spd"][..., 0].sum(axis=1) * 0.1 < thresh_spd)
    #     & (~relevant_agent)
    #     & (ag_valid.any(axis=1))
    #     & (episode[prefix + "agent/type"][:, 0])
    # )
    # for i in range(n_ag_data):
    #     if mask_vehicle_still[i] and (ag_valid.any(axis=1).sum() > n_ag_h5_sim):
    #         agent_pos = episode[prefix + "agent/pos"][i, :, :2][ag_valid[i, :]][-1]
    #         agent_yaw = episode[prefix + "agent/yaw_bbox"][i, :, 0][ag_valid[i, :]][-1]
    #         agent_heading = np.array([np.cos(agent_yaw), np.sin(agent_yaw)])

    #         dist_pos = np.linalg.norm((pos_veh_lane - agent_pos), axis=-1)
    #         dist_rot = np.dot(dir_veh_lane, agent_heading)
    #         candidate_lanes = (dist_pos < 3) & (dist_rot > 0)
    #         not_associate_to_lane = ~(candidate_lanes.any())

    #         far_to_relevant_agent = (np.linalg.norm(ag_pos_rel - start_pos, axis=1).min() > 10) & (
    #             np.linalg.norm(ag_pos_rel - end_pos, axis=1).min() > 10
    #         )
    #         if far_to_relevant_agent & not_associate_to_lane:
    #             ag_valid[i] = False

    # # ! filter vehicles that have small displacement and large yaw change. Training only.
    # if prefix == "" and (ag_valid.any(axis=1).sum() > n_ag_h5_sim):
    #     yaw_diff = np.abs(transform_utils.cast_rad(np.diff(episode["agent/yaw_bbox"][..., 0], axis=1))) * (
    #         ag_valid[:, :-1] & ag_valid[:, 1:]
    #     )
    #     max_yaw_diff = yaw_diff.max(axis=1)
    #     mask_large_yaw_diff_veh = (
    #         (episode["agent/spd"][..., 0].sum(axis=1) * 0.1 < 6) & (max_yaw_diff > 0.5)
    #     ) | (max_yaw_diff > 1.5)
    #     mask_large_yaw_diff_veh = mask_large_yaw_diff_veh & (episode["agent/type"][:, 0])
    #     mask_large_yaw_diff_ped_cyc = (
    #         (episode["agent/spd"][..., 0].sum(axis=1) * 0.1 < 1) & (max_yaw_diff > 0.5)
    #     ) | (max_yaw_diff > 1.5)
    #     mask_large_yaw_diff_ped_cyc = mask_large_yaw_diff_ped_cyc & (episode["agent/type"][:, 1:].any(-1))
    #     mask_agent_large_yaw_change = (
    #         (mask_large_yaw_diff_veh | mask_large_yaw_diff_ped_cyc)
    #         & (~relevant_agent)
    #         & (ag_valid.any(axis=1))
    #     )
    #     ag_valid[mask_agent_large_yaw_change] = False

    # # ! still too many agents: filter agents based on distance to relevant agents
    # while ag_valid.any(axis=1).sum() > n_ag_h5_sim:
    #     mask_agent_remain = ~(relevant_agent) & (ag_valid.any(axis=1))
    #     for i in range(n_ag_data):
    #         if mask_agent_remain[i]:
    #             agent_poses = episode[prefix + "agent/pos"][i, :, :2][ag_valid[i, :]]
    #             close_to_relevant_agent = (
    #                 min(
    #                     np.linalg.norm(ag_pos_rel - agent_poses[0], axis=-1).min(),
    #                     np.linalg.norm(ag_pos_rel - agent_poses[-1], axis=-1).min(),
    #                 )
    #                 < dist_thresh_ag
    #             )
    #             if not close_to_relevant_agent:
    #                 ag_valid[i] = False
    #             if ag_valid.any(axis=1).sum() == n_ag_h5_sim:
    #                 break
    #     dist_thresh_ag = dist_thresh_ag * 0.5
    #     # if dist_thresh_ag < 1.0:
    #     #     raise RuntimeError(f"n_agent={agent_valid.any(axis=0).sum()}, n_agent_max={n_agent_max}")

    mask_sim = ag_valid.any(1)
    mask_no_sim = episode[prefix + "agent/valid"].any(1) & (~mask_sim)
    return mask_sim, mask_no_sim


def repack_episode_agents(
    episode: Dict[str, np.ndarray],
    episode_reduced: Dict[str, np.ndarray],
    mask_sim: np.ndarray,
    n_ag_h5_sim: int,
    dim_veh_lanes: List[int] = [],
    dim_cyc_lanes: List[int] = [],
    dim_ped_lanes: List[int] = [],
    dest_no_pred: bool = False,
    prefix: str = "",
) -> None:
    """fill episode_reduced["history/agent/"], episode_reduced["agent/"]
    Args: episode
        # agent state
        "agent/valid": [N_AGENT_DATA, n_step], bool,
        "agent/pos": [N_AGENT_DATA, n_step, 3], float32
        "agent/vel": [N_AGENT_DATA, n_step, 2], float32, v_x, v_y
        "agent/spd": [N_AGENT_DATA, n_step, 1], float32
        "agent/yaw_bbox": [N_AGENT_DATA, n_step, 1], float32, yaw of the bbox heading
        # agent attribute
        "agent/type": [N_AGENT_DATA, 3], bool, one hot [Vehicle=0, Pedestrian=1, Cyclist=2]
        "agent/cmd": [N_AGENT_DATA, N_AG_CMD], bool, one hot
        "agent/role": [N_AGENT_DATA, :], bool, one hot
        "agent/size": [N_AGENT_DATA, 3], float32: [length, width, height]
        "agent/goal": [N_AGENT_DATA, 4], float32: [x, y, theta, v]
        # map
        "map/valid": [N_MP_DATA, n_mp_pl_node], bool
        "map/id": [N_MP_DATA], int, with -1
        "map/type": [N_MP_DATA], int, >= 0
        "map/pos": [N_MP_DATA, n_mp_pl_node, 3]
        "map/dir": [N_MP_DATA, n_mp_pl_node, 3]
    Returns: episode_reduced
    """
    n_step = episode[prefix + "agent/valid"].shape[1]
    # agent state
    episode_reduced[prefix + "agent/valid"] = np.zeros([n_ag_h5_sim, n_step], dtype=bool)  # bool,
    episode_reduced[prefix + "agent/pos"] = np.zeros([n_ag_h5_sim, n_step, 3], dtype=np.float32)  # x,y,z
    episode_reduced[prefix + "agent/vel"] = np.zeros(
        [n_ag_h5_sim, n_step, 2], dtype=np.float32
    )  # v_x, v_y, in m/s
    episode_reduced[prefix + "agent/spd"] = np.zeros(
        [n_ag_h5_sim, n_step, 1], dtype=np.float32
    )  # m/s, signed
    # m/s2, acc[t] = (spd[t]-spd[t-1])/dt
    episode_reduced[prefix + "agent/acc"] = np.zeros([n_ag_h5_sim, n_step, 1], dtype=np.float32)
    episode_reduced[prefix + "agent/yaw_bbox"] = np.zeros(
        [n_ag_h5_sim, n_step, 1], dtype=np.float32
    )  # [-pi, pi]
    # yaw_rate[t] = (yaw[t]-yaw[t-1])/dt
    episode_reduced[prefix + "agent/yaw_rate"] = np.zeros([n_ag_h5_sim, n_step, 1], dtype=np.float32)
    # agent attribute
    episode_reduced[prefix + "agent/object_id"] = np.zeros([n_ag_h5_sim], dtype=np.int64) - 1
    episode_reduced[prefix + "agent/type"] = np.zeros([n_ag_h5_sim, 3], dtype=bool)
    # one hot [sdc=0, interest=1, predict=2]
    episode_reduced[prefix + "agent/role"] = np.zeros(
        [n_ag_h5_sim, episode[prefix + "agent/role"].shape[-1]], dtype=bool
    )
    episode_reduced[prefix + "agent/size"] = np.zeros(
        [n_ag_h5_sim, 3], dtype=np.float32
    )  # [length, width, height]

    if prefix == "":
        episode_reduced["agent/cmd"] = np.zeros([n_ag_h5_sim, N_AG_CMD], dtype=bool)
        episode_reduced["agent/goal"] = np.zeros(
            [n_ag_h5_sim, 4], dtype=np.float32
        )  # float32 [x, y, theta, v]
        episode_reduced["agent/dest"] = np.zeros(
            [n_ag_h5_sim], dtype=np.int64
        )  # index to "map/valid" in [0, N_PL]
        # ! map info for finding dest
        n_mp, n_mp_pl_node = episode_reduced["map/valid"].shape
        mask_veh_lane = (
            episode_reduced["map/type"][:, dim_veh_lanes].any(axis=-1, keepdims=True)
        ) & episode_reduced["map/valid"]
        pos_veh_lane = episode_reduced["map/pos"][mask_veh_lane, :2]  # [?, 2]
        dir_veh_lane = episode_reduced["map/dir"][mask_veh_lane, :2]  # [?, 2]
        dir_veh_lane = dir_veh_lane / np.linalg.norm(dir_veh_lane, axis=-1, keepdims=True)
        map_id_veh_lane = episode_reduced["map/id"][:, None].repeat(n_mp_pl_node, 1)[mask_veh_lane]
        pl_idx_veh_lane = np.arange(n_mp)[:, None].repeat(n_mp_pl_node, 1)[mask_veh_lane]
        # cyc_lane
        mask_cyc_lane = (
            episode_reduced["map/type"][:, dim_cyc_lanes].any(axis=-1, keepdims=True)
        ) & episode_reduced["map/valid"]
        pos_cyc_lane = episode_reduced["map/pos"][mask_cyc_lane, :2]  # [?, 2]
        dir_cyc_lane = episode_reduced["map/dir"][mask_cyc_lane, :2]  # [?, 2]
        dir_cyc_lane = dir_cyc_lane / np.linalg.norm(dir_cyc_lane, axis=-1, keepdims=True)
        pl_idx_cyc_lane = np.arange(n_mp)[:, None].repeat(n_mp_pl_node, 1)[mask_cyc_lane]
        # road_edge
        mask_road_edge = (
            episode_reduced["map/type"][:, dim_ped_lanes].any(axis=-1, keepdims=True)
        ) & episode_reduced["map/valid"]
        pos_road_edge = episode_reduced["map/pos"][mask_road_edge, :2]  # [?, 2]
        pl_idx_road_edge = np.arange(n_mp)[:, None].repeat(n_mp_pl_node, 1)[mask_road_edge]

    for i, idx in enumerate(np.where(mask_sim)[0]):
        valid = episode[prefix + "agent/valid"][idx]
        # if valid.sum() > 1:
        #     valid_steps = np.where(valid)[0]
        #     step_start, step_end = valid_steps[0], valid_steps[-1]

        #     f_pos = interp1d(valid_steps, episode[prefix + "agent/pos"][idx, valid], axis=0)
        #     f_vel = interp1d(valid_steps, episode[prefix + "agent/vel"][idx, valid], axis=0)
        #     f_spd = interp1d(valid_steps, episode[prefix + "agent/spd"][idx, valid], axis=0)
        #     f_yaw_bbox = interp1d(
        #         valid_steps, np.unwrap(episode[prefix + "agent/yaw_bbox"][idx, valid], axis=0), axis=0
        #     )

        #     x = np.arange(step_start, step_end + 1)
        #     x_spd = f_spd(x)
        #     x_yaw = f_yaw_bbox(x)
        #     episode_reduced[prefix + "agent/valid"][i, step_start : step_end + 1] = True
        #     episode_reduced[prefix + "agent/pos"][i, step_start : step_end + 1] = f_pos(x)
        #     episode_reduced[prefix + "agent/vel"][i, step_start : step_end + 1] = f_vel(x)
        #     episode_reduced[prefix + "agent/spd"][i, step_start : step_end + 1] = x_spd
        #     episode_reduced[prefix + "agent/yaw_bbox"][i, step_start : step_end + 1] = x_yaw

        #     x_acc = np.diff(x_spd, axis=0) / 0.1
        #     episode_reduced[prefix + "agent/acc"][i, step_start + 1 : step_end + 1] = x_acc
        #     x_yaw_rate = np.diff(x_yaw, axis=0) / 0.1
        #     episode_reduced[prefix + "agent/yaw_rate"][i, step_start + 1 : step_end + 1] = x_yaw_rate

        # else:
        valid_step = np.where(valid)[0]
        # episode_reduced[prefix + "agent/valid"][i, valid_step] = True

        for k in ["valid", "pos", "vel", "spd", "yaw_bbox"]:
            episode_reduced[prefix + f"agent/{k}"][i, valid_step] = episode[prefix + f"agent/{k}"][
                idx, valid_step
            ]

        for k in ["object_id", "type", "role", "size"]:
            episode_reduced[prefix + f"agent/{k}"][i] = episode[prefix + f"agent/{k}"][idx]

        # if prefix == "":
        #     episode_reduced["agent/goal"][i] = episode["agent/goal"][idx]
        #     episode_reduced["agent/cmd"][i] = episode["agent/cmd"][idx]
        #     episode_reduced["agent/dest"][i] = _find_dest(
        #         episode_reduced["agent/type"][i],
        #         episode_reduced["agent/goal"][i],
        #         episode["map/edge"],
        #         pos_veh_lane,
        #         dir_veh_lane,
        #         map_id_veh_lane,
        #         pl_idx_veh_lane,
        #         pos_cyc_lane,
        #         dir_cyc_lane,
        #         pl_idx_cyc_lane,
        #         pos_road_edge,
        #         pl_idx_road_edge,
        #         dest_no_pred,
        #     )


def repack_episode_agents_no_sim(
    episode: Dict[str, np.ndarray],
    episode_reduced: Dict[str, np.ndarray],
    mask_no_sim: np.ndarray,
    n_ag_h5_no_sim: int,
    prefix: str,
) -> None:
    n_step = episode[prefix + "agent/valid"].shape[1]
    episode_reduced[prefix + "agent_no_sim/valid"] = np.zeros([n_ag_h5_no_sim, n_step], dtype=bool)
    episode_reduced[prefix + "agent_no_sim/pos"] = np.zeros([n_ag_h5_no_sim, n_step, 3], dtype=np.float32)
    episode_reduced[prefix + "agent_no_sim/vel"] = np.zeros([n_ag_h5_no_sim, n_step, 2], dtype=np.float32)
    episode_reduced[prefix + "agent_no_sim/spd"] = np.zeros([n_ag_h5_no_sim, n_step, 1], dtype=np.float32)
    episode_reduced[prefix + "agent_no_sim/yaw_bbox"] = np.zeros(
        [n_ag_h5_no_sim, n_step, 1], dtype=np.float32
    )
    episode_reduced[prefix + "agent_no_sim/object_id"] = np.zeros([n_ag_h5_no_sim], dtype=np.int64) - 1
    episode_reduced[prefix + "agent_no_sim/type"] = np.zeros([n_ag_h5_no_sim, 3], dtype=bool)
    episode_reduced[prefix + "agent_no_sim/size"] = np.zeros([n_ag_h5_no_sim, 3], dtype=np.float32)
    # no role, no cmd, no goal
    for i, idx in enumerate(np.where(mask_no_sim)[0]):
        for k in ["valid", "pos", "vel", "spd", "yaw_bbox", "object_id", "type", "size"]:
            episode_reduced[prefix + f"agent_no_sim/{k}"][i] = episode[prefix + f"agent/{k}"][idx]


def get_polylines_from_polygon(polygon: np.ndarray) -> List[List[List]]:
    # polygon: [4, 3]
    l1 = np.linalg.norm(polygon[1, :2] - polygon[0, :2])
    l2 = np.linalg.norm(polygon[2, :2] - polygon[1, :2])

    def _pl_interp_start_end(start: np.ndarray, end: np.ndarray) -> List[List]:
        length = np.linalg.norm(start - end)
        unit_vec = (end - start) / length
        pl = []
        for i in range(int(length) + 1):  # 4.5 -> 5 [0,1,2,3,4]
            x, y, z = start + unit_vec * i
            pl.append([x, y, z])
        pl.append([end[0], end[1], end[2]])
        return pl

    # if l1 > l2:
    #     pl1 = _pl_interp_start_end((polygon[0] + polygon[3]) * 0.5, (polygon[1] + polygon[2]) * 0.5)
    # else:
    #     pl1 = _pl_interp_start_end((polygon[0] + polygon[1]) * 0.5, (polygon[2] + polygon[3]) * 0.5)
    # return [pl1, pl1[::-1]]

    if l1 > l2:
        pl1 = _pl_interp_start_end(polygon[0], polygon[1])
        pl2 = _pl_interp_start_end(polygon[2], polygon[3])
    else:
        pl1 = _pl_interp_start_end(polygon[0], polygon[3])
        pl2 = _pl_interp_start_end(polygon[2], polygon[1])
    return [pl1, pl1[::-1], pl2, pl2[::-1]]


def get_map_boundary(map_valid, map_pos):
    """
    Args:
        map_valid: [n_pl, 20],  # bool
        map_pos: [n_pl, 20, 3],  # float32
    Returns:
        map_boundary: [4]
    """
    pos = map_pos[map_valid]
    xmin = pos[:, 0].min()
    ymin = pos[:, 1].min()
    xmax = pos[:, 0].max()
    ymax = pos[:, 1].max()
    return np.array([xmin, xmax, ymin, ymax])


def _find_dest(
    agent_type,  # one_hot [3]
    agent_goal,  # [x, y, theta, v]
    map_edge,  # [?, 2] id0 -> id1, or id0 -> -1
    pos_veh_lane,  # [?, 2]
    dir_veh_lane,  # [?, 2]
    map_id_veh_lane,  # [?]
    pl_idx_veh_lane,  # [?]
    pos_cyc_lane,  # [?, 2]
    dir_cyc_lane,  # [?, 2]
    pl_idx_cyc_lane,  # [?, 2]
    pos_road_edge,  # [?, 2]
    pl_idx_road_edge,  # [?]
    no_pred,
):
    goal_yaw = agent_goal[2]
    goal_heading = np.array([np.cos(goal_yaw), np.sin(goal_yaw)])
    goal_pos = agent_goal[:2]
    if no_pred:
        extended_goal_pos = goal_pos
    else:
        extended_goal_pos = agent_goal[:2] + goal_heading * agent_goal[3] * 5  # 5 seconds with constant speed
    if agent_type[0]:  # veh
        dist_pos = np.linalg.norm((pos_veh_lane - goal_pos), axis=1)
        dist_rot = np.dot(dir_veh_lane, goal_heading)
        candidate_lanes = (dist_pos < 3) & (dist_rot > 0)
        if candidate_lanes.any():  # associate to a lane, extend with map topology
            if no_pred:
                idx_dest = pl_idx_veh_lane[candidate_lanes][np.argmin(dist_pos[candidate_lanes])]
            else:
                dest_map_id = map_id_veh_lane[candidate_lanes][np.argmin(dist_pos[candidate_lanes])]
                next_map_id = dest_map_id
                counter = 0
                continue_extend = True
                while continue_extend:
                    next_edges = np.where(map_edge[:, 0] == next_map_id)[0]
                    dest_map_id, next_map_id = map_edge[np.random.choice(next_edges)]
                    counter += 1
                    # if (
                    #     (next_map_id not in map_id_veh_lane)
                    #     or ((len(next_edges) > 1) and (counter > 3))
                    #     or (counter > 6)
                    # ):
                    if (
                        (next_map_id not in map_id_veh_lane)
                        or ((len(next_edges) > 1) and (counter > 1))
                        or (counter > 3)
                    ):
                        continue_extend = False
                idx_dest = pl_idx_veh_lane[np.where(map_id_veh_lane == dest_map_id)[0][-1]]
        else:  # not associate to a lane, use road edge
            idx_dest = pl_idx_road_edge[np.linalg.norm((pos_road_edge - extended_goal_pos), axis=1).argmin()]
    elif agent_type[1]:  # ped
        idx_dest = pl_idx_road_edge[np.linalg.norm((pos_road_edge - extended_goal_pos), axis=1).argmin()]
    elif agent_type[2]:  # cyc
        dist_pos = np.linalg.norm((pos_cyc_lane - extended_goal_pos), axis=1)
        dist_rot = np.dot(dir_cyc_lane, goal_heading)
        candidate_lanes = (dist_pos < 3) & (dist_rot > 0)
        if candidate_lanes.any():  # associate to a bike lane, extend with constant vel and find bike lane
            idx_dest = pl_idx_cyc_lane[candidate_lanes][np.argmin(dist_pos[candidate_lanes])]
        else:  # not associate to a lane, use road edge
            idx_dest = pl_idx_road_edge[np.linalg.norm((pos_road_edge - extended_goal_pos), axis=1).argmin()]
    return idx_dest


def _classify_track(
    valid: np.ndarray,
    pos: np.ndarray,
    yaw: np.ndarray,
    spd: np.ndarray,
    kMaxSpeedForStationary: float = 2.0,  # (m/s)
    kMaxDisplacementForStationary: float = 5.0,  # (m)
    kMaxLateralDisplacementForStraight: float = 5.0,  # (m)
    kMinLongitudinalDisplacementForUTurn: float = -5.0,  # (m)
    kMaxAbsHeadingDiffForStraight: float = 0.5236,  # M_PI / 6.0
) -> int:
    """github.com/waymo-research/waymo-open-dataset/blob/master/waymo_open_dataset/metrics/motion_metrics_utils.cc
    Args:
        valid: [n_step], bool
        pos: [n_step, 2], x,y
        yaw: [n_step], float32
        spd: [n_step], float32
    Returns:
        traj_type: int in range(N_AG_CMD)
            # STATIONARY = 0;
            # STRAIGHT = 1;
            # STRAIGHT_LEFT = 2;
            # STRAIGHT_RIGHT = 3;
            # LEFT_U_TURN = 4;
            # LEFT_TURN = 5;
            # RIGHT_U_TURN = 6;
            # RIGHT_TURN = 7;
    """
    i0 = valid.argmax()
    i1 = len(valid) - 1 - np.flip(valid).argmax()

    x, y = pos[i1] - pos[i0]
    final_displacement = np.sqrt(x**2 + y**2)

    _c = np.cos(-yaw[i0])
    _s = np.sin(-yaw[i0])
    dx = x * _c - y * _s
    dy = x * _s + y * _c

    heading_diff = yaw[i1] - yaw[i0]
    max_speed = max(spd[i0], spd[i1])

    if max_speed < kMaxSpeedForStationary and final_displacement < kMaxDisplacementForStationary:
        return 0  # TrajectoryType::STATIONARY;

    if np.abs(heading_diff) < kMaxAbsHeadingDiffForStraight:
        if np.abs(dy) < kMaxLateralDisplacementForStraight:
            return 1  # TrajectoryType::STRAIGHT;
        if dy > 0:
            return 2  # TrajectoryType::STRAIGHT_LEFT
        else:
            return 3  # TrajectoryType::STRAIGHT_RIGHT

    if heading_diff < -kMaxAbsHeadingDiffForStraight and dy < 0:
        if dx < kMinLongitudinalDisplacementForUTurn:
            return 6  # TrajectoryType::RIGHT_U_TURN
        else:
            return 7  # TrajectoryType::RIGHT_TURN

    if dx < kMinLongitudinalDisplacementForUTurn:
        return 4  # TrajectoryType::LEFT_U_TURN;

    return 5  # TrajectoryType::LEFT_TURN;
