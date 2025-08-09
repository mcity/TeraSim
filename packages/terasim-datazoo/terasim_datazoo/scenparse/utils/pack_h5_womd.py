import sys

from argparse import ArgumentParser
from tqdm import tqdm
import h5py
import numpy as np
from pathlib import Path
from waymo_open_dataset.protos import scenario_pb2
from . import pack_h5 as pack_utils
import tensorflow as tf

# Licensed under the CC BY-NC 4.0 license (https://creativecommons.org/licenses/by-nc/4.0/)
tf.config.set_visible_devices([], "GPU")

# "map/type"
# FREEWAY = 0
# SURFACE_STREET = 1
# STOP_SIGN = 2
# BIKE_LANE = 3
# TYPE_ROAD_EDGE_BOUNDARY = 4
# TYPE_ROAD_EDGE_MEDIAN = 5
# BROKEN = 6
# SOLID_SINGLE = 7
# DOUBLE = 8
# SPEED_BUMP = 9
# CROSSWALK = 10
N_MP_TYPE = 11
N_MP_PL_NODE = 20
DIM_VEH_LANES, DIM_CYC_LANES, DIM_PED_LANES = [0, 1, 2], [3], [4]

# "tl_lane/state", "tl_stop/state"
N_TL_STATE = 9

N_AG_TYPE = 3  # [TYPE_VEHICLE=0, TYPE_PEDESTRIAN=1, TYPE_CYCLIST=2]

N_MP_DATA, N_TL_DATA, N_AG_DATA = 3000, 50, 1024
N_MP_H5, N_TL_LANE_H5, N_AG_H5_SIM, N_AG_H5_NO_SIM = 1024, 128, 1024, 1024

DIST_THRESH_MP = 500  # ! 120
DIST_THRESH_AG = 120

N_STEP, STEP_CURRENT = 91, 10


def collate_map_features(map_features):
    mp_id, mp_xyz, mp_type, mp_edge = [], [], [], []
    for mf in map_features:
        feature_data_type = mf.WhichOneof("feature_data")
        if (
            feature_data_type is None
        ):  # pip install waymo-open-dataset-tf-2-6-0==1.4.9, not updated, should be driveway
            continue
        feature = getattr(mf, feature_data_type)
        if feature_data_type == "lane":
            if feature.type == 0:  # UNDEFINED
                mp_type.append(1)
            elif feature.type == 1:  # FREEWAY
                mp_type.append(0)
            elif feature.type == 2:  # SURFACE_STREET
                mp_type.append(1)
            elif feature.type == 3:  # BIKE_LANE
                mp_type.append(3)
            mp_id.append(mf.id)
            mp_xyz.append([[p.x, p.y, p.z] for p in feature.polyline][::2])
            if len(feature.exit_lanes) > 0:
                for _id_exit in feature.exit_lanes:
                    mp_edge.append([mf.id, _id_exit])
            else:
                mp_edge.append([mf.id, -1])
        elif feature_data_type == "stop_sign":
            for l_id in feature.lane:
                # override FREEWAY/SURFACE_STREET with stop sign lane
                # BIKE_LANE remains unchanged
                idx_lane = mp_id.index(l_id)
                if mp_type[idx_lane] < 2:
                    mp_type[idx_lane] = 2
        elif feature_data_type == "road_edge":
            assert feature.type > 0  # no UNKNOWN = 0
            mp_id.append(mf.id)
            mp_type.append(feature.type + 3)  # [1, 2] -> [4, 5]
            mp_xyz.append([[p.x, p.y, p.z] for p in feature.polyline][::2])
        elif feature_data_type == "road_line":
            assert feature.type > 0  # no UNKNOWN = 0
            # BROKEN_SINGLE_WHITE = 1
            # SOLID_SINGLE_WHITE = 2
            # SOLID_DOUBLE_WHITE = 3
            # BROKEN_SINGLE_YELLOW = 4
            # BROKEN_DOUBLE_YELLOW = 5
            # SOLID_SINGLE_YELLOW = 6
            # SOLID_DOUBLE_YELLOW = 7
            # PASSING_DOUBLE_YELLOW = 8
            if feature.type in [1, 4, 5]:
                feature_type_new = 6  # BROKEN
            elif feature.type in [2, 6]:
                feature_type_new = 7  # SOLID_SINGLE
            else:
                feature_type_new = 8  # DOUBLE
            mp_id.append(mf.id)
            mp_type.append(feature_type_new)
            mp_xyz.append([[p.x, p.y, p.z] for p in feature.polyline][::2])
        elif feature_data_type in ["speed_bump", "driveway", "crosswalk"]:
            xyz = np.array([[p.x, p.y, p.z] for p in feature.polygon])
            polygon_idx = np.linspace(0, xyz.shape[0], 4, endpoint=False, dtype=int)
            pl_polygon = pack_utils.get_polylines_from_polygon(xyz[polygon_idx])
            mp_xyz.extend(pl_polygon)
            mp_id.extend([mf.id] * len(pl_polygon))
            pl_type = 9 if feature_data_type in ["speed_bump", "driveway"] else 10
            mp_type.extend([pl_type] * len(pl_polygon))
        else:
            raise ValueError

    return mp_id, mp_xyz, mp_type, mp_edge


def collate_traffic_light_features(tl_features):
    tl_lane_state, tl_lane_id, tl_stop_point = [], [], []
    for _step_tl in tl_features:
        step_tl_lane_state, step_tl_lane_id, step_tl_stop_point = [], [], []
        for _tl in _step_tl.lane_states:
            step_tl_lane_state.append(_tl.state)
            step_tl_lane_id.append(_tl.lane)
            step_tl_stop_point.append([_tl.stop_point.x, _tl.stop_point.y, _tl.stop_point.z])

        tl_lane_state.append(step_tl_lane_state)
        tl_lane_id.append(step_tl_lane_id)
        tl_stop_point.append(step_tl_stop_point)
    return tl_lane_state, tl_lane_id, tl_stop_point


def collate_agent_features(tracks, sdc_track_index, track_index_predict, object_id_interest):
    ag_id, ag_type, ag_state, ag_role = [], [], [], []
    for i, _track in enumerate(tracks):
        ag_id.append(_track.id)
        ag_type.append(
            _track.object_type - 1
        )  # [TYPE_VEHICLE=1, TYPE_PEDESTRIAN=2, TYPE_CYCLIST=3] -> [0,1,2]
        step_state = []
        for s in _track.states:
            step_state.append(
                [
                    s.center_x,
                    s.center_y,
                    s.center_z,
                    s.length,
                    s.width,
                    s.height,
                    s.heading,
                    s.velocity_x,
                    s.velocity_y,
                    s.valid,
                ]
            )
            # This angle is normalized to [-pi, pi). The velocity vector in m/s
        ag_state.append(step_state)

        ag_role.append([False, False, False])
        if i in track_index_predict:
            ag_role[-1][2] = True
        if _track.id in object_id_interest:
            ag_role[-1][1] = True
        if i == sdc_track_index:
            ag_role[-1][0] = True

    return ag_id, ag_type, ag_state, ag_role


def pack_scenario_raw(scenario) -> dict[str, np.ndarray]:

    N_MP_PL_NODE = 2000

    mp_id, mp_xyz, mp_type, mp_edge = collate_map_features(scenario.map_features)
    tl_lane_state, tl_lane_id, tl_stop_point = collate_traffic_light_features(
        scenario.dynamic_map_states
    )
    ag_id, ag_type, ag_state, ag_role = collate_agent_features(
        scenario.tracks,
        sdc_track_index=scenario.sdc_track_index,
        track_index_predict=[i.track_index for i in scenario.tracks_to_predict],
        object_id_interest=[i for i in scenario.objects_of_interest],
    )

    episode: dict[str,np.ndarray] = {}
    pack_utils.pack_episode_map(
        episode, mp_id, mp_xyz, mp_type, mp_edge, N_MP_DATA, N_MP_PL_NODE
    )
    pack_utils.pack_episode_traffic_lights(
        episode,
        STEP_CURRENT,
        tl_lane_state,
        tl_lane_id,
        tl_stop_point,
        True,
        False,
        N_TL_DATA,
    )
    pack_utils.pack_episode_agents(
        episode,
        STEP_CURRENT,
        ag_id,
        ag_type,
        ag_state,
        ag_role,
        True,
        False,
        N_AG_DATA,
        N_AG_TYPE,
    )

    episode_reduced: dict[str,np.ndarray] = {}
    pack_utils.filter_episode_map(episode, STEP_CURRENT, N_MP_H5, DIST_THRESH_MP, thresh_z=6)
    pack_utils.repack_episode_map(episode, episode_reduced, N_MP_H5, N_MP_TYPE)

    pack_utils.filter_episode_traffic_lights(episode)
    pack_utils.repack_episode_traffic_lights(
        episode, episode_reduced, N_TL_LANE_H5, N_TL_STATE
    )

    mask_sim, mask_no_sim = pack_utils.filter_episode_agents(
        episode, episode_reduced, STEP_CURRENT, N_AG_H5_SIM, DIST_THRESH_AG, DIM_VEH_LANES
    )
    pack_utils.repack_episode_agents(
        episode,
        episode_reduced,
        mask_sim,
        N_AG_H5_SIM,
        DIM_VEH_LANES,
        DIM_CYC_LANES,
        DIM_PED_LANES,
        dest_no_pred= True,
    )

    episode_with_map = episode["map/valid"].any(1).sum() > 0
    if episode_with_map:
        episode_reduced["map/boundary"] = pack_utils.get_map_boundary(
            episode_reduced["map/valid"], episode_reduced["map/pos"]
        )
    else:
        episode_reduced["map/boundary"] = np.array([0,1,0,1])
    
    return episode_reduced


def main():
    # !!! parse argunments for the main function
    parser = ArgumentParser(allow_abbrev=True)
    parser.add_argument(
        "--data-dir", default="/scratch/mcity_project_root/mcity_project/zhuhj/womd_scenario_v_1_2_0"
    )
    parser.add_argument("--dataset", default="training")
    parser.add_argument(
        "--out-dir",
        default="/scratch/mcity_project_root/mcity_project/zhuhj/h5_womd_traffic_bots_v_1_5_debug",
    )
    parser.add_argument("--rand-pos", default=50.0, type=float, help="Meter. Set to -1 to disable.")
    parser.add_argument("--rand-yaw", default=3.14, type=float, help="Radian. Set to -1 to disable.")
    parser.add_argument("--dest-no-pred", action="store_true")
    args = parser.parse_args()

    # !!! some configurations
    dataset_size = {
        "training": 486995,  # 487002
        "validation": 44097,
        "training_20s": 70541,
        "validation_interactive": 43479,
        "testing": 44920,
        "testing_interactive": 44154,
    }

    if "training" in args.dataset:
        pack_all = True  # ["agent/valid"]
        pack_history = False  # ["history/agent/valid"]
    elif "validation" in args.dataset:
        pack_all = True
        pack_history = True
    elif "testing" in args.dataset:
        pack_all = False
        pack_history = True

    out_path = Path(args.out_dir)
    out_path.mkdir(exist_ok=True)
    out_h5_path = out_path / (args.dataset + ".h5")

    data_path = Path(args.data_dir) / args.dataset
    dataset = tf.data.TFRecordDataset(
        sorted([p.as_posix() for p in data_path.glob("*")]), compression_type=""
    )
    n_mp_max, n_tl_lane_max, n_tl_stop_max, n_ag_max, n_ag_sim, n_ag_no_sim, data_len = 0, 0, 0, 0, 0, 0, 0

    # !!! the main loop
    with h5py.File(out_h5_path, "w") as hf:
        for i, data in tqdm(enumerate(dataset), total=dataset_size[args.dataset]):
            if i > 100:
                break
            scenario = scenario_pb2.Scenario()
            scenario.ParseFromString(data.numpy())

            # !! collation step
            mp_id, mp_xyz, mp_type, mp_edge = collate_map_features(scenario.map_features)
            tl_lane_state, tl_lane_id, tl_stop_point = collate_traffic_light_features(
                scenario.dynamic_map_states
            )
            ag_id, ag_type, ag_state, ag_role = collate_agent_features(
                scenario.tracks,
                sdc_track_index=scenario.sdc_track_index,
                track_index_predict=[i.track_index for i in scenario.tracks_to_predict],
                object_id_interest=[i for i in scenario.objects_of_interest],
            )

            # !! pack step
            episode = {}
            n_mp = pack_utils.pack_episode_map(
                episode, mp_id, mp_xyz, mp_type, mp_edge, N_MP_DATA, N_MP_PL_NODE
            )
            n_tl_stop = pack_utils.pack_episode_traffic_lights(
                episode,
                STEP_CURRENT,
                tl_lane_state,
                tl_lane_id,
                tl_stop_point,
                pack_all,
                pack_history,
                N_TL_DATA,
            )
            n_ag = pack_utils.pack_episode_agents(
                episode,
                STEP_CURRENT,
                ag_id,
                ag_type,
                ag_state,
                ag_role,
                pack_all,
                pack_history,
                N_AG_DATA,
                N_AG_TYPE,
            )

            # !! center at self driving car
            scenario_center, scenario_yaw = pack_utils.center_at_sdc(
                episode, STEP_CURRENT, args.rand_pos, args.rand_yaw
            )
            n_mp_max, n_tl_stop_max, n_ag_max = (
                max(n_mp_max, n_mp),
                max(n_tl_stop_max, n_tl_stop),
                max(n_ag_max, n_ag),
            )

            # !! filter and repack (based on sdc positon)
            episode_reduced = {}
            pack_utils.filter_episode_map(episode, STEP_CURRENT, N_MP_H5, DIST_THRESH_MP, thresh_z=6)
            episode_with_map = episode["map/valid"].any(1).sum() > 0
            pack_utils.repack_episode_map(episode, episode_reduced, N_MP_H5, N_MP_TYPE)

            pack_utils.filter_episode_traffic_lights(episode)
            n_tl_lane = pack_utils.repack_episode_traffic_lights(
                episode, episode_reduced, N_TL_LANE_H5, N_TL_STATE
            )
            n_tl_lane_max = max(n_tl_lane_max, n_tl_lane)

            if "training" in args.dataset:
                mask_sim, mask_no_sim = pack_utils.filter_episode_agents(
                    episode, episode_reduced, STEP_CURRENT, N_AG_H5_SIM, DIST_THRESH_AG, DIM_VEH_LANES
                )
                pack_utils.repack_episode_agents(
                    episode,
                    episode_reduced,
                    mask_sim,
                    N_AG_H5_SIM,
                    DIM_VEH_LANES,
                    DIM_CYC_LANES,
                    DIM_PED_LANES,
                    dest_no_pred=args.dest_no_pred,
                )
            elif "validation" in args.dataset:
                mask_sim, mask_no_sim = pack_utils.filter_episode_agents(
                    episode,
                    episode_reduced,
                    STEP_CURRENT,
                    N_AG_H5_SIM,
                    DIST_THRESH_AG,
                    DIM_VEH_LANES,
                    prefix="history/",
                )
                pack_utils.repack_episode_agents(
                    episode,
                    episode_reduced,
                    mask_sim,
                    N_AG_H5_SIM,
                    DIM_VEH_LANES,
                    DIM_CYC_LANES,
                    DIM_PED_LANES,
                    dest_no_pred=args.dest_no_pred,
                )
                pack_utils.repack_episode_agents(
                    episode, episode_reduced, mask_sim, N_AG_H5_SIM, prefix="history/"
                )
                pack_utils.repack_episode_agents_no_sim(
                    episode, episode_reduced, mask_no_sim, N_AG_H5_NO_SIM, ""
                )
                pack_utils.repack_episode_agents_no_sim(
                    episode, episode_reduced, mask_no_sim, N_AG_H5_NO_SIM, "history/"
                )

            elif "testing" in args.dataset:
                if episode_with_map:
                    mask_sim, mask_no_sim = pack_utils.filter_episode_agents(
                        episode,
                        episode_reduced,
                        STEP_CURRENT,
                        N_AG_H5_SIM,
                        DIST_THRESH_AG,
                        DIM_VEH_LANES,
                        prefix="history/",
                    )
                else:
                    mask_valid = episode["history/agent/valid"].any(1)
                    mask_sim = episode["history/agent/role"].any(-1)
                    for _valid_idx in np.where(mask_valid)[0]:
                        mask_sim[_valid_idx] = True
                        if mask_sim.sum() >= N_AG_H5_SIM:
                            break
                    mask_no_sim = mask_valid & (~mask_sim)

                pack_utils.repack_episode_agents(
                    episode, episode_reduced, mask_sim, N_AG_H5_SIM, prefix="history/"
                )
                pack_utils.repack_episode_agents_no_sim(
                    episode, episode_reduced, mask_no_sim, N_AG_H5_NO_SIM, prefix="history/"
                )
            n_ag_sim = max(n_ag_sim, mask_sim.sum())
            n_ag_no_sim = max(n_ag_no_sim, mask_no_sim.sum())

            if episode_with_map:
                episode_reduced["map/boundary"] = pack_utils.get_map_boundary(
                    episode_reduced["map/valid"], episode_reduced["map/pos"]
                )
            else:
                # only in waymo test split.
                assert args.dataset == "testing"
                episode_reduced["map/boundary"] = pack_utils.get_map_boundary(
                    episode["history/agent/valid"], episode["history/agent/pos"]
                )
                print(f"scenario {i} has no map! map boundary is: {episode_reduced['map/boundary']}")

            hf_episode = hf.create_group(str(i))
            hf_episode.attrs["scenario_id"] = scenario.scenario_id
            hf_episode.attrs["scenario_center"] = scenario_center
            hf_episode.attrs["scenario_yaw"] = scenario_yaw
            hf_episode.attrs["with_map"] = episode_with_map

            for k, v in episode_reduced.items():
                hf_episode.create_dataset(k, data=v, compression="gzip", compression_opts=4, shuffle=True)

            data_len += 1

        print(f"data_len: {data_len}, dataset_size: {dataset_size[args.dataset]}")
        print(f"n_mp_max: {n_mp_max}")
        print(f"n_tl_lane_max: {n_tl_lane_max}, n_tl_stop_max: {n_tl_stop_max}")
        print(f"n_ag_max: {n_ag_max}, n_ag_sim: {n_ag_sim}, n_ag_no_sim: {n_ag_no_sim}")
        hf.attrs["data_len"] = data_len


if __name__ == "__main__":
    main()
