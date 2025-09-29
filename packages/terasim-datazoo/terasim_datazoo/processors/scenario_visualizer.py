import numpy as np
import cv2
import os
from pathlib import Path
from typing import Union
from terasim_datazoo.utils.pack_h5_womd import *

COLOR_BLACK = (0, 0, 0)
COLOR_WHITE = (255, 255, 255)

COLOR_RED = (255, 0, 0)
COLOR_GREEN = (0, 255, 0)
COLOR_BLUE = (0, 0, 255)
COLOR_CYAN = (0, 255, 255)
COLOR_MAGENTA = (255, 0, 255)
COLOR_MAGENTA_2 = (255, 140, 255)
COLOR_YELLOW = (255, 255, 0)
COLOR_YELLOW_2 = (160, 160, 0)
COLOR_VIOLET = (170, 0, 255)

COLOR_BUTTER_0 = (252, 233, 79)
COLOR_BUTTER_1 = (237, 212, 0)
COLOR_BUTTER_2 = (196, 160, 0)
COLOR_ORANGE_0 = (252, 175, 62)
COLOR_ORANGE_1 = (209, 92, 0)
COLOR_ORANGE_2 = (119, 78, 56)
COLOR_CHOCOLATE_0 = (233, 185, 110)
COLOR_CHOCOLATE_1 = (193, 125, 17)
COLOR_CHOCOLATE_2 = (143, 89, 2)
COLOR_CHAMELEON_0 = (138, 226, 52)
COLOR_CHAMELEON_1 = (115, 210, 22)
COLOR_CHAMELEON_2 = (78, 154, 6)
COLOR_SKY_BLUE_0 = (135, 206, 250)
COLOR_SKY_BLUE_1 = (52, 101, 164)
COLOR_SKY_BLUE_2 = (32, 74, 135)
COLOR_PLUM_0 = (200, 160, 195)
COLOR_PLUM_1 = (117, 80, 123)
COLOR_PLUM_2 = (92, 53, 102)
COLOR_SCARLET_RED_0 = (239, 41, 41)
COLOR_SCARLET_RED_1 = (204, 0, 0)
COLOR_SCARLET_RED_2 = (164, 0, 0)
COLOR_ALUMINIUM_0 = (238, 238, 236)
COLOR_ALUMINIUM_1 = (211, 215, 207)
COLOR_ALUMINIUM_2 = (186, 189, 182)
COLOR_ALUMINIUM_3 = (136, 138, 133)
COLOR_ALUMINIUM_4 = (85, 87, 83)
COLOR_ALUMINIUM_4_5 = (66, 62, 64)
COLOR_ALUMINIUM_5 = (46, 52, 54)


class ScenarioVisualizer:
    def __init__(
        self,
        scenario,
        px_per_m: float = 10.0,
        video_size: int = 960,
        types_to_draw: Union[str,list[int]] = "lanecenters",
        attention_weights_to_pl = None,
        shaded: bool = False,
        simplify_tls_color: bool = True,
        simplify_agent_color: bool = True,
        tls_style: str = "tlstop",
    ) -> None:

        # centered around ego vehicle first step, x=0, y=0, theta=0
        self.px_per_m = px_per_m
        self.video_size = video_size

        if type(types_to_draw) == list:
            self.types_to_draw = types_to_draw
        elif types_to_draw == "lanecenters":
            self.types_to_draw = [0,1,2]
        elif types_to_draw == "withedge":
            self.types_to_draw = [0,1,2,4,5]
        elif types_to_draw == "all":
            self.types_to_draw = list(range(11))
        self.shaded: bool = shaded
        self.simplifed_tls_color: bool = simplify_tls_color
        self.simplified_agent_color: bool = simplify_agent_color
        self.tls_style: str = tls_style

        # ----------- style -----------
        self.lane_style = [
            (COLOR_ALUMINIUM_4, 8),  # FREEWAY = 0
            (COLOR_ALUMINIUM_4, 8),  # SURFACE_STREET = 1
            (COLOR_ORANGE_2, 6),  # STOP_SIGN = 2
            (COLOR_CHOCOLATE_2, 6),  # BIKE_LANE = 3
            (COLOR_SKY_BLUE_0, 4),  # TYPE_ROAD_EDGE_BOUNDARY = 4
            (COLOR_PLUM_0, 4),  # TYPE_ROAD_EDGE_MEDIAN = 5
            (COLOR_BUTTER_0, 2),  # BROKEN = 6
            (COLOR_MAGENTA, 2),  # SOLID_SINGLE = 7
            (COLOR_SCARLET_RED_2, 2),  # DOUBLE = 8
            (COLOR_CHAMELEON_2, 4),  # SPEED_BUMP = 9
            (COLOR_SKY_BLUE_0, 4),  # CROSSWALK = 10
        ]
        self.agent_role_style = [
            COLOR_CYAN,  # sdc = 0
            COLOR_CHAMELEON_2,  # interest = 1
            COLOR_MAGENTA,  # predict = 2
        ]
        self.tl_style = [
            COLOR_ALUMINIUM_1,  # unknown
            COLOR_MAGENTA,  # arrow red
            COLOR_ORANGE_0,  # arrow yellow
            COLOR_CYAN,  # arrow green
            COLOR_RED,  # red
            COLOR_YELLOW,  # yellow
            COLOR_GREEN,  # green
            COLOR_CHOCOLATE_0,  # flash red
            COLOR_CHOCOLATE_0,  # flash yellow
        ]

        self.scenario_id = scenario.scenario_id
        self.episode: dict[str, np.ndarray] = pack_scenario_raw(scenario)
        raster_map, self.top_left_px = self._register_map(self.episode["map/boundary"], self.px_per_m)
        self.raster_map = self._draw_map(
            raster_map,
            self.episode["map/valid"],
            self.episode["map/type"],
            self.episode["map/pos"][..., :2],
            attn_weights_to_pl=attention_weights_to_pl
        )

    @staticmethod
    def _register_map(
        map_boundary: np.ndarray, px_per_m: float, edge_px: int = 20
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Args:
            map_boundary: [4], xmin, xmax, ymin, ymax
            px_per_m: float

        Returns:
            raster_map: empty image
            top_left_px
        """
        # y axis is inverted in pixel coordinate
        xmin, xmax, ymax, ymin = (map_boundary * px_per_m).astype(np.int64)
        ymax *= -1
        ymin *= -1
        xmin -= edge_px
        ymin -= edge_px
        xmax += edge_px
        ymax += edge_px

        raster_map = np.ones([ymax - ymin, xmax - xmin, 3], dtype=np.uint8) * 255
        top_left_px = np.array([xmin, ymin], dtype=np.float32)
        return raster_map, top_left_px

    def _draw_map(
        self,
        raster_map: np.ndarray,
        map_valid: np.ndarray,
        map_type: np.ndarray,
        map_pos: np.ndarray,
        attn_weights_to_pl: Union[np.ndarray, None] = None,
    ) -> np.ndarray:
        """
        Args: numpy arrays
            map_valid: [n_pl, 20],  # bool
            map_type: [n_pl, 11],  # bool one_hot
            map_pos: [n_pl, 20, 2],  # float32
            attn_weights_to_pl: [n_pl], sum up to 1

        Returns:
            raster_map
        """
        mask_valid = map_valid.any(axis=1)
        if attn_weights_to_pl is None:
            attn_weights_to_pl = np.zeros(map_valid.shape[0]) - 1

        for type_to_draw in range(len(self.lane_style)):
            if type_to_draw not in self.types_to_draw:
                continue
            for i in np.where((map_type[:, type_to_draw]) & mask_valid)[0]:
                color, thickness = self.lane_style[type_to_draw]
                if self.types_to_draw == [0,1]:
                    color, thickness = COLOR_ALUMINIUM_3, 6
                if attn_weights_to_pl[type_to_draw] > 0:
                    color = tuple(np.array(color) * attn_weights_to_pl[i])
                valid_points = self._to_pixel(map_pos[i][map_valid[i]])
                if len(valid_points) > 2:
                    valid_points = valid_points[1:]  # Remove the first and last two points
                cv2.polylines(
                    raster_map,
                    [valid_points],
                    isClosed=False,
                    color=color,
                    thickness=thickness,
                    lineType=cv2.LINE_AA,
                )
        return raster_map

    def save_map(self, base_dir: Union[str, None] = None, img_name: Union[str, None] = None):
        """
        Saves the current raster map to a specified directory with a specified image name.

        Args:
            base_dir (Union[str, None], optional): The base directory where the map will be saved. 
                If not provided, defaults to "map/{self.scenario_id}".
            img_name (Union[str, None], optional): The name of the image file. 
                If not provided, defaults to "{self.scenario_id}-map.png".

        Returns:
            numpy.ndarray: The raster map in RGB format.
        """
        
        if not base_dir:
            base_dir = Path(f"map/{self.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not img_name:
            img_name = f"{self.scenario_id}-map.png"
        file_path = Path(base_dir / img_name)
        cv2.imwrite(str(file_path), self.raster_map[:, :, ::-1])

        return self.raster_map[:, :, ::-1]

    def save_video(
        self,
        base_dir: Union[str, None] = None,
        video_name: Union[str, None] = None,
        start_step: int = 0,
        end_step: Union[int, None] = None,
    ) -> None:
        """
        Saves a video of the scenario from the specified start step to the end step.
        Args:
            base_dir (Union[str, None], optional): The base directory where the video will be saved. 
                If None, defaults to "map/{self.scenario_id}". Defaults to None.
            video_name (Union[str, None], optional): The name of the video file. 
                If None, defaults to "{self.scenario_id}.mp4". Defaults to None.
            start_step (int, optional): The starting step of the video. Defaults to 0.
            end_step (Union[int, None], optional): The ending step of the video. 
                If None, defaults to the length of the episode.
        Returns:
            list[np.ndarray]: A list of images (frames) used to create the video.
        """

        img_buffer: list[np.ndarray] = []
        if end_step == None:
            end_step = self.episode["agent/valid"].shape[1]

        # every time step
        for t in range(start_step, end_step):
            step_image = self._plot_step(t)
            img_buffer.append(step_image)

        # save it
        if not base_dir:
            base_dir = Path(f"map/{self.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not video_name:
            video_name = f"{self.scenario_id}.mp4"
        file_path = Path(base_dir) / video_name
        video_writer = cv2.VideoWriter(
            str(file_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            10,
            (self.raster_map.shape[1], self.raster_map.shape[0]),
        )  # 10 is the fps
        for im in img_buffer:
            video_writer.write(im[:, :, ::-1])
        video_writer.release()

        return [img[:, :, ::-1] for img in img_buffer]

    def save_step_img(
        self, t: int, base_dir: Union[str, None] = None, img_name: Union[str, None] = None
    ) -> None:
        """
        Saves an image of the current step in the scenario.
        Args:
            t (int): The current time step to visualize.
            base_dir (Union[str, None], optional): The base directory where the image will be saved. 
                If None, defaults to "map/{self.scenario_id}". Defaults to None.
            img_name (Union[str, None], optional): The name of the image file. 
                If None, defaults to "{self.scenario_id}-{t}.png". Defaults to None.
        Returns:
            numpy.ndarray: The image array in RGB format.
        """

        img = self._plot_step(t)
        if not base_dir:
            base_dir = Path(f"map/{self.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not img_name:
            img_name = f"{self.scenario_id}-{t}.png"
        file_path = base_dir / img_name
        cv2.imwrite(str(file_path), img[:, :, ::-1])

        return img[:, :, ::-1]

    def _plot_step(self, t: int, with_agents: bool = True, with_tls: bool = True, only_valid_agents: bool = False) -> np.ndarray:
        """
        Plots a single simulation step on the raster map.
        Args:
            t (int): The time step to plot.
            with_agents (bool, optional): Whether to include agents in the plot. Defaults to True.
            with_tls (bool, optional): Whether to include traffic lights in the plot. Defaults to True.
            only_valid_agents (bool, optional): Whether to include only valid agents. Defaults to False.
        Returns:
            np.ndarray: The image of the plotted step.
        """

        step_image = self.raster_map.copy()

        if with_tls:
            tl_stop_valid = self.episode["tl_stop/valid"][:, t]
            tl_stop_state = self.episode["tl_stop/state"][:, t]

            for i in range(tl_stop_valid.shape[0]):
                if not tl_stop_valid[i]:
                    continue
                tl_state = tl_stop_state[i].argmax()
                if self.simplifed_tls_color:
                    tl_state_mapping = {0: 0, 1: 4, 2: 5, 3: 6, 4: 4, 5: 5, 6: 6, 7: 4, 8: 5}
                    tl_state = tl_state_mapping[tl_state]
                color = self.tl_style[tl_state]
                if self.shaded:
                    color = tuple(((np.array(COLOR_ALUMINIUM_5) + np.array(color)) / 2).astype(int).tolist())

                if self.tls_style == "tlstop":
                    stop_point = self._to_pixel(self.episode["tl_stop/pos"][i])
                    stop_point_end = self._to_pixel(
                        self.episode["tl_stop/pos"][i] + 5 * self.episode["tl_stop/dir"][i]
                    )
                    cv2.arrowedLine(
                        step_image,
                        stop_point,
                        stop_point_end,
                        color=color,
                        thickness=4,
                        line_type=cv2.LINE_AA,
                        tipLength=0.3,
                    )
                elif self.tls_style == "tllane":
                    lane_idx = self.episode["tl_lane/idx"][i]
                    pos = self._to_pixel(
                        self.episode["map/pos"][lane_idx][self.episode["map/valid"][lane_idx]]
                    )
                    cv2.polylines(
                        step_image,
                        [pos],
                        isClosed=False,
                        color=color,
                        thickness=8,
                        lineType=cv2.LINE_AA,
                    )
                    if tl_state >= 1 and tl_state <= 3:
                        cv2.drawMarker(
                            step_image,
                            pos[-1],
                            color=color,
                            markerType=cv2.MARKER_TILTED_CROSS,
                            markerSize=10,
                            thickness=6,
                        )

        if with_agents:
            # ! draw gt agents
            ag_valid = self.episode["agent/valid"][:, t]  # [n_ag]
            if only_valid_agents:
                ag_valid = np.logical_and(ag_valid, self.episode["agent/valid"][:, 10])
            ag_pos = self.episode["agent/pos"][:, t]  # [n_ag, 2]
            ag_yaw_bbox = self.episode["agent/yaw_bbox"][:, t]  # [n_ag, 1]
            bbox_gt = self._to_pixel(
                self._get_agent_bbox(ag_valid, ag_pos, ag_yaw_bbox, self.episode["agent/size"])
            )
            heading_start = self._to_pixel(ag_pos[ag_valid])
            ag_yaw_bbox = ag_yaw_bbox[:, 0][ag_valid]
            heading_end = self._to_pixel(
                ag_pos[ag_valid][:, :2] + 1.5 * np.stack([np.cos(ag_yaw_bbox), np.sin(ag_yaw_bbox)], axis=-1)
            )
            agent_role = self.episode["agent/role"][ag_valid]
            for i in range(agent_role.shape[0]):
                if not agent_role[i].any():
                    color = COLOR_ALUMINIUM_0
                else:
                    color = self.agent_role_style[np.where(agent_role[i])[0].min()]
                if self.simplified_agent_color and not color == COLOR_CYAN:
                    color = COLOR_ALUMINIUM_0
                if self.shaded:
                    color = tuple(((np.array(COLOR_ALUMINIUM_5) + np.array(color)) / 2).astype(int).tolist())

                cv2.fillConvexPoly(step_image, bbox_gt[i], color=color)
                cv2.arrowedLine(
                    step_image,
                    heading_start[i],
                    heading_end[i],
                    color=COLOR_BLACK,
                    thickness=4,
                    line_type=cv2.LINE_AA,
                    tipLength=0.6,
                )
        return step_image

    def _to_pixel(self, pos: np.ndarray) -> np.ndarray:
        """
        Converts a position from meters to pixels.
        This method transforms a given position from meters to pixels based on the 
        pixels per meter (px_per_m) and the top-left pixel offset (top_left_px). 
        The transformation includes scaling the position and adjusting the origin 
        to match the pixel coordinate system.
        Args:
            pos (np.ndarray): A numpy array representing the position in meters. 
                              The array should have at least two dimensions.
        Returns:
            np.ndarray: A numpy array representing the position in pixels, rounded 
                        to the nearest integer and cast to int32 type.
        """

        pos = pos[..., :2] * self.px_per_m
        pos[..., 0] = pos[..., 0] - self.top_left_px[0]
        pos[..., 1] = -pos[..., 1] - self.top_left_px[1]
        return np.round(pos).astype(np.int32)

    @staticmethod
    def _get_agent_bbox(
        agent_valid: np.ndarray, agent_pos: np.ndarray, agent_yaw: np.ndarray, agent_size: np.ndarray
    ) -> np.ndarray:
        """
        Calculate the bounding box for each valid agent.
        Args:
            agent_valid (np.ndarray): A boolean array indicating which agents are valid.
            agent_pos (np.ndarray): An array of shape (n, 2) representing the positions of the agents.
            agent_yaw (np.ndarray): An array of shape (n,) representing the yaw (orientation) of the agents.
            agent_size (np.ndarray): An array of shape (n, 2) representing the size (width, length) of the agents.
        Returns:
            np.ndarray: An array of shape (n, 4, 2) representing the bounding boxes of the valid agents.
                        Each bounding box is defined by 4 vertices in 2D space.
        """
        
        yaw = agent_yaw[agent_valid]  # n, 1
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        v_forward = np.concatenate([cos_yaw, sin_yaw], axis=-1)  # n,2
        v_right = np.concatenate([sin_yaw, -cos_yaw], axis=-1)

        offset_forward = 0.5 * agent_size[agent_valid, 0:1] * v_forward  # [n, 2]
        offset_right = 0.5 * agent_size[agent_valid, 1:2] * v_right  # [n, 2]

        vertex_offset = np.stack(
            [
                -offset_forward + offset_right,
                offset_forward + offset_right,
                offset_forward - offset_right,
                -offset_forward - offset_right,
            ],
            axis=1,
        )  # n,4,2

        agent_pos = agent_pos[agent_valid]
        bbox = agent_pos[:, None, :2].repeat(4, 1) + vertex_offset  # n,4,2
        return bbox
