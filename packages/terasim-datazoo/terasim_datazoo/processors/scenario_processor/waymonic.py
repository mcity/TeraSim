from pathlib import Path
from typing import Union
import os
import matplotlib.pyplot as plt
import json

from waymo_open_dataset.protos import scenario_pb2, map_pb2

from .waymonizer import Waymonizer
from .waymonic_tlsgen import WaymonicTLSGenerator
from terasim_datazoo.core.waymo import *


class Waymonic(Waymonizer, WaymonicTLSGenerator):

    def __init__(self, scenario) -> None:
        self.scenario = scenario
        Waymonizer.__init__(self, scenario)
        WaymonicTLSGenerator.__init__(self, scenario, self.lanecenters, self.signalized_intersections)

    def plot_waymonic_map(
        self,
        base_dir: Union[Path, None] = None,
        filename: Union[str, None] = None,
        fontsize: float = 5,
        stop_signs: bool = True,
        traffic_lights: bool = True,
        intersections: bool = False,
        dpi: float = 300,
    ) -> None:
        """
        Plots a waymonic map of the scenario with various elements such as lane centers, stop signs, and traffic lights.

        Args:
            base_dir (Union[Path, None], optional): The base directory where the map will be saved. Defaults to "map/{scenario_id}".
            filename (Union[str, None], optional): The filename for the saved map image. Defaults to "{scenario_id}-waymonic.png".
            fontsize (float, optional): The font size for the lane center IDs. Defaults to 5.
            stop_signs (bool, optional): Whether to plot stop signs. Defaults to True.
            traffic_lights (bool, optional): Whether to plot traffic lights. Defaults to True.
            intersections (bool, optional): Whether to plot intersections. Defaults to False.
            dpi (float, optional): The resolution of the saved map image. Defaults to 300.

        Returns:
            None
        """

        if not base_dir:
            base_dir = Path(f"map/{self.scenario.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not filename:
            filename = f"{self.scenario.scenario_id}-waymonic.png"
        file_path: Path = base_dir / filename

        colors = ["b", "g", "r", "c", "m", "y", "k", "#50DC9F", "#8A0FC3", "#0F92C3"]
        plt.figure(figsize=(20, 20))

        # lanecenters
        intersection_lcs = [
            id for group in self.signalized_intersections + self.stop_intersections for id in group
        ]
        normal_lcs = (
            set(self.lanecenters.keys()).difference(intersection_lcs)
            if intersections
            else self.lanecenters.keys()
        )

        if intersections:
            for i, group in enumerate(self.signalized_intersections):
                # color = colors[i % len(colors)]
                for id in group:
                    plt.plot(
                        [point.x for point in self.lanecenters[id].lane.polyline],
                        [point.y for point in self.lanecenters[id].lane.polyline],
                        c="grey",
                        linewidth=1,
                    )
                    mid_point = self.lanecenters[id].lane.polyline[
                        len(self.lanecenters[id].lane.polyline) // 2
                    ]
                    plt.text(mid_point.x, mid_point.y, str(id), fontsize=fontsize)

            for i, group in enumerate(self.stop_intersections):
                # color = colors[i % len(colors)]
                for id in group:
                    plt.plot(
                        [point.x for point in self.lanecenters[id].lane.polyline],
                        [point.y for point in self.lanecenters[id].lane.polyline],
                        c="grey",
                        linewidth=1,
                        linestyle="--",
                    )

        for id in normal_lcs:
            color = colors[id % len(colors)]
            if self.lanecenters[id].lane.interpolating:
                color = "orange"
            plt.plot(
                [point.x for point in self.lanecenters[id].lane.polyline],
                [point.y for point in self.lanecenters[id].lane.polyline],
                c=color,
            )
            mid_point = self.lanecenters[id].lane.polyline[len(self.lanecenters[id].lane.polyline) // 2]
            plt.text(mid_point.x, mid_point.y, str(id), fontsize=fontsize)
            plt.scatter(
                self.lanecenters[id].lane.polyline[0].x,
                self.lanecenters[id].lane.polyline[0].y,
                color="grey",
                marker="o",
                s=50,
            )
            plt.scatter(
                self.lanecenters[id].lane.polyline[0].x,
                self.lanecenters[id].lane.polyline[0].y,
                color="grey",
                marker="o",
                s=50,
            )

        # stop signs
        if stop_signs:
            for point in self.stop_signs:
                plt.scatter(point.x, point.y, color="green", marker="D")

        # traffic light heads
        if traffic_lights:
            for point in self.traffic_lights:
                plt.scatter(point.x, point.y, color="blue", marker="X")

        plt.gca().set_aspect("equal", adjustable="box")
        plt.savefig(file_path, dpi=dpi)
        plt.clf()
        plt.close()
        print(f"[File Output] Plotted waymonic map: {file_path} ...")
