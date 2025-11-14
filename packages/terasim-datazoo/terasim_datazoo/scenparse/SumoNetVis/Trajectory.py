"""
Tools for plotting trajectories.
"""

import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
import numpy as np
import warnings


GENERIC_PARAM_MISSING_VALUE = None  # value to assign for timesteps when a generic parameter is missing


class Trajectory:
    """
    Class containing a single trajectory.

    :ivar mappable: ``ScalarMappable`` object. Useful for generating colorbars. None if no applicable colorization.
    """
    def __init__(self, id, type, time=None, x=None, y=None, speed=None, angle=None, lane=None, colors=None, params=None):
        self.id = id
        self.type = type
        self.point_plot_kwargs = {"color": "blue", "ms": 5, "markeredgecolor": "black", "zorder": 200}
        self.time = time if time is not None else []
        self.x = x if x is not None else []
        self.y = y if y is not None else []
        self.speed = speed if speed is not None else []
        self.angle = angle if angle is not None else []
        self.lane = lane if lane is not None else []
        self.colors = colors if colors is not None else []
        self.params = params if params is not None else dict()
        self._mappable = None  # type: plt.cm.ScalarMappable

    @property
    def mappable(self):
        """
        ``ScalarMappable`` object corresponding to colorization of trajectory. Useful for generating colorbars, like so:
        ``plt.colorbar(trajectory.mappable)``
        """
        return self._mappable

    def _append_point(self, time, x, y, speed=None, angle=None, lane=None, color="#000000", params=None):
        """
        Appends a point to the trajectory

        :type time: float
        :type x: float
        :type y: float
        :type speed: float
        :type angle: float
        :type lane: str
        :type color: str
        :type params: dict
        :return: None
        """
        self.time.append(time)
        self.x.append(x)
        self.y.append(y)
        self.speed.append(speed)
        self.angle.append(angle)
        self.lane.append(lane)
        self.colors.append(color)
        params = params if params is not None else dict()
        for key in params:
            if key not in self.params:
                self.params[key] = []
        for key in self.params:
            while len(self.params[key]) < len(self.time)-1:
                self.params[key].append(GENERIC_PARAM_MISSING_VALUE)
            self.params[key].append(params[key] if key in params else GENERIC_PARAM_MISSING_VALUE)

    def assign_colors_constant(self, color):
        """
        Assigns a constant color to the trajectory

        :param color: desired color
        :return: None
        """
        self.colors = [color for i in self.x]

    def assign_colors_speed(self, cmap=None, min_speed=0, max_speed=None):
        """
        Assigns colors to trajectory points based on the speed.

        :param cmap: cmap object or name of cmap to use
        :param min_speed: speed corresponding to low end of the color scale. If None, trajectory's min value is used
        :param max_speed: speed corresponding to high end of the color scale. If None, trajectory's max value is used
        :return: None
        :type min_speed: float
        :type max_speed: float
        """
        if cmap is None:
            cmap = plt.cm.get_cmap("viridis")
        elif type(cmap) == str:
            cmap = plt.cm.get_cmap(cmap)
        if min_speed is None:
            min_speed = min(self.speed)
        if max_speed is None:
            max_speed = max(self.speed)
        self._mappable = plt.cm.ScalarMappable(cmap=cmap, norm=Normalize(vmin=min_speed, vmax=max_speed))
        for i in range(len(self.x)):
            self.colors[i] = self._mappable.to_rgba(self.speed[i])

    def assign_colors_angle(self, cmap=None, angle_mode="deg"):
        """
        Assigns colors to trajectory points based on the angle.

        :param cmap: cmap object or name of cmap to use
        :param angle_mode: units of the angle value. "deg", "rad", or "grad"
        :type angle_mode: str
        :return: None
        """
        if cmap is None:
            cmap = plt.cm.get_cmap("hsv")
        elif type(cmap) == str:
            cmap = plt.cm.get_cmap(cmap)
        max_angles = {"deg": 360, "rad": 2*np.pi, "grad": 400}
        max_angle = max_angles[angle_mode]
        self._mappable = plt.cm.ScalarMappable(cmap=cmap, norm=Normalize(vmin=0, vmax=max_angle))
        for i in range(len(self.x)):
            self.colors[i] = self._mappable.to_rgba(self.angle[i])

    def assign_colors_lane(self, cmap=None, color_dict=None):
        """
        Assigns colors to the trajectory points based on the lane value

        :param cmap: cmap object or name of cmap to use to color lanes
        :param color_dict: dict to override random color selection. Keys are lane IDs, values are colors.
        :return: None
        :type color_dict: dict
        """
        if cmap is None:
            cmap = plt.cm.get_cmap("tab10")
        elif type(cmap) == str:
            cmap = plt.cm.get_cmap(cmap)
        cmapList = cmap.colors
        if color_dict is None:
            color_dict = dict()
            lane_list = set(self.lane)
            for i, lane in enumerate(lane_list):
                color_dict[lane] = cmapList[i % len(cmapList)]
        for i in range(len(self.x)):
            self.colors[i] = color_dict[self.lane[i]]

    def assign_colors_param(self, key, transformation=None, cmap=None, vmin=None, vmax=None):
        """
        Assigns colors based on values of the generic parameter with the given key. Colors can be assigned in one of
        two ways:

        * Provide a cmap. ``transformation`` will be interpreted as a matplotlib norm (default Normalize(vmin, vmax)).
        * Provide a transformation and no cmap, where ``transformation`` has signature transformation(param) -> color.

        In the first case, the trajectory mappable will be generated, allowing automatic colorbar creation.
        If neither a transformation nor a cmap is given, the parameter value will be interpreted as a color.

        :param key: generic parameter key
        :param transformation: (optional) callable if no cmap provided or matplotlib norm if cmap provided
        :param cmap: (optional): cmap to use to assign colors
        :param vmin: minimum value for norm (used to scale values onto cmap)
        :param vmax: maximum value for norm (used to scale values onto cmap)
        :return: None
        :type key: str
        """
        if cmap is not None:
            vmin = vmin if vmin is not None else 0
            vmax = vmax if vmax is not None else 1
            norm = transformation if transformation is not None else Normalize(vmin=vmin, vmax=vmax)
            self._mappable = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
            transformation = self._mappable.to_rgba
        elif transformation is None:
            transformation = lambda x: x
            if vmin is not None or vmax is not None:
                warnings.warn("Parameters vmin and vmax ignored because no cmap was specified.")
        if not callable(transformation):
            raise TypeError("Transformation must be callable.")
        for i, val in enumerate(self.params[key]):
            _val = val if cmap is None else float(val)
            self.colors[i] = transformation(_val)

    def _get_values_at_time(self, time):
        """
        Returns all of the values at the given simulation time

        :param time: Sumo simulation time for which to fetch values
        :return: dict containing x, y, speed, angle, lane, color, and generic parameter values at time
        """
        try:
            idx = self.time.index(time)
        except ValueError:
            return {"x": None,
                    "y": None,
                    "speed": None,
                    "angle": None,
                    "lane": None,
                    "color": None,
                    **{key: None for key in self.params}}
        else:
            return {"x": self.x[idx],
                    "y": self.y[idx],
                    "speed": self.speed[idx],
                    "angle": self.angle[idx],
                    "lane": self.lane[idx],
                    "color": self.colors[idx],
                    **{key: self.params[key][idx] for key in self.params}}

    def plot(self, ax=None, start_time=0, end_time=np.inf, zoom_to_extents=False, **kwargs):
        """
        Plots the trajectory

        :param ax: matplotlib Axes object. Defaults to current axes.
        :param start_time: time at which to start drawing
        :param end_time: time at which to end drawing
        :param zoom_to_extents: if True, window will be set to Trajectory extents
        :param kwargs: keyword arguments to pass to LineCollection or matplotlib.pyplot.plot()
        :type ax: plt.Axes
        :type start_time: float
        :type end_time: float
        :type zoom_to_extents: bool
        :return: artist (LineCollection)
        """
        if ax is None:
            ax = plt.gca()
        if len(self.x) < 2:
            return
        segs, colors = [], []  # line segments and colors
        x_min, y_min, x_max, y_max = np.inf, np.inf, -np.inf, -np.inf
        for i in range(len(self.x)-2):
            if self.time[i] < start_time:
                continue
            if self.time[i+1] > end_time:
                break
            if zoom_to_extents:
                x_min, x_max = min(x_min, self.x[i]), max(x_max, self.x[i])
                y_min, y_max = min(y_min, self.y[i]), max(y_max, self.y[i])
            segs.append([[self.x[i], self.y[i]], [self.x[i+1], self.y[i+1]]])
            colors.append(self.colors[i])
        lc = LineCollection(segs, colors=colors, **kwargs)
        lc.sumo_object = self
        ax.add_collection(lc)
        if zoom_to_extents:
            dx, dy = x_max - x_min, y_max - y_min
            ax.set_xlim([x_min-0.05*dx, x_max+0.05*dx])
            ax.set_ylim([y_min-0.05*dy, y_max+0.05*dy])
        return lc


class Trajectories:
    """
    Object storing a collection of trajectories.

    Individual trajectories can be retrieved by indexing with a number or by vehID. The object is also iterable.

    :param file: file from which to read trajectories. Currently only FCD exports supported.
    :type file: str

    :ivar mappables: dict of ``vehID: ScalarMappable`` pairs. Useful for generating colorbars.
    """
    def __init__(self, file=None):
        """
        Initializes a Trajectories object.

        :param file: file from which to read trajectories
        :type file: str
        """
        self.trajectories = []  # type: list[Trajectory]
        self.graphics = dict()
        self.start = None
        self.end = None
        self.timestep = None
        if file is not None:
            if file.endswith("fcd-output.xml"):
                self.read_from_fcd(file)
            else:
                raise NotImplementedError("Reading from this type of file not implemented: " + file)

    def __iter__(self):
        return iter(self.trajectories)

    def __next__(self):
        return next(self.trajectories)

    def __getitem__(self, i):
        if type(i) == str:
            for trajectory in self.trajectories:
                if trajectory.id == i:
                    return trajectory
            raise IndexError
        elif type(i) == int:
            return self.items[i]
        else:
            raise TypeError("Index type " + type(i).__name__ + " not supported by class " + type(self).__name__)

    def timestep_range(self):
        """
        Returns a numpy ndarray consisting of every simulation time

        :return: ndarray of all simulation times
        """
        return np.arange(self.start, self.end, self.timestep)

    def _append(self, trajectory):
        self.trajectories.append(trajectory)

    def read_from_fcd(self, file):
        """
        Reads trajectories from Sumo floating car data (fcd) output file.

        :param file: Sumo fcd output file
        :return: None
        :type file: str
        """
        root = ET.parse(file).getroot()
        trajectories = dict()
        for timestep in root:
            time = float(timestep.attrib["time"])
            if self.timestep is None and self.start is not None:
                self.timestep = time - self.start
            if self.start is None:
                self.start = time
            self.end = time
            for veh in timestep:
                if veh.tag in ["vehicle", "person", "container"]:
                    vehID = veh.attrib["id"]
                    type = veh.attrib.get("type", "")
                    if vehID not in trajectories:
                        trajectories[vehID] = Trajectory(vehID, type)
                    x = float(veh.attrib["x"])
                    y = float(veh.attrib["y"])
                    lane = veh.attrib.get("lane", "")
                    speed = float(veh.attrib["speed"])
                    angle = float(veh.attrib["angle"])
                    params = {key: veh.attrib[key] for key in veh.attrib if key not in ["id", "type", "x", "y", "lane", "speed", "angle"]}
                    trajectories[vehID]._append_point(time, x, y, speed, angle, lane, params=params)
                    for vehChild in veh:
                        if vehChild.tag in ["person", "container"]:
                            objID = vehChild.attrib["id"]
                            type = vehChild.attrib.get("type", "")
                            if objID not in trajectories:
                                trajectories[objID] = Trajectory(objID, type)
                            x = float(vehChild.attrib["x"]) if "x" in vehChild.attrib else x
                            y = float(vehChild.attrib["y"]) if "y" in vehChild.attrib else y
                            lane = vehChild.attrib.get("lane", "")
                            speed = float(vehChild.attrib["speed"]) if "speed" in vehChild.attrib else speed
                            angle = float(vehChild.attrib["angle"]) if "angle" in vehChild.attrib else angle
                            params = {key: vehChild.attrib[key] for key in vehChild.attrib if key not in ["id", "type", "x", "y", "lane", "speed", "angle"]}
                            params = {"_parent_vehicle": vehID, **params}
                            trajectories[objID]._append_point(time, x, y, speed, angle, lane, params=params)
        for vehID in trajectories:
            self._append(trajectories[vehID])

    @property
    def mappables(self):
        """
        dict of ``vehID: ScalarMappable`` pairs. Useful for generating colorbars, like so:
        ``plt.colorbar(trajectories.mappables[vehID])``
        """
        return {traj.id: traj.mappable for traj in self.trajectories if traj.mappable is not None}

    def plot(self, ax=None, start_time=0, end_time=np.inf, **kwargs):
        """
        Plots all of the trajectories contained in this object.

        :param ax: matplotlib Axes object. Defaults to current axes.
        :param start_time: time at which to start drawing
        :param end_time: time at which to stop drawing
        :param kwargs: keyword arguments to pass to plot function
        :return: list of artists (LineCollection objects)
        :type ax: plt.Axes
        :type start_time: float
        :type end_time: float
        """
        artists = []
        if ax is None:
            ax = plt.gca()
        for trajectory in self:
            artist = trajectory.plot(ax, start_time, end_time, **kwargs)
            artists.append(artist)
        return artists

    def plot_points(self, time, ax=None, animate_color=False):
        """
        Plots the position of each vehicle at the specified time as a point.
        The style for each point is controlled by each Trajectory's point_plot_kwargs attribute.

        :param time: simulation time for which to plot vehicle positions.
        :param ax: matplotlib Axes object. Defaults to current axes.
        :param animate_color: If True, the color of the marker will be animated using the Trajectory's color values.
        :return: matplotlib Artist objects corresponding to the rendered points. Required for blitting animation.
        :type time: float
        :type ax: plt.Axes
        :type animate_color: bool
        """
        if ax is None:
            ax = plt.gca()
        for traj in self.trajectories:
            values = traj._get_values_at_time(time)
            x, y = values["x"], values["y"]
            angle = values["angle"]
            color = values["color"]
            if x is None or y is None:
                if time >= np.nanmax(traj.time):
                    if traj in self.graphics:
                        self.graphics[traj].remove()
                        self.graphics.pop(traj)
                continue
            if animate_color and color is not None:
                traj.point_plot_kwargs["color"] = color
            angle = (360 - angle) % 360
            if traj not in self.graphics:
                self.graphics[traj], = ax.plot(x, y, marker=(3, 0, angle), **traj.point_plot_kwargs)
                self.graphics[traj].sumo_object = traj
            else:
                self.graphics[traj].set_xdata(x)
                self.graphics[traj].set_ydata(y)
                self.graphics[traj].set_marker((3, 0, angle))
                if animate_color:
                    self.graphics[traj].set_color(traj.point_plot_kwargs["color"])
        return tuple(self.graphics[traj] for traj in self.graphics)


if __name__ == "__main__":
    trajectories = Trajectories("../2019-08-30-17-01-38fcd-output.xml")
    fig, ax = plt.subplots()
    trajectories["TESIS_0"].assign_colors_angle()
    trajectories["TESIS_0"].plot(ax, lw=3)
    plt.show()
