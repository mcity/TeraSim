"""
Main classes and functions for dealing with a Sumo network.
"""

import warnings
import xml.etree.ElementTree as ET
from shapely.geometry import *
from shapely.geometry.polygon import orient
import shapely.ops as ops
import matplotlib.patches
import matplotlib.transforms as transforms
import matplotlib.pyplot as plt
import numpy as np
from typing import Union
from terasim_vis import Utils
from terasim_vis import Additionals as _Addls

DEFAULT_LANE_WIDTH = 3.2
STRIPE_WIDTH_SCALE_FACTOR = 1  # factor by which to scale striping widths
COLOR_SCHEME = {
    "junction": "#660000",
    "pedestrian": "#808080",
    "bicycle": "#C0422C",
    "ship": "#96C8C8",
    "authority": "#FF0000",
    "none": "#FFFFFF",
    "no_passenger": "#5C5C5C",
    "crosswalk": "#00000000",
    "other": "#000000"
}
USA_STYLE = "USA"
EUR_STYLE = "EUR"
LANE_MARKINGS_STYLE = EUR_STYLE  # desired lane marking style
PLOT_STOP_LINES = True  # whether to plot stop lines

OBJ_TERRAIN_CLEANUP_TOLERANCE = 0.01  # clean terrain boundaries by dilating then eroding the net bounds by this amount


def set_style(style=None, plot_stop_lines=None):
    """
    Sets the lane marking style settings.

    :param style: desired style ("USA" or "EUR")
    :param plot_stop_lines: whether to plot stop lines
    :return: None
    :type style: str
    :type plot_stop_lines: bool
    """
    global LANE_MARKINGS_STYLE, PLOT_STOP_LINES
    if style is not None:
        if style not in [USA_STYLE, EUR_STYLE]:
            raise IndexError("Specified lane marking style not supported: " + style)
        LANE_MARKINGS_STYLE = style
    if plot_stop_lines is not None:
        PLOT_STOP_LINES = plot_stop_lines


def set_stripe_width_scale(factor=1):
    """
    Sets the lane striping width scale factor.

    :param factor: desired scale factor
    :return: None
    :type factor: float
    """
    global STRIPE_WIDTH_SCALE_FACTOR
    STRIPE_WIDTH_SCALE_FACTOR = factor


class _Edge:
    def __init__(self, attrib):
        """
        Initializes an Edge object.

        :param attrib: dict of Edge attributes
        :type attrib: dict
        """
        self.id = attrib["id"]
        self.function = attrib["function"] if "function" in attrib else "normal"
        self.from_junction_id = attrib["from"] if "from" in attrib else None
        self.to_junction_id = attrib["to"] if "to" in attrib else None
        self.from_junction = None
        self.to_junction = None
        self.lanes = []
        self.stop_offsets = []
        self.params = dict()

    def append_lane(self, lane):
        """
        Makes the specified Lane a child of the Edge

        :param lane: child Lane object
        :return: None
        :type lane: _Lane
        """
        self.lanes.append(lane)
        lane.parentEdge = self

    def get_lane(self, index):
        """
        Returns the lane on the Edge with the given index

        :param index: lane index of Lane to retrieve
        :return: Lane
        """
        for lane in self.lanes:
            if lane.index == index:
                return lane
        # if error occurs, check the index is correct
        print(f"Edge contains no Lane with given index: {index}")
        print(f"Edge: {self.id}")
        print(f"Lanes: {self.lanes}")
        raise IndexError("Edge contains no Lane with given index.")

    def lane_count(self):
        """
        Returns the number of lanes to which this Edge is a parent

        :return: lane count
        """
        return len(self.lanes)

    def intersects(self, other):
        """
        Checks if any lane in the edge intersects a specified geometry

        :param other: the geometry against which to check
        :return: True if any lane intersects other, else False
        :type other: BaseGeometry
        """
        for lane in self.lanes:
            if other.intersects(lane.shape):
                return True
        return False

    def append_stop_offset(self, attrib):
        value = float(attrib["value"])
        vc = attrib["vClasses"] if "vClasses" in attrib else ""
        exceptions = attrib["exceptions"] if "exceptions" in attrib else ""
        vClasses = Utils.Allowance(allow_string=vc, disallow_string=exceptions)
        self.stop_offsets.append((value, vClasses))

    def plot(self, ax, lane_kwargs=None, lane_marking_kwargs=None, tl_phases=None, **kwargs):
        """
        Plots the lane.
        The lane_kwargs and lane_markings_kwargs override the general kwargs for their respective functions.

        :param ax: matplotlib Axes object
        :param lane_kwargs: kwargs to pass to the lane plotting function (matplotlib.patches.Polygon())
        :param lane_marking_kwargs: kwargs to pass to the lane markings plotting function (matplotlib.lines.Line2D())
        :return: list of lane artists, list of lane marking artists
        :type ax: plt.Axes
        """
        lane_artists, lane_marking_artists = [], []
        if lane_kwargs is None:
            lane_kwargs = dict()
        if lane_marking_kwargs is None:
            lane_marking_kwargs = dict()
        for lane in self.lanes:
            lane_artist = lane.plot_shape(ax, **{**kwargs, **lane_kwargs})
            lane_marking_artist = lane.plot_lane_markings(ax, tl_phases=tl_phases, **{**kwargs, **lane_marking_kwargs})
            lane_artists.append(lane_artist)
            lane_marking_artists += lane_marking_artist
        return lane_artists, lane_marking_artists


class _LaneMarking:
    def __init__(self, alignment, linewidth, color, dashes, purpose=None, parent=None):
        """
        Initialize a lane marking object.

        :param alignment: the centerline alignment of the lane marking
        :param linewidth: the width of the marking
        :param color: the color of the marking
        :param dashes: dash pattern of the marking
        :param purpose: string describing what function the marking serves
        :param parent: _Lane object which created the lane marking
        """
        self.purpose = "" if purpose is None else purpose
        self.alignment = alignment
        self.linewidth = linewidth
        self.color = color
        self.dashes = dashes
        self.parent_lane = parent

    def plot(self, ax, **kwargs):
        """
        Plots the lane marking.

        :param ax: matplotlib Axes object
        :param kwargs: kwargs to pass to Line2D
        :return: artist
        :type ax: plt.Axes
        """
        color = kwargs.pop("color") if "color" in kwargs else self.color
        x, y = zip(*self.alignment.coords)
        line = Utils.LineDataUnits(x, y, linewidth=self.linewidth, color=color, dashes=self.dashes, **kwargs)
        line.sumo_object = self
        ax.add_line(line)
        return line

    def get_as_shape(self, cap_style=CAP_STYLE.flat):
        """
        Get marking as a shapely Polygon or MultiPolygon

        :param cap_style: cap style to use when performing buffer
        :return: shapely Polygon or MultiPolygon
        """
        if self.dashes[1] == 0:  # if solid line
            buffer = self.alignment.buffer(self.linewidth / 2, cap_style=cap_style)
        else:  # if dashed line
            buffer = MultiPolygon()
            dash_length, gap = self.dashes
            for s in np.arange(0, self.alignment.length, dash_length + gap):
                assert hasattr(ops, "substring"), "Shapely>=1.7.0 is required for OBJ export of dashed lines."
                dash_segment = ops.substring(self.alignment, s, min(s + dash_length, self.alignment.length))
                buffer = buffer.union(dash_segment.buffer(self.linewidth / 2, cap_style=cap_style))
        return buffer

    def get_as_3d_object(self, z=0.001, extrude_height=0, include_bottom_face=False):
        """
        Generates an Object3D from the marking.

        :param z: z coordinate of marking
        :param extrude_height: distance by which to extrude the marking
        :param include_bottom_face: whether to include the bottom face of the extruded geometry.
        :return: Object3D
        :type z: float
        :type extrude_height: float
        :type include_bottom_face: bool
        """
        shape = self.get_as_shape()
        if shape.geom_type == "Polygon":
            oriented_shape = MultiPolygon([orient(shape)])
        elif shape.geom_type in ["MultiPolygon", "GeometryCollection"]:
            oriented_geoms = []
            for geom in shape:
                if geom.geom_type == "Polygon":
                    oriented_geoms.append(orient(geom))
            oriented_shape = MultiPolygon(oriented_geoms)
        return Utils.Object3D.from_shape(oriented_shape, self.purpose+"_marking", self.color+"_marking", z=z, extrude_height=extrude_height, include_bottom_face=include_bottom_face)


class _Lane:
    def __init__(self, attrib):
        """
        Initialize a Lane object.

        :param attrib: dict of all of the lane attributes
        :type attrib: dict
        """
        self.id = attrib["id"]
        self.index = int(attrib["index"])
        self.speed = float(attrib["speed"])
        allow_string = attrib["allow"] if "allow" in attrib else ""
        disallow_string = attrib["disallow"] if "disallow" in attrib else ""
        self.allows = Utils.Allowance(allow_string, disallow_string)
        self.width = float(attrib["width"]) if "width" in attrib else DEFAULT_LANE_WIDTH
        self.endOffset = attrib["endOffset"] if "endOffset" in attrib else 0
        self.acceleration = attrib["acceleration"] if "acceleration" in attrib else "False"
        coords = [[float(coord) for coord in xy.split(",")[0:2]] for xy in attrib["shape"].split(" ")]
        self.alignment = LineString(coords)
        self.shape = self.alignment.buffer(self.width/2, cap_style=CAP_STYLE.flat)
        if self.shape.geom_type != "Polygon":
            self.shape = self.shape.buffer(0)
            if self.shape.geom_type == "MultiPolygon":
                warnings.warn("Shape of lane " + self.id + " is MultiPolygon. Ignoring all but largest polygon.")
                self.shape = sorted(self.shape, key=lambda x: x.area)[-1]
        self.parentEdge = None
        self.stop_offsets = []
        self.incoming_connections = []
        self.outgoing_connections = []
        self.requests = []  # type: list[_Request]
        self.params = dict()

    def lane_type(self):
        """
        Returns a string descriptor of the type of lane, based on vehicle permissions.

        :return: lane type
        """
        if self.allows == "pedestrian":
            if self.parentEdge is not None and self.parentEdge.function == "crossing":
                return "crosswalk"
            else:
                return "pedestrian"
        if self.allows == "bicycle":
            return "bicycle"
        if self.allows == "ship":
            return "ship"
        if self.allows == "authority":
            return "authority"
        if self.allows == "none":
            return "none"
        if not self.allows["passenger"]:
            return "no_passenger"
        else:
            return "other"

    def lane_color(self):
        """
        Returns the Sumo-GUI default lane color for this lane.

        :return: lane color
        """
        type = self.lane_type()
        return COLOR_SCHEME[type] if type in COLOR_SCHEME else COLOR_SCHEME["other"]

    def plot_alignment(self, ax):
        """
        Plots the centerline alignment of the lane

        :param ax: matplotlib Axes object
        :return: artist
        :type ax: plt.Axes
        """
        x, y = zip(*self.alignment.coords)
        artist, = ax.plot(x, y)
        return artist

    def plot_shape(self, ax, **kwargs):
        """
        Plots the entire shape of the lane

        :param ax: matplotlib Axes object
        :return: artist
        :type ax: plt.Axes
        """
        if "lw" not in kwargs and "linewidth" not in kwargs:
            kwargs["lw"] = 0
        if "color" not in kwargs:
            kwargs["color"] = self.lane_color()
        try:
            poly = matplotlib.patches.Polygon(xy=self.shape.boundary.coords, closed=True, **kwargs)
        except NotImplementedError:
            warnings.warn("Can't plot non-polygonal geometry of lane " + self.id, stacklevel=2)
        else:
            poly.sumo_object = self
            ax.add_patch(poly)
            return poly

    def inverse_lane_index(self):
        """
        Returns the inverted lane index (i.e. counting from inside out)

        :return: inverted lane index
        """
        return self.parentEdge.lane_count() - self.index - 1

    def append_stop_offset(self, attrib):
        """
        Add a stop offset to the lane.

        :param attrib: dict of all the stop offset attributes
        :return: None
        :type attrib: dict
        """
        value = float(attrib["value"])
        vc = attrib["vClasses"] if "vClasses" in attrib else ""
        exceptions = attrib["exceptions"] if "exceptions" in attrib else ""
        vClasses = Utils.Allowance(allow_string=vc, disallow_string=exceptions)
        self.stop_offsets.append((value, vClasses))

    def get_stop_line_locations(self):
        """
        Return a list of stop line locations for the lane based on the stop offsets of the lane and its parent Edge.

        :return: list of stop line locations (distance from end of lane)
        """
        if len(self.stop_offsets) > 0:
            stop_offsets = self.stop_offsets
        else:
            stop_offsets = self.parentEdge.stop_offsets if self.parentEdge is not None else []
        stop_line_locations = []
        accrued_vClasses = Utils.Allowance("none")
        for stop_offset, vClasses in stop_offsets:
            accrued_vClasses += vClasses
            stop_line_locations.append(stop_offset)
        if not accrued_vClasses.is_superset_of(self.allows) and 0 not in stop_line_locations:
            stop_line_locations.append(0)
        return stop_line_locations

    def _requires_stop_line(self):
        """
        Determines whether the Lane should be drawn with a stop line based on its priority and to-Junction.

        :return: True if Lane should be drawn with stop line, else False
        """
        if self.parentEdge.to_junction.type in ["internal", "zipper"]:
            return False
        if self.parentEdge.to_junction.type == "always_stop":
            return True
        for request in self.requests:
            if "1" in request.response:
                return True
        return False

    def get_markings_as_3d_objects(self, z_lane=0, extrude_height=0, include_bottom_face=False):
        """
        Generates list of Object3D objects from the lane markings.

        :param z_lane: z coordinate of the lane. Markings will be generated slightly above this to prevent z fighting.
        :param extrude_height: distance by which to extrude the markings
        :param include_bottom_face: whether to include the bottom face of the extruded geometry.
        :return: Object3D
        :type z_lane: float
        :type extrude_height: float
        :type include_bottom_face: bool
        """
        objects = []
        for marking in self._guess_lane_markings():
            z = z_lane+0.002 if marking.purpose == "crossing" else z_lane+0.001
            try:
                obj = marking.get_as_3d_object(z=z, extrude_height=extrude_height, include_bottom_face=include_bottom_face)
            except (NotImplementedError, AttributeError):
                warnings.warn("Could not generate geometry for " + marking.purpose + " marking of lane " + self.id, stacklevel=2)
            else:
                objects.append(obj)
        return objects

    def get_as_3d_object(self, z=0, include_bottom_face=False, material_param=None, extrude_height_param=None,
                         extrude_height_param_transform=None):
        """
        Generates an Object3D from the lane.

        :param z: z coordinate of junction
        :param include_bottom_face: whether to include the bottom face of the extruded geometry.
        :param material_param: generic parameter to use to override material, if present
        :param extrude_height_param: generic parameter to use to override extrude height, if present
        :param extrude_height_param_transform: function to apply to extrude_height_param values. Defaults to str->float conversion.
        :return: Object3D
        :type z: float
        :type include_bottom_face: bool
        :type material_param: str
        :type extrude_height_param: str
        """
        if self.shape.is_empty:
            return None
        h = 0.15 if self.lane_type() == "pedestrian" else 0
        if extrude_height_param is not None and extrude_height_param in self.params:
            if extrude_height_param_transform is None:
                extrude_height_param_transform = lambda x: float(x) if x is not None else h
            h = extrude_height_param_transform(self.params[extrude_height_param])
        material = self.params.get(material_param, self.lane_type()+"_lane")
        return Utils.Object3D.from_shape(orient(self.shape), self.id, material, z=z, extrude_height=h, include_bottom_face=include_bottom_face)

    def _guess_lane_markings(self, tl_phases=None):
        """
        Guesses lane markings based on lane configuration and globally specified lane marking style.

        :return: dict containing the marking alignment, line width, color, and dash pattern.
        """
        markings = []
        if self.parentEdge.function == "internal" or self.allows == "ship" or self.allows == "rail":
            return markings
        if self.parentEdge.function == "crossing":
            color, dashes = "0.7", (0.2, 0.2)
            markings.append(_LaneMarking(self.alignment, self.width, color, dashes, purpose="crossing", parent=self))
            return markings
        # US-style markings
        if LANE_MARKINGS_STYLE == USA_STYLE:
            lw = 0.1 * STRIPE_WIDTH_SCALE_FACTOR
            # Draw centerline stripe if necessary
            if self.inverse_lane_index() == 0:
                leftEdge = self.alignment.parallel_offset(self.width/2-lw, side="left")
                color, dashes = "y", (100, 0)
                markings.append(_LaneMarking(leftEdge, lw, color, dashes, purpose="center", parent=self))
            # Draw non-centerline markings
            else:
                adjacent_lane = self.parentEdge.get_lane(self.index+1)
                leftEdge = self.alignment.parallel_offset(self.width/2, side="left")
                color, dashes = "w", (3, 9)  # set default settings
                if self.allows("bicycle") != adjacent_lane.allows("bicycle"):
                    dashes = (100, 0)  # solid line where bicycles may not change lanes
                elif self.allows("passenger") != adjacent_lane.allows("passenger"):
                    if self.allows("bicycle"):
                        dashes = (1, 3)  # short dashed line where bikes may change lanes but passenger vehicles not
                    else:
                        dashes = (100, 0)  # solid line where neither passenger vehicles nor bikes may not change lanes
                markings.append(_LaneMarking(leftEdge, lw, color, dashes, purpose="lane", parent=self))
            # draw outer lane marking if necessary
            if self.index == 0 and not (self.allows("pedestrian") and not self.allows("all")):
                rightEdge = self.alignment.parallel_offset(self.width/2, side="right")
                color, dashes = "w", (100, 0)
                markings.append(_LaneMarking(rightEdge, lw, color, dashes, purpose="outer", parent=self))
        # European-style markings
        elif LANE_MARKINGS_STYLE == EUR_STYLE:
            lw = 0.1 * STRIPE_WIDTH_SCALE_FACTOR
            # Draw centerline stripe if necessary
            if self.inverse_lane_index() == 0:
                leftEdge = self.alignment.parallel_offset(self.width/2, side="left")
                color, dashes = "0.2", (100, 0)
                markings.append(_LaneMarking(leftEdge, lw, color, dashes, purpose="center", parent=self))
            # Draw non-centerline markings
            else:
                adjacent_lane = self.parentEdge.get_lane(self.index + 1)
                leftEdge = self.alignment.parallel_offset(self.width / 2, side="left")
                color, dashes = "0.2", (3, 9)  # set default settings
                if self.allows("bicycle") != adjacent_lane.allows("bicycle"):
                    dashes = (100, 0)  # solid line where bicycles may not change lanes
                elif self.allows("passenger") != adjacent_lane.allows("passenger"):
                    if self.allows("bicycle"):
                        dashes = (1, 3)  # short dashed line where bikes may change lanes but passenger vehicles not
                    else:
                        dashes = (100, 0)  # solid line where neither passenger vehicles nor bikes may not change lanes
                markings.append(_LaneMarking(leftEdge, lw, color, dashes, purpose="lane", parent=self))
            # draw outer lane marking if necessary
            if self.index == 0 and not (self.allows("pedestrian") and not self.allows("all")):
                rightEdge = self.alignment.parallel_offset(self.width / 2, side="right")
                color, dashes = "0.2", (100, 0)
                markings.append(_LaneMarking(rightEdge, lw, color, dashes, purpose="outer", parent=self))
        # Stop line markings (all styles)
        slw = 0.5
        if PLOT_STOP_LINES and self.allows not in ["pedestrian", "ship"] and self._requires_stop_line():
            for stop_line_location in self.get_stop_line_locations():
                if not hasattr(ops, "substring"):
                    warnings.warn("Shapely >=1.7.0 required for drawing stop lines.", stacklevel=2)
                    break
                pos = self.alignment.length - stop_line_location - slw/2
                end_cl = ops.substring(self.alignment, pos-1, pos)
                if pos < 1:
                    warnings.warn("Unable to generate stop line for short lane " + self.id)
                    continue
                if not hasattr(end_cl, "parallel_offset"):
                    warnings.warn("Unable to generate stop line for short lane " + self.id)
                    continue
                if end_cl.is_empty or not end_cl.is_valid:
                    warnings.warn("Can't generate stopline geometry for lane " + self.id)
                    continue
                try:
                    end_left = end_cl.parallel_offset(self.width / 2, side="left")
                    end_right = end_cl.parallel_offset(self.width / 2, side="right")
                    stop_line = LineString([end_left.coords[-1], end_right.coords[0]])
                except (NotImplementedError, IndexError, ValueError):
                    warnings.warn("Can't generate stopline geometry for lane " + self.id)
                else:
                    if tl_phases is not None and self.id in tl_phases.get('green', []):
                        color = "lime"
                    else:
                        color = "lightcoral"
                    markings.append(_LaneMarking(stop_line, slw, color, (100, 0), purpose="stopline", parent=self))
        return markings

    def plot_lane_markings(self, ax, tl_phases=None, **kwargs):
        """
        Guesses and plots some simple lane markings.

        :param ax: matplotlib Axes object
        :return: list of artists
        :type ax: plt.Axes
        """
        artists = []
        for marking in self._guess_lane_markings(tl_phases=tl_phases):
            try:
                artist = marking.plot(ax, **kwargs)
            except NotImplementedError:
                warnings.warn("Can't plot center stripe for lane " + self.id, stacklevel=2)
            except ValueError:
                warnings.warn("Generated lane marking geometry is empty for lane " + self.id, stacklevel=2)
            else:
                artists.append(artist)
        return artists


class _Connection:
    def __init__(self, attrib):
        """
        Initialize a _Connection object.

        :param attrib: dict of all of the connection attributes
        :type attrib: dict
        """
        self.from_edge_id = attrib["from"]
        self.to_edge_id = attrib["to"]
        self.from_edge = None
        self.to_edge = None
        self.from_lane_index = int(attrib["fromLane"])
        self.to_lane_index = int(attrib["toLane"])
        self.from_lane = None
        self.to_lane = None
        self.via_id = attrib["via"] if "via" in attrib else None
        self.via_lane = None
        self.dir = attrib["dir"]
        self.state = attrib["state"]
        self.tl = attrib["tl"] if "tl" in attrib else None
        self.linkIndex = attrib["linkIndex"] if "linkIndex" in attrib else None

        if "shape" in attrib:
            coords = [[float(coord) for coord in xy.split(",")[0:2]] for xy in attrib["shape"].split(" ")]
            self.shape = LineString(coords)
        else:
            self.shape = None

    def _generate_shape(self):
        """
        Generate the shape of the lane in two dimensions based on the from_lane, via_lane, and to_lane.

        The alignment is taken from the via_lane, with the extruded points being adjusted to match the corners of the
        from_lane and to_lane. The width of the Connection is taken from the from_lane.

        :return: Polygon of the Connection shape
        """
        if self.from_lane is None or self.to_lane is None or self.via_lane is None:
            raise ReferenceError("Valid reference to from-, to-, and via-lanes required to generate connection shape.")
        # Get lane edges
        from_lane_left_edge = [list(c) for c in self.from_lane.alignment.parallel_offset(self.from_lane.width/2, side="left").coords]
        from_lane_right_edge = [list(c) for c in self.from_lane.alignment.parallel_offset(self.from_lane.width/2, side="right").coords]
        to_lane_left_edge = [list(c) for c in self.to_lane.alignment.parallel_offset(self.to_lane.width/2, side="left").coords]
        to_lane_right_edge = [list(c) for c in self.to_lane.alignment.parallel_offset(self.to_lane.width/2, side="right").coords]
        try:
            left_edge = [list(c) for c in self.via_lane.alignment.parallel_offset(self.from_lane.width/2, side="left").coords]
        except (ValueError, NotImplementedError):
            left_edge = []  # if offset fails, don't use any intermediate coordinates
        try:
            right_edge = [list(c) for c in self.via_lane.alignment.parallel_offset(self.from_lane.width/2, side="right").coords]
        except (ValueError, NotImplementedError):
            right_edge = []  # if offset fails, don't use any intermediate coordinates
        right_edge.reverse()
        # Generate coordinates
        left_coords = [from_lane_left_edge[-1]] + left_edge[1:-1] + [to_lane_left_edge[0]]
        right_coords = [from_lane_right_edge[0]] + right_edge[1:-1] + [to_lane_right_edge[-1]]
        left_coords.reverse()
        boundary_coords = right_coords + left_coords + [right_coords[0]]
        return Polygon(boundary_coords)

    def get_as_3d_object(self, z=0, include_bottom_face=False):
        """
        Generates an Object3D from the connection.

        :param z: z coordinate of connection
        :param include_bottom_face: whether to include the bottom face of the extruded geometry.
        :return: Object3D
        :type z: float
        :type include_bottom_face: bool
        """
        shape = self.shape if self.shape is not None else self._generate_shape()
        if shape.is_empty:
            return None
        h = 0.15 if self.from_lane.lane_type() == "pedestrian" and self.to_lane.lane_type() == "pedestrian" else 0
        material = "pedestrian_connection" if self.from_lane.lane_type() == "pedestrian" and self.to_lane.lane_type() == "pedestrian" else "connection"
        return Utils.Object3D.from_shape(orient(shape), "cxn_via_" + self.via_id, material, z=z, extrude_height=h, include_bottom_face=include_bottom_face)

    def plot_alignment(self, ax, **kwargs):
        """
        Plot the centerline of the connection.
        :param ax: matplotlib Axes object
        :return: artist
        :type ax: plt.Axes
        """
        if self.shape:
            x, y = zip(*self.shape.coords)
            line, = ax.plot(x, y, **kwargs)
            line.sumo_object = self
            return line


class _Request:
    def __init__(self, attrib, parent_junction=None):
        """
        Initializes a Request object

        :param attrib: dict of xml attributes
        :param parent_junction: parent Junction of this Request
        :type attrib: dict
        :type parent_junction: _Junction
        """
        self.index = int(attrib["index"])
        self.response = attrib["response"]
        self.foes = attrib["foes"]
        if "cont" in attrib:
            self.cont = attrib["cont"]
        else:
            self.cont = "0"
            warnings.warn("Request object missing attribute 'cont'. Defaulting to false.", stacklevel=2)
        self.parentJunction = parent_junction


class _Junction:
    def __init__(self, attrib):
        """
        Initializes a Junction object.

        :param attrib: dict of junction attributes.
        :type attrib: dict
        """
        self.id = attrib["id"]
        self.type = attrib["type"]
        self.incLane_ids = attrib["incLanes"].split(" ") if attrib["incLanes"] != "" else []
        self.intLane_ids = attrib["intLanes"].split(" ") if attrib["intLanes"] != "" else []
        self.incLanes = []
        self.intLanes = []
        self._requests = []
        self.params = dict()
        self.shape = None
        if "shape" in attrib:
            coords = [[float(coord) for coord in xy.split(",")[0:2]] for xy in attrib["shape"].split(" ")]
            if len(coords) > 2:
                self.shape = Polygon(coords)

    def append_request(self, request):
        """
        Add a Request object to the Junction.

        :param request: request to add to the junction
        :return: None
        :type request: _Request
        """
        request.parentJunction = self
        self._requests.append(request)

    def get_request_by_index(self, index):
        """
        Returns the Request with the given index.

        :param index: index of Request to get
        :return: Request with given index
        :type index: int
        """
        for req in self._requests:
            if req.index == index:
                return req
        raise IndexError("Junction " + self.id + " has no request with index " + str(index))

    def get_request_by_int_lane(self, lane_id):
        """
        Returns the Request corresponding to the internal lane with the specified id.

        :param lane_id: id of the internal lane for which to get the corresponding Request
        :return: Request corresponding to the specified internal lane
        """
        try:
            index = self.intLane_ids.index(lane_id)
        except ValueError as err:
            raise IndexError("Junction " + self.id + " does not include lane " + lane_id) from err
        else:
            return self.get_request_by_index(index)

    def get_as_3d_object(self, z=0, extrude_height=0, include_bottom_face=False, material_param=None,
                         extrude_height_param=None, extrude_height_param_transform=None):
        """
        Generates an Object3D from the junction.

        :param z: z coordinate of junction
        :param extrude_height: distance by which to extrude the junction
        :param include_bottom_face: whether to include the bottom face of the extruded geometry.
        :param material_param: generic parameter to use to override material, if present
        :param extrude_height_param: generic parameter to use to override extrude height, if present
        :param extrude_height_param_transform: function to apply to extrude_height_param values. Defaults to str->float conversion.
        :return: Object3D
        :type z: float
        :type extrude_height: float
        :type include_bottom_face: bool
        :type material_param: str
        :type extrude_height_param: str
        """
        if self.shape.is_empty:
            return None
        if extrude_height_param is not None and extrude_height_param in self.params:
            if extrude_height_param_transform is None:
                extrude_height_param_transform = lambda x: float(x) if x is not None else extrude_height
            extrude_height = extrude_height_param_transform(self.params[extrude_height_param])
        material = self.params.get(material_param, "junction")
        return Utils.Object3D.from_shape(orient(self.shape), self.id, "junction", z=z, extrude_height=extrude_height, include_bottom_face=include_bottom_face)

    def plot(self, ax, **kwargs):
        """
        Plots the Junction.

        :param ax: matplotlib Axes object
        :return: artist
        :type ax: plt.Axes
        """
        if self.shape is not None:
            if "lw" not in kwargs and "linewidth" not in kwargs:
                kwargs["lw"] = 0
            if "color" not in kwargs:
                kwargs["color"] = COLOR_SCHEME["junction"]
            poly = matplotlib.patches.Polygon(xy=self.shape.boundary.coords, closed=True, **kwargs)
            poly.sumo_object = self
            ax.add_patch(poly)
            return poly


class _TLPhase:
    def __init__(self, attrib):
        self.duration = int(attrib["duration"])
        self.state = attrib["state"]
        self.minDur = int(attrib["minDur"]) if "minDur" in attrib else None
        self.maxDur = int(attrib["maxDur"]) if "maxDur" in attrib else None
        self.name = attrib["name"] if "name" in attrib else None
        self.next = [int(x) for x in attrib["next"].split(" ")] if "next" in attrib else None


class _TLLogic:
    def __init__(self, attrib):
        self.id = attrib["id"]
        self.type = attrib["type"]
        self.programID = attrib["programID"]
        self.offset = int(attrib["offset"])
        self.phases = []  # type: list[_TLPhase]

    def add_phase(self, phase):
        self.phases.append(phase)


class Net:
    """
    :param file: path to Sumo network file
    :param additional_files: optional path to additional file (or list of paths) to include with the network.
    :type file: str
    :type additional_files: Union[str, list[str]]

    :ivar edges: dict with edge IDs as keys and _Edge objects as values
    :ivar junctions: dict with junction IDs as keys and _Junction objects as values
    :ivar connections: list of all _Connection objects
    :ivar netOffset: (x, y) tuple of the netOffset parameter from the net file
    :ivar projParameter: the projParameter from the net file
    """
    def __init__(self, file, additional_files=None):
        """
        Initializes a Net object from a Sumo network file

        :param file: path to Sumo network file
        :param additional_files: optional path to additional file (or list of paths) to include with the network.
        :type file: str
        :type additional_files: Union[str, list[str]]
        """
        self.additionals = []
        self.edges = dict()
        self.junctions = dict()
        self.tlLogics = dict()
        self.connections = []
        self.netOffset = (0, 0)
        self.projParameter = "!"
        net = ET.parse(file).getroot()
        for obj in net:
            if obj.tag == "location":
                if "netOffset" in obj.attrib:
                    self.netOffset = tuple(float(i) for i in obj.attrib["netOffset"].split(","))
                if "projParameter" in obj.attrib:
                    self.projParameter = obj.attrib["projParameter"]
            if obj.tag == "edge":
                if "function" in obj.attrib and obj.attrib["function"] == "walkingarea":
                    continue
                edge = _Edge(obj.attrib)
                for edgeChild in obj:
                    if edgeChild.tag == "stopOffset":
                        edge.append_stop_offset(edgeChild.attrib)
                    elif edgeChild.tag == "param":
                        edge.params[edgeChild.attrib["key"]] = edgeChild.attrib["value"]
                    elif edgeChild.tag == "lane":
                        lane = _Lane(edgeChild.attrib)
                        for laneChild in edgeChild:
                            if laneChild.tag == "stopOffset":
                                lane.append_stop_offset(laneChild.attrib)
                            elif laneChild.tag == "param":
                                lane.params[laneChild.attrib["key"]] = laneChild.attrib["value"]
                        edge.append_lane(lane)
                self.edges[edge.id] = edge
            elif obj.tag == "junction":
                junction = _Junction(obj.attrib)
                for jnChild in obj:
                    if jnChild.tag == "request":
                        req = _Request(jnChild.attrib)
                        junction.append_request(req)
                    elif jnChild.tag == "param":
                        junction.params[jnChild.attrib["key"]] = jnChild.attrib["value"]
                self.junctions[junction.id] = junction
            elif obj.tag == "connection":
                connection = _Connection(obj.attrib)
                self.connections.append(connection)
            elif obj.tag == "tlLogic":
                tlLogic = _TLLogic(obj.attrib)
                for tl_child in obj:
                    if tl_child.tag == "phase":
                        phase = _TLPhase(tl_child.attrib)
                        tlLogic.add_phase(phase)
                self.tlLogics[tlLogic.id] = tlLogic
        self._link_objects()
        if additional_files is not None:
            if type(additional_files) == str:
                self.load_additional_file(additional_files)
            else:
                for addl_file in additional_files:
                    self.load_additional_file(addl_file)

    def _link_objects(self):
        """
        Adds links between objects in the Network as necessary for certain functions.

        :return: None
        """
        # link junctions to edges
        for edge in self.edges.values():
            edge.from_junction = self.junctions.get(edge.from_junction_id, None)
            edge.to_junction = self.junctions.get(edge.to_junction_id, None)
        # link edges and lanes to connections
        for connection in self.connections:
            if connection.via_id is not None:
                connection.via_lane = self._get_lane(connection.via_id)
            connection.from_edge = self.edges.get(connection.from_edge_id, None)
            if connection.from_edge is not None:
                connection.from_lane = connection.from_edge.get_lane(connection.from_lane_index)
                connection.from_lane.outgoing_connections.append(connection)
            connection.to_edge = self.edges.get(connection.to_edge_id, None)
            if connection.to_edge is not None:
                connection.to_lane = connection.to_edge.get_lane(connection.to_lane_index)
                connection.to_lane.incoming_connections.append(connection)
        # make junction-related links
        for junction in self.junctions.values():
            if junction.type == "internal":
                continue
            # link incoming lanes to junction
            for i in junction.incLane_ids:
                incLane = self._get_lane(i)
                if incLane is not None:
                    junction.incLanes.append(incLane)
            # link internal lanes to junction
            for i in junction.intLane_ids:
                intLane = self._get_lane(i)
                if intLane is not None:
                    junction.intLanes.append(intLane)
            # link connections and requests to incoming lanes
            for lane in junction.incLanes:
                for cxn in lane.outgoing_connections:
                    if cxn.via_id is not None:
                        reqs = []
                        try:
                            req = junction.get_request_by_int_lane(cxn.via_id)
                        except IndexError:  # if no request found for via, look one level deeper
                            cxns_internal = self._get_connections_from_lane(cxn.via_id)
                            for cxni in cxns_internal:
                                req = junction.get_request_by_int_lane(cxni.via_id)
                                reqs.append(req)
                        else:
                            reqs.append(req)
                        for req in reqs:
                            lane.requests.append(req)

    def load_additional_file(self, file):
        """
        Load the specified additional file and associate it with this Net.

        :param file: path to additionals file
        :return: None
        """
        addl = _Addls(file, reference_net=self)
        self.additionals.append(addl)

    def _get_extents(self, apply_netOffset=False):
        lane_geoms = []
        for edge in self.edges.values():
            for lane in edge.lanes:
                lane_geoms.append(lane.shape)
        polygons = MultiPolygon(lane_geoms)
        xoff, yoff = self.netOffset if apply_netOffset else (0, 0)
        bounds = [sum(i) for i in zip(polygons.bounds, (-xoff, -yoff, -xoff, -yoff))]
        return bounds

    def _get_connections_from_lane(self, lane_id):
        cxns = []
        for connection in self.connections:
            if connection.from_edge_id + "_" + str(connection.from_lane_index) == lane_id:
                cxns.append(connection)
        return cxns

    def _get_connections_to_lane(self, lane_id):
        cxns = []
        for connection in self.connections:
            if connection.to_edge_id + "_" + str(connection.to_lane_index) == lane_id:
                cxns.append(connection)
        return cxns

    def _get_connections_via_lane(self, via):
        cxns = []
        for connection in self.connections:
            if connection.via_id == via:
                cxns.append(connection)
        return cxns

    def _get_lane(self, lane_id):
        edge_id = "_".join(lane_id.split("_")[:-1])
        lane_num = int(lane_id.split("_")[-1])
        edge = self.edges.get(edge_id, None)
        return edge.get_lane(lane_num) if edge is not None else None

    def _get_mask(self):
        """
        Returns a shape representing the area(s) covered by network lane and junction shapes.

        :return: result of shapely.ops.unary_union(polys), where polys contains all junction and lane shapes.
        """
        polys = []
        for junction in self.junctions.values():
            if junction.shape is not None:
                polys.append(junction.shape)
        for edge in self.edges.values():
            for lane in edge.lanes:
                if lane.shape is not None:
                    polys.append(lane.shape)
        mask = ops.unary_union(polys)
        return mask

    def generate_obj_text(self, style=None, stripe_width_scale=1, terrain_distance=0, terrain_z=0, terrain_hi_q=False,
                          material_mapping=None, material_param=None, extrude_height_param=None,
                          extrude_height_param_transform=None, apply_netOffset=False):
        """
        Generates the contents for a Wavefront-OBJ file which represents the network as a 3D model.

        This text can be saved as text to a file with the ``*.obj`` extension and then imported into a 3D software.
        The axis configuration in the generated file is Y-Forward, Z-Up.

        Sumo "generic parameters" can be used to override the extrude height and material of lanes, junctions, and polys.

        :param style: lane marking style to use for rendering ("USA" or "EUR"). Defaults to last used or "EUR".
        :param stripe_width_scale: scale factor for lane striping widths. Defaults to 1.
        :param terrain_distance: if > 0: distance from network to which to generate terrain plane.
        :param terrain_z: z value for terrain plane
        :param terrain_hi_q: if True, generates "high-quality" mesh for terrain (no interior angles > 20°). WARNING: this can be very computationally intensive for large or complex networks.
        :param material_mapping: a dictionary mapping terasim_vis-generated material names to user-defined ones
        :param material_param: generic parameter to use to override material, if present. material_mapping is applied also to this value.
        :param extrude_height_param: generic parameter to use to override extrude height, if present
        :param extrude_height_param_transform: function to apply to extrude_height_param values. Defaults to str->float conversion.
        :param apply_netOffset: if True, network will be shifted by the inverse of the netOffset
        :return: None
        :type style: str
        :type stripe_width_scale: float
        :type terrain_distance: float
        :type terrain_z: float
        :type terrain_hi_q: bool
        :type material_mapping: dict
        :type material_param: str
        :type extrude_height_param: str
        :type apply_netOffset: bool
        """
        if style is not None:
            set_style(style)
        set_stripe_width_scale(stripe_width_scale)
        objects = []
        for edge in self.edges.values():
            if edge.function == "internal":
                continue
            for lane in edge.lanes:
                if edge.function not in ["crossing", "walkingarea"]:
                    objects.append(lane.get_as_3d_object(material_param=material_param,
                                                         extrude_height_param=extrude_height_param,
                                                         extrude_height_param_transform=extrude_height_param_transform))
                objects += lane.get_markings_as_3d_objects()
        for junction in self.junctions.values():
            if junction.shape is not None:
                objects.append(junction.get_as_3d_object(material_param=material_param,
                                                         extrude_height_param=extrude_height_param,
                                                         extrude_height_param_transform=extrude_height_param_transform))
        for connection in self.connections:
            if connection.via_id is not None:
                if connection.from_lane.lane_type() == "pedestrian" and connection.to_lane.lane_type() == "pedestrian":
                    objects.append(connection.get_as_3d_object())
        for additional in self.additionals:
            for bus_stop in additional.bus_stops.values():
                objects += bus_stop.get_as_3d_objects()
            for poly in additional.polys.values():
                objects.append(poly.get_as_3d_object(material_param=material_param,
                                                     extrude_height_param=extrude_height_param,
                                                     extrude_height_param_transform=extrude_height_param_transform))
        while None in objects:
            objects.remove(None)
        if terrain_distance > 0:
            net_mask = self._get_mask()
            if OBJ_TERRAIN_CLEANUP_TOLERANCE > 0:
                net_mask = net_mask.buffer(OBJ_TERRAIN_CLEANUP_TOLERANCE).buffer(-OBJ_TERRAIN_CLEANUP_TOLERANCE)
            net_buffer = net_mask.buffer(terrain_distance, cap_style=2, join_style=2)
            terrain_shape = net_buffer.difference(net_mask)
            additional_opts = "q" if terrain_hi_q else ""
            objects.append(Utils.Object3D.from_shape_triangulated(terrain_shape, "terrain", "terrain", terrain_z,
                                                                   additional_opts=additional_opts))
        offset = (0, 0, 0) if apply_netOffset is False else (-self.netOffset[0], -self.netOffset[1], 0)
        return Utils.generate_obj_text_from_objects(objects, material_mapping=material_mapping, offset=offset)

    def plot(self, ax=None, clip_to_limits=False, zoom_to_extents=True, style=None, stripe_width_scale=1,
             plot_stop_lines=None, apply_netOffset=False, lane_kwargs=None, lane_marking_kwargs=None,
             junction_kwargs=None, additionals_kwargs=None, tl_phases=None, **kwargs):
        """
        Plots the Net. Kwargs are passed to the plotting functions, with object-specific kwargs overriding general ones.

        :param ax: matplotlib Axes object. Defaults to current axes.
        :param clip_to_limits: if True, only objects in the current view will be drawn. Speeds up saving of animations.
        :param zoom_to_extents: if True, window will be set to the network extents. Ignored if clip_to_limits is True
        :param style: lane marking style to use for plotting ("USA" or "EUR"). Defaults to last used or "EUR".
        :param stripe_width_scale: scale factor for lane striping widths
        :param plot_stop_lines: whether to plot stop lines
        :param apply_netOffset: whether to translate the network by the inverse of the netOffset value
        :param lane_kwargs: kwargs to pass to the lane plotting function (matplotlib.patches.Polygon())
        :param lane_marking_kwargs: kwargs to pass to the lane markings plotting function (matplotlib.lines.Line2D())
        :param junction_kwargs: kwargs to pass to the junction plotting function (matplotlib.patches.Polygon())
        :param additionals_kwargs: kwargs to pass to the additionals plotting function (Additionals.plot())
        :return: terasim_vis.ArtistCollection object containing all generated artists
        :type ax: plt.Axes
        :type clip_to_limits: bool
        :type zoom_to_extents: bool
        :type style: str
        :type stripe_width_scale: float
        :type plot_stop_lines: bool
        :type apply_netOffset: bool
        """
        if style is not None:
            set_style(style=style)
        if plot_stop_lines is not None:
            set_style(plot_stop_lines=plot_stop_lines)
        set_stripe_width_scale(stripe_width_scale)
        if ax is None:
            ax = plt.gca()
        xoff, yoff = self.netOffset
        if apply_netOffset:
            tr = transforms.Affine2D().translate(-xoff, -yoff) + ax.transData
            if "transform" in kwargs:
                tr += kwargs["transform"]
            kwargs["transform"] = tr
        if junction_kwargs is None:
            junction_kwargs = dict()
        if lane_kwargs is None:
            lane_kwargs = dict()
        if lane_marking_kwargs is None:
            lane_marking_kwargs = dict()
        if additionals_kwargs is None:
            additionals_kwargs = dict()
        if zoom_to_extents and not clip_to_limits:
            x_min, y_min, x_max, y_max = self._get_extents(apply_netOffset)
            ax.set_xlim(x_min, x_max)
            ax.set_ylim(y_min, y_max)
        ax.set_clip_box(ax.get_window_extent())
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        bounds = [[xmin, ymax], [xmax, ymax], [xmax, ymin], [xmin, ymin]]
        window = Polygon(bounds)
        artist_collection = Utils.ArtistCollection()
        for edge in self.edges.values():
            if edge.function != "internal" and (not clip_to_limits or edge.intersects(window)):
                la, lma = edge.plot(ax, {"zorder": -100, **lane_kwargs}, {"zorder": -90, **lane_marking_kwargs}, tl_phases=tl_phases, **kwargs)
                artist_collection.lanes += la
                artist_collection.lane_markings += lma
        for junction in self.junctions.values():
            if not clip_to_limits or (junction.shape is not None and junction.shape.intersects(window)):
                ja = junction.plot(ax, **{"zorder": -110, **kwargs, **junction_kwargs})
                artist_collection.junctions.append(ja)
        for additional in self.additionals:
            addls_ac = additional.plot(ax, **{**kwargs, **additionals_kwargs})
            artist_collection += addls_ac
        return artist_collection


if __name__ == "__main__":
    net = Net('../Sample/test.net.xml')
    fig, ax = plt.subplots()
    # ax.set_facecolor("green")
    net.plot(ax, style=USA_STYLE, stripe_width_scale=3)
    plt.show()
