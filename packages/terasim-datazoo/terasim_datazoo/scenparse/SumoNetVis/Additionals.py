"""
Main classes and functions for dealing with Sumo "additional" files.
"""

import warnings
import xml.etree.ElementTree as ET
from shapely.geometry import *
from shapely import ops
import numpy as np
import matplotlib.patches
import matplotlib.pyplot as plt
from SumoNetVis import _Utils
from SumoNetVis import Net as _Net


BUS_STOP_STYLE_SUMO = "SUMO"
BUS_STOP_STYLE_GER = "GER"
BUS_STOP_STYLE_UK = "UK"
BUS_STOP_STYLE_USA = "USA"
BUS_STOP_STYLE = BUS_STOP_STYLE_SUMO

BUS_STOP_AREA_COLOR = {
    BUS_STOP_STYLE_SUMO: "#008853",
    BUS_STOP_STYLE_USA: "#C0422C"
}


def set_bus_stop_style(style):
    """
    Sets the bus stop plotting style. Valid values are 'SUMO', 'GER', 'UK', and 'USA'.

    :param style: desired bus stop plotting style
    :return: None
    """
    global BUS_STOP_STYLE
    if BUS_STOP_STYLE not in [BUS_STOP_STYLE_SUMO, BUS_STOP_STYLE_GER, BUS_STOP_STYLE_UK, BUS_STOP_STYLE_USA]:
        raise ValueError("Invalid bus stop style '" + style + "'.")
    BUS_STOP_STYLE = style


class _Poly:
    def __init__(self, attrib):
        """
        Initializes a Sumo additionals polygon.

        :param attrib: a dict of all the poly's XML attributes
        :type attrib: dict
        """
        self.id = attrib["id"]
        coords = [[float(coord) for coord in xy.split(",")] for xy in attrib["shape"].split(" ")]
        self.fill = attrib.get("fill", "f") in ["t", "true", "1"]
        self.shape = Polygon(coords) if self.fill else LineString(coords)
        self.color = _Utils.convert_sumo_color(attrib["color"])
        self.geo = attrib.get("geo", "f") in ["t", "true", "1"]
        if self.geo:
            warnings.warn("Geographic coordinates not supported for polygons in additional files.")
        self.lineWidth = float(attrib.get("lineWidth", 1))
        self.layer = float(attrib.get("layer", "0"))
        self.type = attrib.get("type", "")
        self.imgFile = attrib.get("imgFile", None)
        if self.imgFile is not None:
            warnings.warn("Display of polygons in additional files as images not supported.")
        self.angle = float(attrib.get("angle", 0))
        self.params = dict()

    def get_as_3d_object(self, z=0, extrude_height=0, include_bottom_face=False, material_param=None,
                         extrude_height_param=None, extrude_height_param_transform=None):
        """
        Generates a list of Object3D objects from the bus stop area and markings.

        :param z: desired z coordinate of base of object
        :param extrude_height: amount by which to extrude the polygon along the z axis.
        :param include_bottom_face: whether to include the bottom face when extruding.
        :param material_param: generic parameter to use to override material, if present
        :param extrude_height_param: generic parameter to use to override extrude height, if present
        :param extrude_height_param_transform: function to apply to extrude_height_param values. Defaults to str->float conversion.
        :return: Object3D representing the polygon
        :type z: float
        :type extrude_height: float
        :type include_bottom_face: bool
        :type material_param: str
        :type extrude_height_param: str
        """
        if extrude_height_param is not None and extrude_height_param in self.params:
            if extrude_height_param_transform is None:
                extrude_height_param_transform = lambda x: float(x) if x is not None else extrude_height
            extrude_height = extrude_height_param_transform(self.params[extrude_height_param])
        material = self.params.get(material_param, self.type+"_poly")
        return _Utils.Object3D.from_shape(self.shape, self.id, material, z=z, extrude_height=extrude_height,
                                          include_bottom_face=include_bottom_face, include_top_face=self.fill)

    def plot(self, ax, **kwargs):
        """
        Plot the polygon.

        :param ax: matplotlib Axes object
        :param kwargs: kwargs to pass to the plotting function
        :return: artist
        :type ax: plt.Axes
        """
        kwargs = {"zorder": self.layer-110 if self.layer <= 0 else self.layer-90, "color": self.color, **kwargs}
        if self.fill is True:
            if "lw" not in kwargs and "linewidth" not in kwargs:
                kwargs["lw"] = 0
            poly = matplotlib.patches.Polygon(self.shape.boundary.coords, True, **kwargs)
            poly.sumo_object = self
            ax.add_patch(poly)
            return poly
        else:
            x, y = zip(*self.shape.coords)
            line = _Utils.LineDataUnits(x, y, linewidth=self.lineWidth, **kwargs)
            line.sumo_object = self
            ax.add_line(line)
            return line


class _POI:
    def __init__(self, attrib, reference_net=None):
        """
        Initializes a Sumo additionals POI.

        :param attrib: a dict of all the POI's XML attributes
        :param reference_net: reference network to use for POIs whose position is specified based on a lane
        :type attrib: dict
        :type reference_net: SumoNetVis.Net
        """
        self.id = attrib["id"]
        self.color = _Utils.convert_sumo_color(attrib["color"])
        self.reference_net = reference_net
        self.x, self.y = None, None
        if "x" in attrib and "y" in attrib:
            self.x = float(attrib["x"])
            self.y = float(attrib["y"])
        elif "lane" in attrib and "pos" in attrib:
            if self.reference_net is None:
                warnings.warn("Reference Net required for POIs with locations defined relative to lane.")
            else:
                edge_id = "_".join(attrib["lane"].split("_")[:-1])
                lane_num = int(attrib["lane"].split("_")[-1])
                try:
                    lane = reference_net.edges.get(edge_id, None).get_lane(lane_num)
                except (AttributeError, IndexError) as err:
                    raise IndexError("Lane " + attrib["lane"] + " does not exist in reference network.") from err
                lane_pos = float(attrib["pos"])
                lane_pos_lat = float(attrib.get("posLat", 0))
                side = "right" if lane_pos_lat < 0 else "left"
                idx = 0 if side == "right" else -1  # shapely parallel_offset flips direction if side is "right"
                assert hasattr(ops, "substring"), "Shapely>=1.7.0 is required for POIs with lane-based locations."
                self.x, self.y = ops.substring(lane.alignment, 0, lane_pos).parallel_offset(abs(lane_pos_lat), side=side).coords[idx]
        elif "lat" in attrib and "lon" in attrib:
            warnings.warn("POI locations defined as lat/lon not supported.")
        else:
            raise ValueError("POI " + self.id + " has no valid position attributes.")
        self.type = attrib.get("type", "")
        self.layer = float(attrib.get("layer", "0"))
        self.imgFile = attrib.get("imgFile", None)
        if self.imgFile is not None:
            warnings.warn("Display of POIs in additional files as images not supported.")
        self.width = float(attrib.get("width", 0))
        self.height = float(attrib.get("height", 0))
        self.angle = float(attrib.get("angle", 0))
        self.params = dict()

    def plot(self, ax, **kwargs):
        """
        Plot the POI.

        :param ax: matplotlib Axes object
        :param kwargs: kwargs to pass to the plotting function
        :return: artist
        :type ax: plt.Axes
        """
        kwargs = {"color": self.color, "radius": 1,
                  "zorder": self.layer-110 if self.layer <= 0 else self.layer-90, **kwargs}
        if self.x is not None and self.y is not None:
            circle = matplotlib.patches.Circle((self.x, self.y), **kwargs)
            circle.sumo_object = self
            ax.add_patch(circle)
            return circle


class _BusStop:
    def __init__(self, attrib, reference_net):
        """
        Initialize a bus stop object.

        :param attrib: dict of all bus stop XML attributes
        :param reference_net: reference Sumo network object
        :type attrib: dict
        :type reference_net: SumoNetVis.Net
        """
        self.id = attrib["id"]
        self.lane_id = attrib["lane"]
        edge_id = "_".join(attrib["lane"].split("_")[:-1])
        lane_num = int(attrib["lane"].split("_")[-1])
        try:
            self.lane = reference_net.edges.get(edge_id, None).get_lane(lane_num)
        except (AttributeError, IndexError) as err:
            raise IndexError("Lane " + attrib["lane"] + " does not exist in reference network.") from err
        self.startPos = float(attrib.get("startPos", 0))
        self.endPos = float(attrib.get("endPos", self.lane.alignment.length))
        self.friendlyPos = attrib.get("friendlyPos", "f") in ["t", "true", "1"]
        self.name = attrib.get("name", "")
        self.lines = attrib.get("lines", "").split(" ")

    def _get_shape(self):
        """
        Return the outline shape of the bus stop based on the style setting. None if shape not to be drawn.

        :return: shapely geometry corresponding to bus stop outline
        """
        assert hasattr(ops, "substring"), "Shapely>=1.7.0 is required to plot bus stops."
        lane_cl_seg = ops.substring(self.lane.alignment, self.startPos, self.endPos)
        if BUS_STOP_STYLE == BUS_STOP_STYLE_SUMO:
            outline = lane_cl_seg.parallel_offset(self.lane.width/2, "right").buffer(1, cap_style=CAP_STYLE.flat)
        elif BUS_STOP_STYLE == BUS_STOP_STYLE_GER:
            outline = None
        elif BUS_STOP_STYLE == BUS_STOP_STYLE_UK:
            outline = None
        elif BUS_STOP_STYLE == BUS_STOP_STYLE_USA:
            outline = lane_cl_seg.buffer(self.lane.width/2, cap_style=CAP_STYLE.flat)
        return outline

    def _get_markings(self):
        """
        Returns a list of lane marking objects for the bus stop based on the style setting.

        :return: list of _LaneMarking objects
        """
        assert hasattr(ops, "substring"), "Shapely>=1.7.0 is required to plot bus stops."
        lane_cl_seg = ops.substring(self.lane.alignment, self.startPos, self.endPos)
        markings = []
        if BUS_STOP_STYLE == BUS_STOP_STYLE_SUMO:
            pass  # no markings for Sumo-style bus stops
        elif BUS_STOP_STYLE == BUS_STOP_STYLE_GER:
            # generate german zig-zag style markings
            lw, dashes = 0.12, (100, 0)
            area_width = 1.5
            curb_align = lane_cl_seg.parallel_offset(self.lane.width/2, "right")
            inner_align = curb_align.parallel_offset(area_width, "right")
            n_zags = round(lane_cl_seg.length / area_width / 2)
            if n_zags % 2 == 0:
                n_zags += 1
            zig_coords = [curb_align.interpolate(u).coords[0] for u in np.linspace(0, curb_align.length, n_zags)]
            zag_coords = [inner_align.interpolate(u).coords[0] for u in np.linspace(curb_align.length, 0, n_zags)]
            zigzag_coords = [curb_align.coords[0], inner_align.coords[-1]]
            for i in range(n_zags):
                if i % 2 == 0:
                    zigzag_coords.append(tuple(zag_coords[i]))
                else:
                    zigzag_coords.append(tuple(zig_coords[i]))
            zigzag_coords.append(curb_align.coords[-1])
            zigzag_line = LineString(zigzag_coords)
            markings.append(_Net._LaneMarking(zigzag_line, lw, "w", dashes, purpose="busstop", parent=self))
        elif BUS_STOP_STYLE == BUS_STOP_STYLE_UK:
            # generate UK-style inset dashed line box markings
            inset = 0.2
            heavy_lw, light_lw = 0.3, 0.1
            curb_dashes, end_dashes, inner_dashes = (100, 0), (1, 0.5), (0.75, 0.75)
            curb_align = lane_cl_seg.parallel_offset(self.lane.width/2-inset-heavy_lw/2, "right")
            inner_align = lane_cl_seg.parallel_offset(self.lane.width/2-inset-light_lw/2, "left")
            start_edge = LineString([curb_align.coords[0], inner_align.coords[-1]])
            end_edge = LineString([curb_align.coords[-1], inner_align.coords[0]])
            curb_align = ops.substring(curb_align, 2*light_lw, curb_align.length-2*light_lw).\
                parallel_offset(heavy_lw/2, "right")
            markings.append(_Net._LaneMarking(curb_align, heavy_lw, "y", curb_dashes, purpose="busstop", parent=self))
            markings.append(_Net._LaneMarking(start_edge, light_lw, "y", end_dashes, purpose="busstop", parent=self))
            markings.append(_Net._LaneMarking(end_edge, light_lw, "y", end_dashes, purpose="busstop", parent=self))
            markings.append(_Net._LaneMarking(inner_align, light_lw, "y", end_dashes, purpose="busstop", parent=self))
        elif BUS_STOP_STYLE == BUS_STOP_STYLE_USA:
            # return simple USA-style solid outline markings
            lw, dashes = 0.1, (100, 0)
            outline = lane_cl_seg.buffer(self.lane.width/2, cap_style=CAP_STYLE.flat).boundary
            markings.append(_Net._LaneMarking(outline, lw, "w", dashes, purpose="busstop", parent=self))
        return markings

    def plot(self, ax, area_kwargs=None, marking_kwargs=None, **kwargs):
        """
        Plot the bus stop.
        Kwargs are passed to the plotting functions, with object-specific kwargs overriding general ones.

        :param ax: matplotlib Axes object
        :param area_kwargs: kwargs to pass to area plotting function
        :param marking_kwargs: kwargs to pass to marking plotting function
        :return: list of artists
        :type ax: plt.Axes
        :type area_kwargs: dict
        :type marking_kwargs: dict
        """
        artists = []
        outline = self._get_shape()
        if area_kwargs is None:
            area_kwargs = dict()
        if marking_kwargs is None:
            marking_kwargs = dict()
        area_zorder = -80 if BUS_STOP_STYLE == BUS_STOP_STYLE_SUMO else -96
        area_kwargs = {"zorder": area_zorder, **kwargs, **area_kwargs}
        marking_kwargs = {"zorder": -94, **kwargs, **marking_kwargs}
        if outline is not None:
            area_color = BUS_STOP_AREA_COLOR.get(BUS_STOP_STYLE, "#00000000")
            area_kwargs = {"color": area_color, **area_kwargs}
            poly = matplotlib.patches.Polygon(outline.boundary.coords, True, **area_kwargs)
            poly.sumo_object = self
            ax.add_patch(poly)
            artists.append(poly)
        for marking in self._get_markings():
            artist = marking.plot(ax, **marking_kwargs)
            artists.append(artist)
        return artists

    def get_as_3d_objects(self, area_kwargs=None, markings_kwargs=None, **kwargs):
        """
        Generates a list of Object3D objects from the bus stop area and markings.

        Object-specific kwargs override general kwargs. Options are: "z", "extrude_height", and "include_bottom_face".
        Default for area_kwargs: 0.002, 0, False
        Default for markings_kwargs: 0.003, 0, False

        :param area_kwargs: kwargs for 3D area object generation.
        :param markings_kwargs: kwargs for 3D lane markings object generation.
        :type area_kwargs: dict
        :type markings_kwargs: dict
        """
        if area_kwargs is None:
            area_kwargs = dict()
        if markings_kwargs is None:
            markings_kwargs = dict()
        area_kwargs = {"z": 0.002, "extrude_height": 0, "include_bottom_face": False, **kwargs, **area_kwargs}
        markings_kwargs = {"z": 0.003, "extrude_height": 0, "include_bottom_face": False, **kwargs, **markings_kwargs}
        objs = []
        outline = self._get_shape()
        if outline is not None:
            objs.append(_Utils.Object3D.from_shape(outline, "busstop_area", "busstop_area", **area_kwargs))
        for marking in self._get_markings():
            objs.append(marking.get_as_3d_object(**markings_kwargs))
        return objs


class Additionals:
    """
    Stores objects from a Sumo additional XML file.

    :param file: path to Sumo additional file
    :param reference_net: network to use for objects which reference network elements (optional)
    :type file: str
    :type reference_net: SumoNetVis.Net
    """

    def __init__(self, file, reference_net=None):
        """
        Reads objects from a Sumo additional XML file.

        :param file: path to Sumo additional file
        :param reference_net: network to use for objects which reference network elements (optional)
        :type file: str
        :type reference_net: SumoNetVis.Net
        """
        self.reference_net = reference_net
        self.polys = dict()
        self.pois = dict()
        self.bus_stops = dict()
        root = ET.parse(file).getroot()
        for obj in root:
            if obj.tag == "poly":
                poly = _Poly(obj.attrib)
                self.polys[poly.id] = poly
                for polyChild in obj:
                    if polyChild.tag == "param":
                        poly.params[polyChild.attrib["key"]] = polyChild.attrib["value"]
            elif obj.tag == "poi":
                poi = _POI(obj.attrib, reference_net=reference_net)
                self.pois[poi.id] = poi
                for poiChild in obj:
                    if poiChild.tag == "param":
                        poi.params[poiChild.attrib["key"]] = poiChild.attrib["value"]
            elif obj.tag in ["busStop", "trainStop"]:
                bus_stop = _BusStop(obj.attrib, reference_net=reference_net)
                self.bus_stops[bus_stop.id] = bus_stop

    def plot_polygons(self, ax=None, **kwargs):
        """
        Plot all polygons.

        :param ax: matplotlib Axes object
        :param kwargs: kwargs to pass to the plotting function
        :return: list of artists
        :type ax: plt.Axes
        """
        artists = []
        if ax is None:
            ax = plt.gca()
        for poly in self.polys.values():
            artist = poly.plot(ax, **kwargs)
            artists.append(artist)
        return artists

    def plot_pois(self, ax=None, **kwargs):
        """
        Plot all POIs.

        :param ax: matplotlib Axes object
        :param kwargs: kwargs to pass to the plotting function
        :return: list of artists
        :type ax: plt.Axes
        """
        artists = []
        if ax is None:
            ax = plt.gca()
        for poi in self.pois.values():
            artist = poi.plot(ax, **kwargs)
            artists.append(artist)
        return artists

    def generate_bus_stops_obj_text(self, area_kwargs=None, markings_kwargs=None, **kwargs):
        """
        Generates the contents for a Wavefront-OBJ file which represents the bus stops as a 3D model.

        This text can be saved as text to a file with the ``*.obj`` extension and then imported into a 3D software.
        The axis configuration in the generated file is Y-Forward, Z-Up.

        Object-specific kwargs override general kwargs. Options are: "z", "extrude_height", and "include_bottom_face".

        :param area_kwargs: kwargs for 3D area object generation.
        :param markings_kwargs: kwargs for 3D lane markings object generation.
        :type area_kwargs: dict
        :type markings_kwargs: dict
        """
        objs = []
        for bus_stop in self.bus_stops.values():
            objs += bus_stop.get_as_3d_objects(area_kwargs, markings_kwargs, **kwargs)
        return _Utils.generate_obj_text_from_objects(objs)

    def generate_polygons_obj_text(self, **kwargs):
        """
        Generates the contents for a Wavefront-OBJ file which represents the polygons as a 3D model.

        This text can be saved as text to a file with the ``*.obj`` extension and then imported into a 3D software.
        The axis configuration in the generated file is Y-Forward, Z-Up.

        Possible kwargs are: "z", "extrude_height", and "include_bottom_face". Defaults are 0, 0, False.
        """
        objs = []
        kwargs = {"z": 0, "extrude_height": 0, "include_bottom_face": False, **kwargs}
        for poly in self.polys.values():
            objs.append(poly.get_as_3d_object(**kwargs))
        return _Utils.generate_obj_text_from_objects(objs)

    def plot_bus_stops(self, ax=None, area_kwargs=None, marking_kwargs=None, **kwargs):
        """
        Plots all bus stops.

        :param ax: matplotlib Axes object
        :param area_kwargs: kwargs to pass to the bus stop area plotting function
        :param marking_kwargs: kwargs to pass to the bus stop markings plotting function
        :return: list of artists
        """
        artists = []
        if ax is None:
            ax = plt.gca()
        for bus_stop in self.bus_stops.values():
            artist = bus_stop.plot(ax, area_kwargs, marking_kwargs, **kwargs)
            artists += artist
        return artists

    def plot(self, ax=None, polygon_kwargs=None, poi_kwargs=None, bus_stop_area_kwargs=None,
             bus_stop_marking_kwargs=None, **kwargs):
        """
        Plot all supported objects contained within the Additionals object.
        Kwargs are passed to the plotting functions, with object-specific kwargs overriding general ones.

        :param ax: matplotlib Axes object
        :param polygon_kwargs: kwargs to pass to the polygon plotting function
        :param poi_kwargs: kwargs to pass to the POI plotting function
        :param bus_stop_area_kwargs: kwargs to pass to the bus stop area plotting function
        :param bus_stop_marking_kwargs: kwargs to pass to the bus stop markings plotting function
        :return: SumoNetVis.ArtistCollection object containing all generated artists
        :type ax: plt.Axes
        :type polygon_kwargs: dict
        :type poi_kwargs: dict
        :type bus_stop_area_kwargs: dict
        :type bus_stop_marking_kwargs: dict
        """
        if ax is None:
            ax = plt.gca()
        if polygon_kwargs is None:
            polygon_kwargs = dict()
        if poi_kwargs is None:
            poi_kwargs = dict()
        artist_collection = _Utils.ArtistCollection()
        for poly in self.polys.values():
            artist = poly.plot(ax, **{**kwargs, **polygon_kwargs})
            artist_collection.polys.append(artist)
        for poi in self.pois.values():
            artist = poi.plot(ax, **{**kwargs, **poi_kwargs})
            artist_collection.pois.append(artist)
        for bus_stop in self.bus_stops.values():
            artists = bus_stop.plot(ax, area_kwargs=bus_stop_area_kwargs, marking_kwargs=bus_stop_marking_kwargs, **kwargs)
            artist_collection.bus_stops += artists
        return artist_collection


if __name__ == "__main__":
    net = _Net.Net("../Sample/test.net.xml")
    net.plot()
    set_bus_stop_style(BUS_STOP_STYLE_GER)
    addls = Additionals("../Sample/test.add.xml", reference_net=net)
    addls.plot()
    plt.gca().set_aspect("equal")
    plt.show()
