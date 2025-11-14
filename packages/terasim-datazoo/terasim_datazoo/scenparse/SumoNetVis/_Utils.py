"""
Contains miscellaneous utility classes and functions for internal library use.
"""

import warnings
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.colors
from shapely.geometry import Polygon as shapelyPolygon
try:
    from shapely.ops import polylabel
except ImportError:
    _POLYLABEL_IMPORTED = False
else:
    _POLYLABEL_IMPORTED = True
try:
    import triangle
except ImportError:
    _TRIANGLE_IMPORTED = False
else:
    _TRIANGLE_IMPORTED = True


class Object3D:
    def __init__(self, name, material, vertices, faces, lines=None):
        """
        Create a 3D object.

        :param name: name of the object
        :param material: name of the material to associate to the object
        :param vertices: list of vertex coordinates
        :param faces: list of faces, each a list of indices of the vertices making up the face
        :param lines: list of lines, each a list of indices of the vertices making up the line
        :type name: str
        :type material: str
        :type vertices: list[list[float]]
        :type faces: list[list[int]]
        :type lines: list[list[int]]
        """
        self.vertices = vertices
        self.faces = faces
        self.lines = lines if lines is not None else []
        self.name = name
        self.material = material

    @classmethod
    def from_shape(cls, shape, name, material, z=0, extrude_height=0, include_bottom_face=False, include_top_face=True):
        """
        Generates an Object3D from a shapely shape, either as a flat plane or by extrusion along the z axis

        :param shape: shapely Polygon or MultiPolygon from which to create 3D object
        :param name: name of the object
        :param material: name of the material to associate to the object
        :param z: the desired z coordinate for the base of the object. Defaults to zero.
        :param extrude_height: distance by which to extrude the face vertically.
        :param include_bottom_face: whether to include the bottom face of the extruded geometry.
        :param include_top_face: whether to include the top face of the geometry.
        :type name: str
        :type material: str
        :type z: float
        :type extrude_height: float
        :type include_bottom_face: bool
        :type include_top_face: bool
        """
        vertices, faces, lines = [], [], []
        # get coordinate sequences from shape
        if shape.geometryType() == "MultiPolygon":
            outlines = [polygon.boundary.coords for polygon in shape]
        elif shape.geometryType() == "Polygon":
            outlines = [shape.boundary.coords]
        elif shape.geometryType() == "MultiLineString":
            outlines = [line.coords for line in shape]
        elif shape.geometryType() == "LineString":
            outlines = [shape.coords]
        else:
            raise NotImplementedError("Can't generate 3D object from " + shape.geometryType())
        if shape.geometryType() in ["MultiLineString", "LineString"]:
            if include_top_face or include_bottom_face:
                warnings.warn("Ignoring 3D geometry top and bottom faces for geometry type "+shape.geometryType())
                include_bottom_face = include_top_face = False
        # calculate vertices and faces
        for outline in outlines:
            # generate coordinates of top and bottom face
            top_vertices = [[v[0], v[1], z+extrude_height] for v in outline]
            v_offset = len(vertices)
            edge_len = len(top_vertices)
            # add top vertices and face
            vertices += top_vertices
            if include_top_face:
                faces += [[i+1 for i in range(v_offset, v_offset+edge_len)]]
            elif extrude_height == 0 and not include_bottom_face:
                lines = [[i+1 for i in range(v_offset, v_offset+edge_len)]]
            # perform extrusion
            if extrude_height != 0:
                bottom_vertices = [[v[0], v[1], z] for v in outline]
                vertices += bottom_vertices
                # add side faces
                faces += [[i+edge_len+1, i+edge_len+2, i+2, i+1] for i in range(v_offset, v_offset+edge_len-1)]
                # add bottom face
                if include_bottom_face:
                    faces += [[i+edge_len+1 for i in reversed(range(v_offset, v_offset+edge_len))]]
        return cls(name, material, vertices, faces, lines)

    @classmethod
    def from_shape_triangulated(cls, shape, name, material, z=0, **kwargs):
        """
        Generate a planar triangulated object from a shapely shape. Supports multi-geometries and polygons with holes.

        :param shape: shapely shape from which to generate Object3D
        :param name: name of the object
        :param material: name of the material to associate to the object.
        :param z: the z coordinate for the object
        :param kwargs: kwargs to pass to triangulation function.
        :type name: str
        :type material: str
        :type z: float
        """
        vertices, faces = triangulate_polygon_constrained(shape, **kwargs)
        vertices = [[v[0], v[1], z] for v in vertices]
        return cls(name, material, vertices, faces)


def triangulate_polygon_constrained(shape, additional_opts=""):
    """
    Perform constrained polygon triangulation. Essentially a compatibility layer between shapely and triangle.

    For more information about the triangulation, see documentation for triangle: https://rufat.be/triangle/API.html

    :param shape: shapely shape to triangulate
    :param additional_opts: additional options to  pass to triangle.triangulate(). "p" option is always used.
    :type additional_opts: str
    :return: vertices, faces; where vertices is a list of coordinates, and faces a list of 1-indexed face definitions.
    """
    if not _TRIANGLE_IMPORTED:
        raise EnvironmentError("Library 'triangle' required for triangulation.")
    if not _POLYLABEL_IMPORTED:
        raise EnvironmentError("Constrained polygon triangulation requires shapely>=1.7.0")
    # Polygon case
    if shape.geometryType() == "Polygon":
        tri = {"vertices": [], "segments": [], "holes": []}
        # add exterior edge
        ext_pt_cnt = len(shape.exterior.coords) - 1
        tri["vertices"] += shape.exterior.coords[:-1]
        tri["segments"] += [[i, i+1] for i in range(ext_pt_cnt-1)] + [[ext_pt_cnt-1, 0]]
        # add interior edges and holes
        offset = ext_pt_cnt
        for hole in shape.interiors:
            hole_pt_cnt = len(hole.coords) - 1
            tri["vertices"] += list(hole.coords[:-1])
            tri["segments"] += [[i, i+1] for i in range(offset, offset+hole_pt_cnt-1)] + [[offset+hole_pt_cnt-1, offset]]
            rp = shapelyPolygon(hole.coords).representative_point()
            tri["holes"].append(list(*rp.coords))
            offset += hole_pt_cnt
        if len(tri["holes"]) == 0:
            tri.pop("holes")
        # perform triangulation
        t = triangle.triangulate(tri, "p"+additional_opts)
        vertices = t["vertices"]
        faces = [[i+1 for i in j] for j in t["triangles"]]  # switch from 0- to 1-indexing
        return vertices, faces
    # MultiPolygon/GeometryCollection case (recursive)
    elif shape.geometryType() in ["MultiPolygon", "GeometryCollection"]:
        vertices, faces = [], []
        for part in shape:
            if part.geometryType() == "Polygon":
                v, f = triangulate_polygon_constrained(part, additional_opts)
                offset = len(vertices)
                vertices += [list(i) for i in v]
                faces += [[i+offset for i in j] for j in f]
        return vertices, faces
    # Unsupported geometry case
    else:
        raise NotImplementedError("Can't do constrained triangulation on geometry type " + shape.geometryType())


def generate_obj_text_from_objects(objects, material_mapping=None):
    """
    Generate Wavefront OBJ text from a list of Object3D objects

    :param objects: list of Object3D objects
    :param material_mapping: a dictionary mapping SumoNetVis-generated material names to user-defined ones
    :return: Wavefront OBJ text
    :type objects: list[Object3D]
    :type material_mapping: dict
    """
    if material_mapping is None:
        material_mapping = dict()
    content = ""
    vertex_count = 0
    for object in objects:
        content += "o " + object.name
        content += "\nusemtl " + material_mapping.get(object.material, object.material)
        content += "\nv " + "\nv ".join([" ".join([str(c) for c in vertex]) for vertex in object.vertices])
        if len(object.faces) > 0:
            content += "\nf " + "\nf ".join([" ".join([str(v + vertex_count) for v in face]) for face in object.faces])
        if len(object.lines) > 0:
            content += "\nl " + "\nl ".join([" ".join([str(v + vertex_count) for v in line]) for line in object.lines])
        content += "\n\n"
        vertex_count += len(object.vertices)
    return content


class Allowance:
    """
    A class for handling vehicle class lane allowances.
    """
    vClass_list = np.array(
        ["private", "emergency", "authority", "army", "vip", "pedestrian", "passenger", "hov", "taxi",
         "bus", "coach", "delivery", "truck", "trailer", "motorcycle", "moped", "bicycle", "evehicle",
         "tram", "rail_urban", "rail", "rail_electric", "rail_fast", "ship", "custom1", "custom2"])

    def __init__(self, allow_string="all", disallow_string=""):
        """
        Initializes an Allowance object.

        :param allow_string: string containing space-separated list of allowed vehicle classes
        :param disallow_string: string containing space-separated list of disallowed vehicle classes
        :type allow_string: str
        :type disallow_string: str
        """
        if allow_string in ["all", ""]:
            allows = self.vClass_list
        elif allow_string == "none":
            allows = np.array([])
        else:
            allows = np.array(allow_string.split(" "))
        if disallow_string == "all":
            disallows = self.vClass_list
        elif disallow_string == "":
            disallows = np.full(len(self.vClass_list), False)
        else:
            disallows = np.array(disallow_string.split(" "))
        allow_mask = np.isin(self.vClass_list, allows)
        disallow_mask = np.isin(self.vClass_list, disallows)
        self.mask = np.logical_and(allow_mask, ~disallow_mask)

    def allows(self, vClass):
        """
        Checks if the specified vClass is allowed by this Allowance.

        :param vClass: vClass to check
        :return: True if vClass allowed, else False
        :type vClass: str
        """
        if vClass == "all":
            return self.mask.all()
        if vClass == "none":
            return np.logical_not(self.mask).all()
        if vClass not in self.vClass_list:
            raise IndexError("Invalid vehicle class " + str(vClass))
        return self.mask[np.where(self.vClass_list == vClass)[0]].all()

    def get_allow_string(self):
        """Return the 'allow' string as it would appear in a Net file."""
        if self.mask.all():
            return "all"
        elif not self.mask.any():
            return "none"
        else:
            return " ".join(self.vClass_list[self.mask])

    def get_disallow_string(self):
        """Return the 'disallow' string as it would appear in a Net file."""
        if self.mask.all():
            return "none"
        elif not self.mask.any():
            return "all"
        else:
            return " ".join(self.vClass_list[~self.mask])

    def is_superset_of(self, other):
        return self.mask[other.mask].all()

    def __getitem__(self, vClass):
        return self.allows(vClass)

    def __call__(self, *vClasses, operation="all"):
        if operation not in ["all", "any"]:
            raise ValueError("Invalid operation " + str(operation))
        op = np.all if operation == "all" else np.any
        return op([self.allows(vClass) for vClass in vClasses])

    def __invert__(self):
        inverted = Allowance()
        inverted.mask = ~self.mask
        return inverted

    def __eq__(self, other):
        if type(other) == str:
            other = type(self)(other)
        if type(other) == type(self):
            return np.array_equal(self.mask, other.mask)
        else:
            raise NotImplementedError("Can't compare " + str(type(self)) + " to " + str(type(other)))

    def __add__(self, other):
        if type(other) == str:
            other = type(self)(other)
        result = Allowance()
        result.mask = np.logical_or(self.mask, other.mask)
        return result

    def __repr__(self):
        return "Allowance('" + self.get_allow_string() + "')"


class LineDataUnits(Line2D):
    """
    A Line2D object, but with the linewidth and dash properties defined in data coordinates.
    """
    def __init__(self, *args, **kwargs):
        _lw_data = kwargs.pop("linewidth", 1)
        _dashes_data = kwargs.pop("dashes", (1, 0))
        super().__init__(*args, **kwargs)
        if _dashes_data != (1, 0):
            self.set_linestyle("--")
        self._lw_data = _lw_data
        self._dashes_data = _dashes_data
        self._dashOffset = 0

    def _get_lw(self):
        if self.axes is not None:
            ppd = 72./self.axes.figure.dpi
            trans = self.axes.transData.transform
            return ((trans((1, self._lw_data))-trans((0, 0)))*ppd)[1]
        else:
            return 1

    def _set_lw(self, lw):
        self._lw_data = lw

    def _get_dashes(self):
        if self.axes is not None:
            ppd = 72./self.axes.figure.dpi
            trans = self.axes.transData.transform
            dpu = (trans((1, 1)) - trans((0, 0)))[0]
            return tuple([u*dpu*ppd for u in self._dashes_data])
        else:
            return tuple((1, 0))

    def _set_dashes(self, dashes):
        self._dashes_data = dashes

    _linewidth = property(_get_lw, _set_lw)
    _dashSeq = property(_get_dashes, _set_dashes)


def convert_sumo_color(sumo_color):
    """
    Convert a Sumo-compatible color string to a matplotlib-compatible format.

    :param sumo_color: the color as specified in the sumo file
    :return: a matplotlib-compatible representation of the color
    :type sumo_color: str
    """
    if matplotlib.colors.is_color_like(sumo_color):
        return sumo_color
    elif "," in sumo_color:
        c = tuple([float(i) for i in sumo_color.split(",")])
        if max(c) > 1:
            c = tuple([ci/255 for ci in c])
        if len(c) not in [3, 4] or max(c) > 1:
            raise ValueError("Invalid color tuple '" + sumo_color + "'.")
        return c
    else:
        raise ValueError("Invalid color '" + sumo_color + "'.")


class NonelessList(list):
    """A special list-like object that will ignore any Nones added to it."""
    def append(self, item):
        if item is not None:
            super().append(item)

    def __iadd__(self, other):
        for item in other:
            self.append(item)
        return self

    def __add__(self, other):
        result = type(self)(self)
        result += other
        return result


class ArtistCollection:
    """
    Collection of Artist objects generated by various SumoNetVis plotting functions.

    Consists of multiple lists of artists broken down by the type of Sumo object that generated them.
    The object can also be iterated and indexed just like a list. For example:

        for artist in artist_collection:
            artist.do_something()

    :ivar lanes: list of artists created by lanes
    :ivar lane_markings: list of artists created by lane markings
    :ivar junctions: list of artists created by junctions
    :ivar polys: list of artists created by additionals polygons
    :ivar pois: list of artists created by additionals POIs
    :ivar bus_stops: list of artists created by bus stops
    """
    def __init__(self):
        self.lanes = NonelessList()
        self.lane_markings = NonelessList()
        self.junctions = NonelessList()
        self.polys = NonelessList()
        self.pois = NonelessList()
        self.bus_stops = NonelessList()

    def _as_list(self):
        return self.lanes + self.lane_markings + self.junctions + self.polys + self.pois + self.bus_stops

    def __iter__(self):
        return iter(self._as_list())

    def __getitem__(self, item):
        return self._as_list()[item]

    def __iadd__(self, other):
        self.lanes += other.lanes
        self.lane_markings += other.lane_markings
        self.junctions += other.junctions
        self.polys += other.polys
        self.pois += other.pois
        self.bus_stops += other.bus_stops
        return self
