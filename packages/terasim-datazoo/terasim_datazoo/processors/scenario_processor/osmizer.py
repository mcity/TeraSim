from terasim_datazoo.core.osm import *
from terasim_datazoo.core.waymo import LaneType


class Osmizer:
    """
    # Waymo -> OSM

    A Waymo to OSM map converter. The conversion is naive and incomplete, and the converter is mainly for Waymo map visualization purpose.

    ### Usage

    1. Instantiate a class object ``converter = Waymo2OSM(waymo_data)``,
    where waymo_data is a raw map scenario object of type ``scenario_pb2.Scenario()`` extracted from Waymo open dataset.
    You may use ``utils.read_data(file_path, scenario_id_set)`` to extract the scenario first.
    Parsing will be done right after the initialization.

    2. ``converter.save_OSM_file(base_dir)``: output the converted OSM .xml file, with ``base_dir`` as the directory. The resulted OSM file has each lane as a separated edge, and edges are unconnected.

    3. ``converter.plot_OSM_map(base_dir, traffic_light, stop_sign)``: plot a visualiation image to look at the Waymo data. ``base_dir`` is the directory.
       - if ``traffic_light` is set to true, all traffic light head positions in Waymo data are plotted in the map as blue dots.
       - if ``stop_sign`` is set to true, all stop signs in Waymo data are plotted in the map as green dots.
    """

    def __init__(self, scenario, base_latitude=40, base_longitude=-73) -> None:

        self._BASE_LATITUDE = base_latitude
        self._BASE_LONGITUDE = base_longitude

        self.osm_nodes: dict[int, OSMNode] = {}
        """A dict of geenrated OSM nodes, keyed with node id"""
        self.osm_ways: dict[int, OSMWay] = {}
        """A dict of generated OSM ways, keyed with way id"""

        self._node_id_counter: int = 0

        # parse map features
        for feature in scenario.map_features:
            feature_data_type = feature.WhichOneof("feature_data")
            if feature_data_type == "lane":
                self._translate_lane(feature)

    def _translate_lane(self, feature) -> None:

        lane = feature.lane
        if lane.type == LaneType.UNDEFINED:
            return

        def _create_node(x: float, y: float):
            self.osm_nodes[self._node_id_counter] = OSMNode(
                self._node_id_counter,
                x,
                y,
                base_latitude=self._BASE_LATITUDE,
                base_longitude=self._BASE_LONGITUDE,
            )
            self._node_id_counter += 1
            return self._node_id_counter - 1

        node_id_list = [_create_node(point.x, point.y) for point in lane.polyline]
        self.osm_ways[int(feature.id)] = OSMWay(feature.id, node_id_list)

        if lane.type == 1:  # TYPE_FREEWAY
            self.osm_ways[int(feature.id)].tags["highway"] = "trunk"
        elif lane.type == 2:  # TYPE_SURFACE_STREET
            self.osm_ways[int(feature.id)].tags["highway"] = "secondary"
        elif lane.type == 3:
            self.osm_ways[int(feature.id)].tags["highway"] = "cycleway"

        self.osm_ways[int(feature.id)].tags["lanes"] = "1"
