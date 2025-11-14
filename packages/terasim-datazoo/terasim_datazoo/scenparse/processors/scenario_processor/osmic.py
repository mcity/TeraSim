from pathlib import Path
from typing import Union
import os

import xml.dom.minidom
import xml.etree.ElementTree as ET

from .osmizer import Osmizer


class Osmic(Osmizer):
    def __init__(self, scenario, osmize_config: dict = {}) -> None:

        super().__init__(scenario, **osmize_config)

    def save_OSM_netfile(
        self,
        base_dir: Union[Path, str, None] = None,
        filename: Union[str, None] = None,
    ):
        """
        Generating OSM .xml file
        """
        print("saving OSM file...")

        if not base_dir:
            base_dir = Path(f"map/{self.scenario_id}")
        base_dir = Path(base_dir)
        os.makedirs(base_dir, exist_ok=True)
        if not filename:
            filename = f"{self.scenario_id}-osm.xml"
        file_path = base_dir / filename

        osm_root = ET.Element("osm")
        osm_root.set("version", "0.6")
        osm_root.set("generator", "WaymoToOSM")
        for node in self.osm_nodes.values():
            node_element = ET.SubElement(osm_root, "node")
            node_element.set("id", str(node.id))
            node_element.set("visible", "true")
            node_element.set("version", "1")
            node_element.set("lat", str(node.lat))
            node_element.set("lon", str(node.lon))

        for way in self.osm_ways.values():
            way_element = ET.SubElement(osm_root, "way")
            way_element.set("id", str(way.id))
            way_element.set("visible", "true")
            way_element.set("version", "1")
            for point in way.node_refs:
                node_ref_element = ET.SubElement(way_element, "nd")
                node_ref_element.set("ref", str(point))

        osm_xml = xml.dom.minidom.parseString(ET.tostring(osm_root)).toprettyxml()
        with open(file_path, "w") as f:
            f.write(osm_xml)
        print(f"[File Output] Saved OSM netfile: {file_path} ...")