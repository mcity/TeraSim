from pathlib import Path
from crdesigner.map_conversion.sumo_map.config import SumoConfig
from crdesigner.map_conversion.sumo_map.cr2sumo.converter import CR2SumoMapConverter
from crdesigner.map_conversion.sumo_map.sumo2cr import convert_net_to_cr
import os
from lxml import etree
from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet

def test_opendrive_to_lanelet():
    # Test conversion from OpenDRIVE to Lanelet2
    xodr_path = "test_output/Ann_Arbor_Michigan_USA_highway_05b42f4d/map.xodr"
    ll_path = "test_output/Ann_Arbor_Michigan_USA_highway_05b42f4d/map.lanelet2.osm"
    opendrive_to_lanelet(xodr_path, ll_path)

    # Check if the file was created
    assert os.path.exists(ll_path)
    assert os.path.getsize(ll_path) > 0
    assert os.path.exists(ll_path)


# def test_sumo_to_lanelet():
#     # Test conversion from SUMO to Lanelet2
#     sumo_path = Path("test_output/Ann_Arbor_Michigan_USA_highway_05b42f4d/map.net.xml")
#     ll_path = Path("test_output/Ann_Arbor_Michigan_USA_highway_05b42f4d/map_from_sumo.lanelet2.osm")
#     scenario = convert_net_to_cr(str(sumo_path))
#     l2osm = CR2SumoMapConverter(config=SumoConfig())
#     osm = l2osm(scenario)
#     with open(f"{ll_path}", "wb") as file_out:
#         file_out.write(
#             etree.tostring(osm, xml_declaration=True, encoding="UTF-8", pretty_print=True)
#         ) 
#     # Check if the file was created
#     assert os.path.exists(ll_path)
#     assert os.path.getsize(ll_path) > 0

if __name__ == "__main__":
    test_opendrive_to_lanelet()
    # test_sumo_to_lanelet()