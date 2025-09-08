import json
import random
import sumolib
import yaml


def create_constructionzone_yaml(sumo_net_path, metadata_json_path, output_file):
    """Create a YAML file for construction zones based on the SUMO net file.

    Args:
        sumo_net_path (str): The path to the SUMO net file.
        metadata_json_path (str): The path to the metadata JSON file containing the AV route.
        output_file (str): The path to the output YAML file.
    """    
    # read the SUMO net file
    sumo_net = sumolib.net.readNet(sumo_net_path)
    
    # find the av route based on the metadata JSON
    av_route_geo = None
    with open(metadata_json_path, 'r') as f:
        metadata_json = json.load(f)
        av_route_geo = metadata_json.get('av_route_sumo', None)

    if av_route_geo is None:
        print("No av_route_sumo found in metadata!")
        return
    
    av_route = []
    radius = 0.1
    for point in av_route_geo:
        lat, lon = point
        x, y = sumo_net.convertLonLat2XY(lon, lat)
        edges = sumo_net.getNeighboringEdges(x, y, radius)
        # pick the closest edge
        if len(edges) > 0:
            distancesAndEdges = sorted([(dist, edge) for edge, dist in edges], key=lambda x:x[0])
            dist, closestEdge = distancesAndEdges[0]
            if closestEdge not in av_route:
                av_route.append(closestEdge)
    if not av_route:
        print("No edges found for the AV route in the SUMO network.")
        return

    # randomly select one edge with multiple lanes for construction
    # and randomly select the leftmost lane or the rightmost lane for construction
    construction_lane = None
    possible_construction_edges = []
    for edge in av_route:
        lanes = edge.getLanes()
        if lanes and len(lanes) > 1:
            possible_construction_edges.append(edge)
    if not possible_construction_edges:
        print("No possible construction edges (more than one lane) found in the AV route.")
        return
    # randomly select one edge for construction
    construction_edge = random.choice(possible_construction_edges)
    # randomly select the leftmost lane or the rightmost lane for construction
    lanes = construction_edge.getLanes()
    if random.choice([True, False]):
        construction_lane = construction_edge.getID() + "_0"
    else:
        construction_lane = construction_edge.getID() + "_" + str(len(lanes) - 1)
    print(f"Selected construction lane: {construction_lane}")

    # create the YAML structure
    construction_zone_yaml = {
        'adversity_cfg': {
            'static': {
                'construction': {
                    '_target_': 'terasim_nde_nade.adversity.ConstructionAdversity',
                    '_convert_': 'all',
                    'lane_id': construction_lane
                }
            }
        }
    }

    with open(output_file, 'w') as f:
        yaml.dump(construction_zone_yaml, f)

    print(f"Construction zones YAML file created at {output_file}")

if __name__ == "__main__":
    sumo_net_path = "test_output/task_1748282877046_ou8rd4upc/10/map.net.xml"
    metadata_json_path = "test_output/task_1748282877046_ou8rd4upc/10/metadata.json"
    output_file = "construction_zone_tmp.yaml"
    
    create_constructionzone_yaml(sumo_net_path, metadata_json_path, output_file)