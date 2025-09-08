import json
import os
import sumolib
import yaml
from typing import Dict, List, Any, Optional
from anthropic import Anthropic


def create_llm_highway_scenario(sumo_net_path: str, metadata_json_path: str, output_file: str, scenario_description: str = None) -> bool:
    """Create a YAML file for highway scenarios using LLM-driven object placement.

    Args:
        sumo_net_path (str): The path to the SUMO net file.
        metadata_json_path (str): The path to the metadata JSON file containing the AV route.
        output_file (str): The path to the output YAML file.
        scenario_description (str): Natural language description of the desired scenario.
                                  If None, creates a default highway scenario.

    Returns:
        bool: True if scenario generation succeeded, False otherwise.
    """
    try:
        # Step 1: Extract semantic road information
        semantic_info, av_route_edges = extract_semantic_road_info(sumo_net_path, metadata_json_path)
        if not av_route_edges:
            print("Failed to extract route information from the map.")
            return False
        
        # Step 2: Use LLM to plan object placement
        if scenario_description is None:
            scenario_description = "Create a highway scenario with one stalled vehicle, one police car, and two pedestrians"
        
        placement_plan = llm_plan_object_placement(scenario_description, semantic_info)
        if not placement_plan:
            print("Failed to generate placement plan using LLM.")
            return False
        
        # Step 3: Generate YAML configuration
        yaml_config = generate_scenario_config(placement_plan, av_route_edges)
        
        # Step 4: Save configuration
        with open(output_file, 'w') as f:
            yaml.dump(yaml_config, f, default_flow_style=False, sort_keys=False)
        
        print(f"LLM-generated highway scenario created at {output_file}")
        print(f"Generated objects: {[obj['object_type'] for obj in placement_plan.get('placements', [])]}")
        return True
        
    except Exception as e:
        print(f"Error creating LLM highway scenario: {e}")
        return False


def extract_semantic_road_info(sumo_net_path: str, metadata_json_path: str) -> tuple[Dict[str, Any], List[Any]]:
    """Extract semantic road information for LLM understanding.
    
    Returns:
        tuple: (semantic_info_dict, av_route_edges_list)
    """
    # Read the SUMO net file
    sumo_net = sumolib.net.readNet(sumo_net_path)
    
    # Find the AV route based on the metadata JSON
    with open(metadata_json_path, 'r') as f:
        metadata_json = json.load(f)
        av_route_geo = metadata_json.get('av_route_sumo', None)

    if av_route_geo is None:
        return {}, []
    
    # Convert geo coordinates to SUMO edges
    av_route_edges = []
    radius = 0.1
    for point in av_route_geo:
        lat, lon = point
        x, y = sumo_net.convertLonLat2XY(lon, lat)
        edges = sumo_net.getNeighboringEdges(x, y, radius)
        if len(edges) > 0:
            distancesAndEdges = sorted([(dist, edge) for edge, dist in edges], key=lambda x: x[0])
            dist, closestEdge = distancesAndEdges[0]
            if closestEdge not in av_route_edges:
                av_route_edges.append(closestEdge)
    
    if not av_route_edges:
        return {}, []
    
    # Extract semantic information
    semantic_info = {
        "route_length_m": sum([edge.getLength() for edge in av_route_edges]),
        "road_segments": []
    }
    
    cumulative_position = 0
    for i, edge in enumerate(av_route_edges):
        lanes_info = []
        for idx, lane in enumerate(edge.getLanes()):
            lanes_info.append({
                "lane_id": lane.getID(),
                "lane_index": idx,
                "speed_limit_kmh": lane.getSpeed() * 3.6,  # Convert m/s to km/h
                "width_m": lane.getWidth(),
                "lane_type": classify_lane_type(lane, idx, len(edge.getLanes()))
            })
        
        segment_info = {
            "segment_id": f"seg_{i}",
            "edge_id": edge.getID(),
            "length_m": edge.getLength(),
            "position_start_m": cumulative_position,
            "position_end_m": cumulative_position + edge.getLength(),
            "lanes": lanes_info,
            "can_place_objects": len(lanes_info) > 1  # Can place objects if multiple lanes
        }
        
        semantic_info["road_segments"].append(segment_info)
        cumulative_position += edge.getLength()
    
    return semantic_info, av_route_edges


def classify_lane_type(lane, lane_index: int, total_lanes: int) -> str:
    """Classify lane type based on position and characteristics."""
    speed_kmh = lane.getSpeed() * 3.6
    
    # Simple heuristic based on lane position
    if lane_index == 0:
        return "left_lane"  # Leftmost lane
    elif lane_index == total_lanes - 1:
        if speed_kmh < 50:  # Low speed limit suggests emergency/shoulder
            return "emergency_lane"
        else:
            return "right_lane"  # Rightmost driving lane
    else:
        return "middle_lane"  # Middle lanes


def llm_plan_object_placement(scenario_description: str, semantic_info: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Use LLM to plan object placement based on scenario description and road info."""
    try:
        client = Anthropic(api_key=os.getenv('ANTHROPIC_API_KEY'))
        
        prompt = f"""As a traffic simulation expert, please arrange object positions for the following highway scenario:

Scenario description: {scenario_description}

Available road information:
- Total route length: {semantic_info['route_length_m']:.0f} meters
- Number of road segments: {len(semantic_info['road_segments'])}

Road segment details:
{format_road_segments_for_llm(semantic_info['road_segments'])}

Please consider the following factors:
1. Real-world logic (where do different objects typically appear?)
2. Testing effectiveness (how to maximize challenge for autonomous vehicles?)
3. Safety (ensure AV has reasonable reaction space and lane change opportunities)
4. Object spacing (avoid being too dense to prevent passage)

Supported object types:
- stalled_vehicle: Stalled vehicle (typically in right lane or emergency lane)
- police_car: Police car (can patrol or be stationary)
- pedestrian: Pedestrian (typically on shoulder or emergency areas)
- construction_zone: Construction zone (typically blocks some lanes)

Please return JSON format (only return JSON, no other content):
{{
    "placements": [
        {{
            "object_type": "stalled_vehicle",
            "object_id": "stalled_car_1",
            "segment_id": "seg_X",
            "lane_type": "right_lane",
            "position_percent": 0.3,
            "reasoning": "Placed in right lane at 30% position to give AV space for lane change"
        }}
    ]
}}"""
        
        response = client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=2000,
            messages=[{"role": "user", "content": prompt}]
        )
        
        # Parse LLM response
        response_text = response.content[0].text.strip()
        
        # Remove code block markers if present
        if response_text.startswith('```json'):
            response_text = response_text[7:-3].strip()
        elif response_text.startswith('```'):
            response_text = response_text[3:-3].strip()
        
        placement_plan = json.loads(response_text)
        return placement_plan
        
    except Exception as e:
        print(f"Error in LLM planning: {e}")
        return None


def format_road_segments_for_llm(segments: List[Dict]) -> str:
    """Format road segments information for LLM understanding."""
    formatted = []
    for seg in segments:
        lanes_desc = ", ".join([f"{lane['lane_type']}({lane['speed_limit_kmh']:.0f}km/h)" for lane in seg['lanes']])
        formatted.append(f"- {seg['segment_id']}: {seg['length_m']:.0f}m, lanes: [{lanes_desc}]")
    return "\n".join(formatted)


def generate_scenario_config(placement_plan: Dict[str, Any], av_route_edges: List[Any]) -> Dict[str, Any]:
    """Generate YAML configuration from LLM placement plan."""
    # Object templates
    object_templates = {
        "stalled_vehicle": {
            "_target_": "terasim_nde_nade.adversity.StalledObjectAdversity",
            "_convert_": "all",
            "object_type": "veh_passenger",
            "other_settings": {"duration": 600}
        },
        "police_car": {
            "_target_": "terasim_nde_nade.adversity.vehicles.SpecialVehicleAdversity",
            "_convert_": "all",
            "vehicle_type": "police",
            "behavior": "patrol"
        },
        "pedestrian": {
            "_target_": "terasim_nde_nade.adversity.vru.HighwayPedestrianAdversity",
            "_convert_": "all",
            "ego_type": "vulnerable_road_user",
            "location": "highway_shoulder"
        },
        "construction_zone": {
            "_target_": "terasim_nde_nade.adversity.ConstructionAdversity",
            "_convert_": "all"
        }
    }
    
    # Build configuration
    config = {"adversity_cfg": {"static": {}, "vehicle": {}, "vulnerable_road_user": {}}}
    
    for placement in placement_plan.get("placements", []):
        obj_type = placement["object_type"]
        obj_id = placement["object_id"]
        
        if obj_type not in object_templates:
            continue
        
        # Find corresponding lane and position
        lane_id, position = resolve_placement_to_sumo(placement, av_route_edges)
        if not lane_id:
            continue
        
        # Create object config
        obj_config = object_templates[obj_type].copy()
        obj_config["lane_id"] = lane_id
        
        if "position" in obj_config or obj_type in ["stalled_vehicle", "construction_zone"]:
            obj_config["lane_position"] = position
        
        # Categorize object
        if obj_type in ["stalled_vehicle", "construction_zone"]:
            config["adversity_cfg"]["static"][obj_id] = obj_config
        elif obj_type == "police_car":
            config["adversity_cfg"]["vehicle"][obj_id] = obj_config
        elif obj_type == "pedestrian":
            config["adversity_cfg"]["vulnerable_road_user"][obj_id] = obj_config
    
    # Clean up empty categories
    config["adversity_cfg"] = {k: v for k, v in config["adversity_cfg"].items() if v}
    
    return config


def resolve_placement_to_sumo(placement: Dict[str, Any], av_route_edges: List[Any]) -> tuple[Optional[str], float]:
    """Resolve LLM placement to actual SUMO lane ID and position."""
    try:
        segment_id = placement["segment_id"]
        lane_type = placement["lane_type"]
        position_percent = placement["position_percent"]
        
        # Find segment index
        seg_index = int(segment_id.split("_")[1])
        if seg_index >= len(av_route_edges):
            return None, 0.0
        
        edge = av_route_edges[seg_index]
        lanes = edge.getLanes()
        
        # Map lane type to actual lane index
        lane_index = 0
        if lane_type == "left_lane":
            lane_index = 0
        elif lane_type == "right_lane" or lane_type == "emergency_lane":
            lane_index = len(lanes) - 1
        elif lane_type == "middle_lane" and len(lanes) > 2:
            lane_index = len(lanes) // 2
        
        lane_id = lanes[lane_index].getID()
        position = edge.getLength() * position_percent
        
        return lane_id, position
        
    except Exception as e:
        print(f"Error resolving placement: {e}")
        return None, 0.0


# Backward compatibility: keep the old function name as an alias
def create_pull_over_case(sumo_net_path, metadata_json_path, output_file):
    """Legacy function - now uses LLM-driven approach."""
    return create_llm_highway_scenario(
        sumo_net_path, 
        metadata_json_path, 
        output_file,
        "Create a highway scenario with construction zone"
    )


if __name__ == "__main__":
    # Test with default scenario
    import dotenv
    dotenv.load_dotenv()
    
    sumo_net_path = "test_output_texas_1km_bbox_beginend/3/map.net.xml"
    metadata_json_path = "test_output_texas_1km_bbox_beginend/3/metadata.json"
    output_file = "llm_highway_scenario.yaml"
    
    # Test LLM-driven scenario generation
    success = create_llm_highway_scenario(
        sumo_net_path, 
        metadata_json_path, 
        output_file,
        "Create a highway scenario with one stalled vehicle, one patrol police car, and two pedestrians on the roadside"
    )
    
    if success:
        print("\nLLM-driven scenario generation completed successfully!")
    else:
        print("\nScenario generation failed.")