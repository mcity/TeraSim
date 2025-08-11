import requests
from typing import Dict, Optional, Tuple
import os
import dotenv
import sumolib
from pathlib import Path

dotenv.load_dotenv()

def get_traffic_flow_data(
    api_key: str,
    lat: float,
    lon: float,
    zoom: int = 10,
    style: str = "absolute",
    unit: str = "kmph"
) -> Dict:
    """
    Get traffic flow data around a specific latitude/longitude point.
    
    Args:
        api_key (str): TomTom API key
        lat (float): Latitude of the point
        lon (float): Longitude of the point
        zoom (int, optional): Zoom level (0-22). Defaults to 10.
        style (str, optional): Style of the flow data. Defaults to "absolute".
            Options: absolute, relative, relative0, relative0-dark, relative-delay, reduced-sensitivity
        unit (str, optional): Unit of speed. Defaults to "kmph". Options: kmph, mph
    
    Returns:
        Dict: Traffic flow data including:
            - currentSpeed: Current average speed
            - freeFlowSpeed: Expected speed under ideal conditions
            - currentTravelTime: Current travel time in seconds
            - freeFlowTravelTime: Expected travel time under ideal conditions
            - confidence: Quality measure of the data (0-1)
            - coordinates: List of coordinates describing the road segment
    """
    base_url = "https://api.tomtom.com/traffic/services/4/flowSegmentData"
    
    # Construct the URL with parameters
    url = f"{base_url}/{style}/{zoom}/json"
    
    params = {
        "key": api_key,
        "point": f"{lat},{lon}",
        "unit": unit
    }
    
    try:
        response = requests.get(url, params=params)
        response.raise_for_status()
        data = response.json()
        
        return data.get("flowSegmentData", {})
    except requests.exceptions.RequestException as e:
        print(f"Error fetching traffic data: {e}")
        return {}

import matplotlib.pyplot as plt

def visualize_tomtom_traffic_data(
    traffic_data_list,
    sumo_net: sumolib.net.Net
):
    fig, ax = plt.subplots()
    # Set equal aspect ratio to ensure x and y scales are the same
    ax.set_aspect('equal')
    for traffic_data in traffic_data_list:
        try:
            congestion_level = traffic_data.get("currentSpeed") / traffic_data.get("freeFlowSpeed")
            coordinates = traffic_data.get("coordinates")["coordinate"]
            coordinates_list = [
                (coordinate["longitude"], coordinate["latitude"])
                for coordinate in coordinates
            ]
            # Plot line with color based on congestion level (red=0, green=1)
            x_coords, y_coords = zip(*coordinates_list)
            ax.plot(x_coords, y_coords, color=(1-congestion_level, congestion_level, 0))
        except Exception as e:
            print(f"Error visualizing traffic data: {e}")
    plt.savefig("tomtom_traffic_data.png")

def get_traffic_flow_in_sumo_net(
    api_key: str,
    sumo_net: sumolib.net.Net,
    lat: float = None,
    lon: float = None,
    radius: int = None
) -> Dict:
    """
    Get traffic flow data around a point within specified radius.
    
    Args:
        api_key (str): TomTom API key
        lat (float): Latitude of the center point
        lon (float): Longitude of the center point
        radius (int, optional): Radius in meters to search around. Defaults to 500.
    
    Returns:
        Dict: Traffic flow data for the area
    """
    # for sumo lib, get all edges within 500m of the point
    if lat is None or lon is None:
        # Get the network boundary and calculate center
        xmin, ymin, xmax, ymax = sumo_net.getBoundary()
        x = (xmin + xmax) / 2
        y = (ymin + ymax) / 2
    else:
        x, y = sumo_net.convertLonLat2XY(lon, lat)

    if radius is None: # get all edges in the net
        edges = sumo_net.getEdges()
    else:
        edges = sumo_net.getNeighboringEdges(x, y, radius)

    # get all center of the edges
    edge_centers = []
    for edge in edges:
        edge_centers.append(edge.getShape()[len(edge.getShape())//2])

    traffic_data_list = []
    # get the traffic data for each edge
    for edge_center in edge_centers:
        lon_tmp, lat_tmp = sumo_net.convertXY2LonLat(edge_center[0], edge_center[1])
        traffic_data = get_traffic_flow_data(
            api_key=api_key,
            lat=lat_tmp,
            lon=lon_tmp
        )
        traffic_data_list.append(traffic_data)
    
    return traffic_data_list

def get_tomtom_traffic_incidents(
    api_key: str,
    bbox: str
) -> Dict:
    """
    Get traffic incidents around a specific latitude/longitude point.
    
    Args:
        api_key (str): TomTom API key
        lat (float): Latitude of the point
        lon (float): Longitude of the point
        radius (int, optional): Radius in meters to search around. Defaults to 500.
    
    Returns:
        Dict: Traffic incidents data including:
            - incidents: List of traffic incidents with details like:
                - type: Type of incident
                - geometry: Location information
                - properties: Additional incident properties
    """
    base_url = "https://api.tomtom.com/traffic/services/5/incidentDetails"
    
    # Calculate bounding box based on radius
    # Convert radius from meters to degrees (approximate)
    # bbox = f"{min_lon},{min_lat},{max_lon},{max_lat}"
    
    # Define fields to retrieve
    fields = "{incidents{type,geometry{type,coordinates},properties{iconCategory,events{description,code,iconCategory}}}}"
    
    params = {
        "key": api_key,
        "bbox": bbox,
        "fields": fields,
        "language": "en-GB",
        "timeValidityFilter": "present"
    }
    
    try:
        response = requests.get(base_url, params=params)
        response.raise_for_status()
        data = response.json()
        
        return data
    except requests.exceptions.RequestException as e:
        print(f"Error fetching traffic incidents: {e}")
        return {}

def get_bbox_from_center_and_radius(
    center_lon: float,
    center_lat: float,
    radius: int
) -> Tuple[float, float, float, float]:
    lat_degree = radius / 111000  # 1 degree of latitude is approximately 111km
    lon_degree = radius / (111000 * abs(center_lat))  # Adjust for longitude based on latitude

    # Create bounding box
    bbox = f"{center_lon-lon_degree},{center_lat-lat_degree},{center_lon+lon_degree},{center_lat+lat_degree}"
    
    return bbox

# Example usage:
if __name__ == "__main__":
    API_KEY = os.getenv("TOMTOM_API_KEY")
    sumo_net_path = Path("test_output_AA_500m_bbox/1/map.net.xml")
    sumo_net = sumolib.net.readNet(sumo_net_path)

    xmin, ymin, xmax, ymax = sumo_net.getBoundary()
    min_lon, min_lat = sumo_net.convertXY2LonLat(xmin, ymin)
    max_lon, max_lat = sumo_net.convertXY2LonLat(xmax, ymax)
    print(f"min_lon: {min_lon}, min_lat: {min_lat}, max_lon: {max_lon}, max_lat: {max_lat}")
    bbox = f"{min_lon},{min_lat},{max_lon},{max_lat}"

    # lon_center, lat_center = -83.85558504440473, 42.29721644274976
    lat_center, lon_center = 29.51382799260146, -98.15264106442983
    bbox = get_bbox_from_center_and_radius(
        center_lon=lon_center,
        center_lat=lat_center,
        radius=500
    )
    print(f"bbox: {bbox}")


    incidents = get_tomtom_traffic_incidents(
        api_key=API_KEY,
        bbox=bbox
    )
    print("aaa")
    
    # traffic_data_list = get_traffic_flow_in_sumo_net(
    #     api_key=API_KEY,
    #     sumo_net=sumo_net,
    # )
    # visualize_tomtom_traffic_data(
    #     traffic_data_list=traffic_data_list,
    #     sumo_net=sumo_net
    # )

    # for traffic_data in traffic_data_list:
    #     print(f"Current Speed: {traffic_data.get('currentSpeed')} km/h")
    #     print(f"Free Flow Speed: {traffic_data.get('freeFlowSpeed')} km/h")
    #     print(f"Current Travel Time: {traffic_data.get('currentTravelTime')} seconds")
    #     print(f"Confidence: {traffic_data.get('confidence')}")
