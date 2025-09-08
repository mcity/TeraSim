from pathlib import Path
from matplotlib import pyplot as plt
import sumolib
import os
import googlemaps
from googlemaps.convert import decode_polyline
from dotenv import load_dotenv

load_dotenv()

sumo_map_path = Path("path/to/road/directory/map.net.xml")

# Load SUMO network file
net = sumolib.net.readNet(str(sumo_map_path))

# Get all edges
edges = net.getEdges()

def tomtom_snap_to_road(points):
    token = os.getenv("TOMTOM_API_KEY")
    if not token:
        raise ValueError("TOMTOM_API_KEY not found in .env file")
    
    import requests
    
    # Convert points to the format required by TomTom API
    # TomTom API expects points in (longitude,latitude) format
    points_str = ";".join([f"{lon},{lat}" for lat, lon in points])
    
    # Construct the API URL
    base_url = "https://api.tomtom.com/snapToRoads/1"
    params = {
        "key": token,
        "points": points_str,
        "vehicleType": "PassengerCar",
        "measurementSystem": "auto"
    }
    
    # Make the API request
    response = requests.get(base_url, params=params)
    
    if response.status_code != 200:
        raise Exception(f"TomTom API request failed with status code {response.status_code}: {response.text}")
    
    # Parse the response
    result = response.json()
    
    # Extract the snapped points
    snapped_points = []
    if "route" in result:
        for point in result["route"]:
            coords = point["geometry"]["coordinates"]
            # Convert back to (lat, lon) format to match Google Maps format
            coords_latlon = [(coord[1], coord[0]) for coord in coords]
            snapped_points.extend(coords_latlon)
    
    return snapped_points

def analyse_snapped_points(points, snapped_points, edge_id):
    # print("\nSnapped points from Google Maps:")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    # Plot on first axis
    ax1.plot([point[1] for point in snapped_points], [point[0] for point in snapped_points], 'ro-', label="snapped points")
    ax1.plot([point[1] for point in points], [point[0] for point in points], 'bo-', label="original points") 
    ax1.legend()
    ax1.set_title("Road Network")

    # Plot on second axis with satellite imagery
    ax2.plot([point[1] for point in snapped_points], [point[0] for point in snapped_points], 'ro-', label="snapped points")
    ax2.plot([point[1] for point in points], [point[0] for point in points], 'bo-', label="original points")
    ax2.legend()
    
    # Add satellite imagery basemap
    import contextily as cx
    cx.add_basemap(
        ax2,
        crs="EPSG:4326",
        source=cx.providers.Esri.WorldImagery,
    )
    ax2.set_title("Satellite View")

    plt.savefig(f"test_match_google_map_{edge_id}.png")
    plt.close()

def compare_snap_results(google_points, tomtom_points, original_points, edge_id):
    print("\nComparing Google Maps and TomTom Snap to Road results:")
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))
    
    # Plot on first axis
    ax1.plot([point[1] for point in google_points], [point[0] for point in google_points], 'ro-', label="Google snapped points")
    ax1.plot([point[1] for point in tomtom_points], [point[0] for point in tomtom_points], 'go-', label="TomTom snapped points")
    ax1.plot([point[1] for point in original_points], [point[0] for point in original_points], 'bo-', label="original points") 
    ax1.legend()
    ax1.set_title("Road Network Comparison")

    # Plot on second axis with satellite imagery
    ax2.plot([point[1] for point in google_points], [point[0] for point in google_points], 'ro-', label="Google snapped points")
    ax2.plot([point[1] for point in tomtom_points], [point[0] for point in tomtom_points], 'go-', label="TomTom snapped points")
    ax2.plot([point[1] for point in original_points], [point[0] for point in original_points], 'bo-', label="original points")
    ax2.legend()
    
    # Add satellite imagery basemap
    import contextily as cx
    cx.add_basemap(
        ax2,
        crs="EPSG:4326",
        source=cx.providers.Esri.WorldImagery,
    )
    ax2.set_title("Satellite View Comparison")

    plt.savefig(f"test_match_comparison_{edge_id}.png")
    plt.close()

# Print edge information
for edge in edges:
    print(f"\nEdge ID: {edge.getID()}")
    print(f"Edge length: {edge.getLength()}")
    print(f"Edge from: {edge.getFromNode().getID()}")
    print(f"Edge to: {edge.getToNode().getID()}")
    
    # Get shape in local coordinates
    shape = edge.getRawShape()
    print(f"Edge shape (local coordinates): {shape}")
    
    # Convert shape to lon/lat
    lon_lat_shape = [net.convertXY2LonLat(x, y) for x, y in shape]
    print(f"Edge shape (lon/lat): {lon_lat_shape}")
    
    # Convert coordinates to the format required by APIs
    # Both APIs expect points in (lat, lng) format
    points = [(lat, lon) for lon, lat in lon_lat_shape]
    
    # Use Google Maps Snap to Road API
    token = os.getenv("GOOGLE_MAPS_API_KEY")
    if not token:
        raise ValueError("GOOGLE_MAPS_API_KEY not found in .env file")
    
    gmaps = googlemaps.Client(key=token)
    
    # Call Google Maps Snap to Road API
    google_snapped = gmaps.snap_to_roads(points, interpolate=True)
    google_snapped_points = [(point['location']["latitude"], point['location']["longitude"]) for point in google_snapped]
    print(f"Google Maps snapped points: {google_snapped_points}")
    
    # Call TomTom Snap to Road API
    tomtom_snapped_points = tomtom_snap_to_road(points)
    print(f"TomTom snapped points: {tomtom_snapped_points}")
    
    # Compare results
    compare_snap_results(google_snapped_points, tomtom_snapped_points, points, edge.getID())
    
    # Convert back to local coordinates for analysis
    google_snapped_local = [net.convertLonLat2XY(point[1], point[0]) for point in google_snapped_points]
    tomtom_snapped_local = [net.convertLonLat2XY(point[1], point[0]) for point in tomtom_snapped_points]
    
    # Analyze individual results
    # analyse_snapped_points(points, google_snapped_points, f"{edge.getID()}_google")
    # analyse_snapped_points(points, tomtom_snapped_points, f"{edge.getID()}_tomtom")
    