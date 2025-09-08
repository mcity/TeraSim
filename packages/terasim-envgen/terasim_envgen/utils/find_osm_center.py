import os
import osmnx as ox
import numpy as np
import argparse
import networkx as nx
from pathlib import Path
import xml.etree.ElementTree as ET

def find_osm_center(osm_file_path):
    """
    Find the center point of an OSM file using OSMnx
    
    Args:
        osm_file_path: Path to the OSM file
        
    Returns:
        tuple: (latitude, longitude) of the center point
    """
    # Load the OSM file
    print(f"Loading OSM file from: {osm_file_path}")
    try:
        # First attempt to load directly as an XML file
        G = ox.graph_from_xml(osm_file_path)
        print("Successfully loaded OSM file using graph_from_xml")
    except Exception as e:
        print(f"Error loading as XML: {e}")
        print("Trying alternative approach...")
        # If direct loading failed, try parsing the XML manually
        try:
            tree = ET.parse(osm_file_path)
            root = tree.getroot()
            
            # Extract bounds if available
            bounds = root.find('bounds')
            if bounds is not None:
                minlat = float(bounds.attrib['minlat'])
                minlon = float(bounds.attrib['minlon'])
                maxlat = float(bounds.attrib['maxlat'])
                maxlon = float(bounds.attrib['maxlon'])
                
                # Calculate center point
                center_lat = (minlat + maxlat) / 2
                center_lon = (minlon + maxlon) / 2
                
                print(f"Extracted bounds from OSM file: {minlat}, {minlon}, {maxlat}, {maxlon}")
                print(f"Calculated center: {center_lat}, {center_lon}")
                
                return (center_lat, center_lon)
            else:
                # If no bounds, try to extract nodes and calculate manually
                nodes = []
                for node in root.findall('.//node'):
                    lat = float(node.attrib['lat'])
                    lon = float(node.attrib['lon'])
                    nodes.append((lat, lon))
                
                if nodes:
                    lats, lons = zip(*nodes)
                    center_lat = sum(lats) / len(lats)
                    center_lon = sum(lons) / len(lons)
                    
                    print(f"Calculated center from nodes: {center_lat}, {center_lon}")
                    return (center_lat, center_lon)
                else:
                    raise ValueError("No nodes found in the OSM file")
        except Exception as e2:
            print(f"Alternative approach also failed: {e2}")
            raise e
    
    # Calculate center from node positions
    node_points = []
    for node, data in G.nodes(data=True):
        if 'y' in data and 'x' in data:
            # In OSMnx, y is latitude and x is longitude
            node_points.append((data['y'], data['x']))
    
    if node_points:
        # Calculate the center as the average of all node positions
        center_point = np.mean(node_points, axis=0)
        # Ensure return format is (latitude, longitude)
        center_lat, center_lon = center_point
    else:
        # Fallback to the graph's bounding box (if available)
        try:
            bbox = ox.utils_graph.graph_to_gdfs(G, nodes=True, edges=False).total_bounds
            # In GeoDataFrame, bbox order is (minx, miny, maxx, maxy)
            # So bbox[0] is minx (longitude), bbox[1] is miny (latitude)
            center_lon = (bbox[0] + bbox[2]) / 2  # longitude
            center_lat = (bbox[1] + bbox[3]) / 2  # latitude
        except Exception as e:
            print(f"Error calculating center from graph: {e}")
            # Final fallback - use the graph's centroid if available
            print("Using graph's center if available...")
            if hasattr(G, 'graph') and 'center' in G.graph:
                center_point = G.graph['center']
                if isinstance(center_point, tuple) and len(center_point) == 2:
                    # Ensure return format is (latitude, longitude)
                    center_lat, center_lon = center_point
                else:
                    raise ValueError("Invalid center point in graph")
            else:
                raise ValueError("Could not determine center point from graph")
    
    center_point = (float(center_lat), float(center_lon))
    print(f"Center point: {center_point}")
    
    return center_point

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Find center point of an OSM file')
    parser.add_argument('osm_file', type=str, help='Path to the OSM file')
    args = parser.parse_args()
    
    # Check if file exists
    if not os.path.exists(args.osm_file):
        print(f"Error: File {args.osm_file} does not exist")
        exit(1)
    
    center = find_osm_center(args.osm_file)
    print(f"\nResult: Center point coordinates of OSM file '{args.osm_file}':")
    print(f"  - Latitude: {center[0]:.6f}")
    print(f"  - Longitude: {center[1]:.6f}")
    print(f"  - Formatted coordinates: {center[0]:.6f}, {center[1]:.6f}")