import json
import fiftyone as fo
from fiftyone.core.labels import GeoLocation
import os
import glob
from terasim_envgen.utils.find_osm_center import find_osm_center
# Load environment variables from .env file
from dotenv import load_dotenv
import uuid
import atexit
import signal
import psutil

load_dotenv()
# Get Mapbox token from environment variables
mapbox_token = os.getenv('MAPBOX_TOKEN')
# fo.config.mapbox_token = mapbox_token

def extract_port_from_url(url: str) -> str:
    try:
        port = url.split(":")[-1]
        return port
    except Exception:
        return str(uuid.uuid4())
    
def load_and_launch_fiftyone_map_route_visualization(root_dir="test_output", dataset_name="TeraSim-Generated-Dataset"):
    """
    Load image data (preview.png) from directories containing both preview.png and metadata.json files 
    into a FiftyOne dataset and launch the app for map route visualization.
    
    Args:
        root_dir (str): Root directory to search for preview.png and metadata.json files
        dataset_name (str): Name of the dataset to create
        
    Returns:
        dict: Dictionary containing dataset, session, url, and session_id
    """
    # Find all directories that contain both preview.png and metadata.json
    samples = []
    
    # Search for all preview.png files recursively
    preview_files = glob.glob(f"{root_dir}/**/preview.png", recursive=True)
    
    for preview_file in preview_files:
        # Check if metadata.json exists in the same directory
        preview_dir = os.path.dirname(preview_file)
        metadata_file = os.path.join(preview_dir, "metadata.json")
        
        if not os.path.exists(metadata_file):
            continue
            
        # Load metadata to get center coordinates and route information
        try:
            with open(metadata_file, "r") as f:
                metadata = json.load(f)
            
            center_coords = metadata.get("center_coordinates", [0, 0])
            scene_id = metadata.get("scene_id", "unknown")
            av_route = metadata.get("av_route", [])
            bbox_size = metadata.get("bbox_size", 1000)
            osm_center = metadata["center_coordinates"]
        
            location = fo.GeoLocation(latitude=osm_center[0], longitude=osm_center[1])
            location.point = [osm_center[1], osm_center[0]]
            # Create sample with the preview image (simplified - no location for now)
            sample = fo.Sample(filepath=preview_file, location=location)
            
            # Add metadata as custom fields
            sample["scene_id"] = scene_id
            sample["bbox_size"] = bbox_size
            sample["center_latitude"] = center_coords[0]
            sample["center_longitude"] = center_coords[1]
            sample["route_points_count"] = len(av_route)
            
            # Add route information as a polyline (if route exists)
            if av_route:
                # Convert route to the format expected by FiftyOne
                # Each point should be [longitude, latitude]
                route_points = [[point[1], point[0]] for point in av_route]
                sample["av_route"] = route_points
            
            # Add tags for easier filtering
            sample.tags.append("route_visualization")
            sample.tags.append(f"scene_{scene_id}")
            
            samples.append(sample)
            
        except Exception as e:
            print(f"Error processing {preview_file}: {e}")
            continue
    
    print(f"Found {len(preview_files)} preview files, successfully processed {len(samples)} samples")
    
    # Check if dataset exists and handle it
    if fo.dataset_exists(dataset_name):
        print(f"Dataset '{dataset_name}' already exists. Deleting it...")
        fo.delete_dataset(dataset_name)
    
    # Create new dataset
    dataset = fo.Dataset(dataset_name)
    
    # Add custom fields to the dataset schema
    dataset.add_sample_field("scene_id", fo.StringField)
    dataset.add_sample_field("bbox_size", fo.IntField)
    dataset.add_sample_field("center_latitude", fo.FloatField)
    dataset.add_sample_field("center_longitude", fo.FloatField)
    dataset.add_sample_field("route_points_count", fo.IntField)
    # Skip the complex av_route field and location field for now - focus on basic functionality
    
    # Add samples to the dataset
    if samples:
        dataset.add_samples(samples)
    else:
        print("Warning: No valid samples found to add to the dataset")
    
    # Configure and launch the app
    fo.config.session_timeout = 3600
    session = fo.launch_app(dataset, auto=False, remote=True)
    url = session.url
    session_id = extract_port_from_url(url)
    print(f"FiftyOne app URL: {url}")
    print(f"Dataset contains {len(samples)} route visualization samples")
    
    # Return dataset and session information
    return {
        "dataset": dataset,
        "session": session,
        "url": url,
        "session_id": session_id
    }

def load_and_launch_fiftyone(root_dir="test_output_demo", filter_type=None, dataset_name="TeraSim-Generated-Dataset"):
    """
    Load video data from the specified directory into a FiftyOne dataset and launch the app.
    
    Args:
        root_dir (str): Root directory to search for video files
        filter_type (str): Accident type to filter by (None to include all)
        dataset_name (str): Name of the dataset to create
        
    Returns:
        tuple: (dataset, session) - The created dataset and the FiftyOne session
    """
    # Get all mp4 files under root_dir recursively
    video_files = glob.glob(f"{root_dir}/**/*.mp4", recursive=True)
    samples = []
    
    for video_file in video_files:
        # Get the accident type from the parent directory name
        accident_type = os.path.basename(os.path.dirname(video_file)).rsplit('_', 1)[0]
        
        # Filter by accident type if specified
        if filter_type and filter_type not in accident_type:
            continue
            
        # Get the osm file from grandparent directory
        grandparent_dir = os.path.dirname(os.path.dirname(os.path.dirname(video_file)))
        
        # get the metadata.json file from the grandparent directory
        metadata_file = os.path.join(grandparent_dir, "metadata.json")
        if not os.path.exists(metadata_file):
            osm_file = os.path.join(grandparent_dir, "map.osm")
            osm_center = find_osm_center(osm_file)
            # Write metadata file with center coordinates if it doesn't exist
            metadata = {
                "center_coordinates": osm_center
            }
            with open(metadata_file, "w") as f:
                json.dump(metadata, f, indent=4)
        with open(metadata_file, "r") as f:
            metadata = json.load(f)

        
        osm_center = metadata["center_coordinates"]
        
        location = fo.GeoLocation(latitude=osm_center[0], longitude=osm_center[1])
        location.point = [osm_center[1], osm_center[0]]
        
        sample = fo.Sample(filepath=video_file, location=location)
        sample.tags.append(accident_type)  # Add accident type as a tag
        sample["accident_type"] = accident_type  # Also store in metadata
        
        samples.append(sample)
    
    print(f"Found {len(video_files)} video files, filtered to {len(samples)} samples")
    
    # Check if dataset exists and handle it
    if fo.dataset_exists(dataset_name):
        print(f"Dataset '{dataset_name}' already exists. Deleting it...")
        fo.delete_dataset(dataset_name)
    
    # Create new dataset
    dataset = fo.Dataset(dataset_name)
    
    # Add fields to the dataset
    dataset.add_sample_field("GeoLocation", fo.GeoPointField)
    
    # Add samples to the dataset
    dataset.add_samples(samples)
    
    # Configure and launch the app
    fo.config.session_timeout = 3600
    session = fo.launch_app(dataset, auto=False, remote=True)
    url = session.url
    session_id = extract_port_from_url(url)
    print(f"FiftyOne app URL: {url}")
    # Return dataset and session without waiting
    return {
        "dataset": dataset,
        "session": session,
        "url": url,
        "session_id": session_id
    }

def cleanup_on_exit():
    try:
        import fiftyone as fo
        fo.close_app()
    except:
        pass
    

atexit.register(cleanup_on_exit)

def signal_handler(signum, frame):
    cleanup_on_exit()
    exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Example usage when run directly
if __name__ == "__main__":
    result = load_and_launch_fiftyone()
    session = result["session"]
    session.wait()  # Only wait when run directly