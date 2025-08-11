#!/usr/bin/env python3
"""
test_map_searcher.py - Tests for map searching and downloading functionality
"""

import pytest
from concurrent.futures import ProcessPoolExecutor
import os
from pathlib import Path
import dotenv
from tqdm import tqdm
import logging
import yaml

dotenv.load_dotenv()

# Set environment variable for offscreen rendering
os.environ["QT_QPA_PLATFORM"] = "offscreen"

from terasim_envgen.core.map_searcher import MapSearcher

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def process_city_section(city, section_type):
    # Initialize MapSearcher with test config
    searcher = MapSearcher("test_config.yaml")
    logger.info(f"\nFinding {section_type} sections in {city}...")
    sections = searcher.find_section(
        city=city,
        section_type=section_type,
        save_plots=True,
        max_samples=5
    )
    logger.info(f"Found {len(sections)} {section_type} sections in {city}")
    return sections


def test_map_download(output_dir="test_output", multi_process=False):
    """Test basic map downloading functionality."""
    # Create test configuration with satellite view enabled

    config = {
        "map_search": {
            "default_city": "Ann Arbor, Michigan, USA",
            "bbox_size": 500,
            "max_results": 5,
        },
        "visualization": {
            "preview": {
                "randomize": {
                    "enabled": True,
                    "node_selection": True,
                    "visualization": True,
                    "map_style": True,
                    "seed": 33,
                },
                "satellite_view": True,  # Enable satellite view
                "interactive_map": False,
                "dpi": 300,
                "figure_width": 8,
                "figure_height": 6,
            }
        },
        "output": {
            "base_dir": output_dir,
        },
    }

    # Save test config
    with open("test_config.yaml", "w") as f:
        yaml.dump(config, f)

    # Initialize MapSearcher with test config
    searcher = MapSearcher("test_config.yaml")

    # Test city
    test_city_list = ["Ann Arbor, Michigan, USA", "Chicago, Illinois, USA", "San Francisco, California, USA", "New York, New York, USA", "Los Angeles, California, USA", "Houston, Texas, USA", "Phoenix, Arizona, USA", "Philadelphia, Pennsylvania, USA", "San Antonio, Texas, USA", "San Diego, California, USA"]
    section_types = ["signalized", "roundabout", "highway"]
    
    # single process
    if not multi_process:
        for test_city in tqdm(test_city_list, desc="Processing cities"):
            for section_type in tqdm(section_types, desc=f"Processing sections for {test_city}", leave=False):
                logger.info(f"\nFinding {section_type} sections in {test_city}...")
                sections = searcher.find_section(
                    city=test_city,
                    section_type=section_type,
                    save_plots=True,
                    max_samples=5
                )
                logger.info(f"Found {len(sections)} {section_type} sections in {test_city}")

    # multi process
    else:
        # Create all combinations of cities and section types
        tasks = [(city, section_type) for city in test_city_list for section_type in section_types]
        
        with ProcessPoolExecutor(max_workers=min(len(test_city_list), os.cpu_count())) as executor:
            # Submit all tasks and collect futures
            futures = [executor.submit(process_city_section, city, section_type) 
                      for city, section_type in tasks]
            
            # Wait for all tasks to complete and collect results with progress bar
            for future in tqdm(futures, desc="Processing tasks", total=len(tasks)):
                try:
                    future.result()  # This will wait for each task to complete
                except Exception as e:
                    logger.error(f"Task failed with error: {str(e)}")

    # Test completed
    logger.info("Map download tests completed")

def list_valid_maps(output_dir):
    # check all preview.png under output_dir including all subdirectories
    output_dir = Path(output_dir)
    preview_files = list(output_dir.glob("**/preview.png"))
    print(f"Found {len(preview_files)} preview.png files")
    total_subfolders = len(list(output_dir.glob("**/")))
    print(f"Total number of subfolders: {total_subfolders}")

def test_map_download_latlon(output_dir):
    # Initialize MapSearcher with test config
    searcher = MapSearcher("test_config.yaml")
    # Test city
    geolocation = (42.28055546332519, -83.7484335171531)
    searcher.save_osm_and_visualization_for_point(geolocation, output_dir, bbox_size=2500)

def test_latlon_with_bounding_box():
    """Test save_osm_and_visualization_for_point function with various latlon and bbox_size combinations."""
    
    # Test configurations
    test_cases = [
        {
            "name": "austin_police_case",
            "latlon": (30.230930876683622, -97.73349014424019),
            "bbox_size": 500,
            "output_dir": "austin_cases/austin_police_case"
        },
        {
            "name": "austin_wrong_leftturn_case",
            "latlon": (30.25848277319393, -97.74828142386522),
            "bbox_size": 500,
            "output_dir": "austin_cases/austin_wrong_leftturn_case"
        }
    ]
    
    # Initialize MapSearcher with test config
    searcher = MapSearcher("test_config.yaml")
    
    for test_case in test_cases:
        logger.info(f"Testing {test_case['name']} with bbox_size={test_case['bbox_size']}m")
        success = searcher.save_osm_and_visualization_for_point(
                point_data=test_case['latlon'],
                scene_dir=test_case['output_dir'],
                bbox_size=test_case['bbox_size']
        )
    
    logger.info("Latlon bounding box test completed")
    

if __name__ == "__main__":
    # test_map_download(output_dir="test_output_validate", multi_process=True)
    # list_valid_maps(output_dir="test_output_validate")
    test_map_download_latlon(output_dir="test_demo")
    # test_latlon_with_bounding_box()