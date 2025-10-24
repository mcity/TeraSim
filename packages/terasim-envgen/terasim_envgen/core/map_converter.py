#!/usr/bin/env python3
"""
map_converter.py - Convert OSM data to various simulation formats (SUMO, OpenDRIVE, etc.)
"""

from concurrent.futures import ProcessPoolExecutor
import os
import logging
import shutil
import gzip
# from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet
from tqdm import tqdm
import yaml
import subprocess
from pathlib import Path
import glob
# from crdesigner.map_conversion.map_conversion_interface import opendrive_to_lanelet

logger = logging.getLogger(__name__)

def process_osm_file(osm_path):
    """Process a single OSM file with a new converter instance"""
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    config_path = os.path.join(project_root, "config/config.yaml")
    converter = MapConverter(convert_types=["sumo", "opendrive"])
    scene_dir = os.path.dirname(osm_path)
    scene_id = os.path.basename(scene_dir)
    
    logger.info(f"Processing scene {scene_id} from {osm_path}")
    
    # Convert the OSM file
    net_path, xodr_path, ll2_path = converter.convert(
        osm_path=osm_path, scene_id=scene_id, scenario_name="autonomous_driving"
    )
    
    if net_path and xodr_path:
        logger.info(f"Successfully processed scene {scene_id}")
    else:
        logger.error(f"Failed to process scene {scene_id}")
    
    return net_path, xodr_path, ll2_path

def convert_all_osm_files(output_dir=None):
    """Process all OSM files in the test_output directory"""
    # Get current directory for relative path
    current_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(current_dir)
    base_dir = Path(output_dir)
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    # Find all OSM files recursively
    # Search for .osm files in both current directory and subdirectories
    osm_files_recursive = [f for f in glob.glob(os.path.join(base_dir, "**/*.osm"), recursive=True) if not f.endswith('.lanelet2.osm')]
    osm_files_current = [f for f in glob.glob(os.path.join(base_dir, "*.osm")) if not f.endswith('.lanelet2.osm')]
    osm_files = list(set(osm_files_recursive + osm_files_current))
    if not osm_files:
        logger.warning(f"No OSM files found in {base_dir}")
        return

    logger.info(f"Found {len(osm_files)} OSM files to process")

    for osm_file in osm_files:
        process_osm_file(osm_file)

    # Process each OSM file in parallel using multiprocessing
    # with ProcessPoolExecutor() as executor:
    #     # Pass config_path to each worker process
    #     results = list(tqdm(executor.map(
    #         process_osm_file,
    #         osm_files
    #     ), total=len(osm_files), desc="Processing OSM files"))


class MapConverter:
    def __init__(self, convert_types=None):
        # Load configuration
        self.convert_types = convert_types

    def convert(self, osm_path, scene_id, scenario_name="autonomous_driving"):
        """
        Convert OSM file to SUMO network and OpenDRIVE format.

        Args:
            osm_path (str): Path to OSM file
            scene_id (int): Scene ID for file naming
            scenario_name (str): Name of the autonomous driving scenario

        Returns:
            tuple: (Path to SUMO network file, Path to OpenDRIVE file, Path to Lanelet2 file)
        """
        logger.info(f"Converting OSM file {osm_path} to SUMO network and OpenDRIVE")

        # Use the same directory as the OSM file
        output_dir = os.path.dirname(osm_path)

        # Define output file paths with simpler names
        original_net_path = os.path.join(output_dir, "osm.net.xml.gz")
        fallback_net_path = os.path.join(output_dir, "map.osm")
        net_path = os.path.join(output_dir, "map.net.xml")
        xodr_path = os.path.join(output_dir, "map.xodr")
        poly_path = os.path.join(output_dir, "map.poly.xml")
        ll2_path = os.path.join(output_dir, "map.lanelet2.osm")

        try:
            if "sumo" in self.convert_types:
                # Process the net file - decompress if it's a .gz file
                if os.path.exists(original_net_path):
                    # Decompress the .net.xml.gz file to .net.xml
                    with gzip.open(original_net_path, 'rb') as gz_file:
                        with open(net_path, 'wb') as output_file:
                            output_file.write(gz_file.read())
                    logger.info(f"Successfully decompressed {original_net_path} to {net_path}")
                else:
                    logger.warning(f"Original net file {original_net_path} does not exist, using fallback net file {fallback_net_path}")
                    netconvert_cmd = [
                        "netconvert",
                        "--osm-files",
                        fallback_net_path,
                        "--output",
                        net_path,
                        "--ramps.guess",
                        "true",
                        "--roundabouts.guess",
                        "true",
                        "--tls.guess",
                        "true",
                    ]
                    subprocess.run(netconvert_cmd, capture_output=True, text=True, check=True)

            if "opendrive" in self.convert_types:
                # Convert to OpenDRIVE
                self._convert_to_opendrive(net_path, xodr_path)

            # Convert OpenDRIVE to Lanelet2
            if "lanelet2" in self.convert_types:
                try:
                    opendrive_to_lanelet(xodr_path, ll2_path)
                except Exception as e:
                    logger.warning(f"Warning: failed to convert OpenDRIVE to Lanelet2: {e} {xodr_path}")
                    ll2_path = None
            # Generate polygon file for visualization
            self._generate_polys(osm_path, poly_path)

            logger.info(f"Successfully converted OSM to SUMO network: {net_path}")
            logger.info(f"Successfully converted to OpenDRIVE: {xodr_path}")
            return str(net_path), str(xodr_path), str(ll2_path)

        except subprocess.CalledProcessError as e:
            logger.error(f"Error running netconvert: {e}")
            if e.stdout:
                logger.error(f"stdout: {e.stdout}")
            if e.stderr:
                logger.error(f"stderr: {e.stderr}")
            return None, None, None
        except Exception as e:
            logger.error(f"Error converting OSM to SUMO/OpenDRIVE: {e}")
            return None, None, None

    def _convert_to_opendrive(self, net_path, xodr_path):
        """Convert SUMO network to OpenDRIVE format"""
        try:
            cmd = [
                "netconvert",
                "--sumo-net-file",
                str(net_path),
                "--opendrive-output",
                str(xodr_path),
                "--verbose",
                "false",
            ]

            logger.info(f"Running command: {' '.join(cmd)}")
            subprocess.run(cmd, capture_output=True, text=True, check=True)
            logger.info(f"Successfully generated OpenDRIVE file: {xodr_path}")

        except subprocess.CalledProcessError as e:
            logger.error(f"Error converting to OpenDRIVE: {e}")
            raise
        except Exception as e:
            logger.error(f"Error in OpenDRIVE conversion: {e}")
            raise

    def _generate_polys(self, osm_path, poly_path):
        """Generate polygon file for visualization"""
        try:
            cmd = [
                "polyconvert",
                "--osm-files",
                osm_path,
                "--net-file",
                str(poly_path).replace("poly.xml", "net.xml"),
                "-o",
                str(poly_path),
                "--osm.keep-full-type",
                "--verbose",
                "false",
            ]

            logger.info(f"Running command: {' '.join(cmd)}")
            subprocess.run(cmd, capture_output=True, text=True, check=True)
            logger.info(f"Successfully generated polygon file: {poly_path}")

        except subprocess.CalledProcessError as e:
            logger.warning(f"Warning: polyconvert failed: {e}")
        except Exception as e:
            logger.warning(f"Warning: failed to generate polygon file: {e}")


