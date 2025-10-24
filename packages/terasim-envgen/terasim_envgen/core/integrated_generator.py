"""
Integrated generator that combines map search, conversion, and traffic flow generation.
Accepts lat/lon coordinates with bounding box to produce complete simulation scenarios.
"""

import os
import sys
import logging
from pathlib import Path
from typing import Tuple, Optional, List, Dict

# Add project root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)

from terasim_envgen.core.map_searcher import MapSearcher
from terasim_envgen.core.traffic_flow_generator import TrafficFlowGenerator
from terasim_envgen.core.map_converter import MapConverter


class IntegratedScenarioGenerator:
    """
    Integrated generator that creates complete simulation scenarios from lat/lon coordinates.
    Combines map downloading, format conversion, and traffic flow generation.
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize the integrated generator with all necessary components.
        
        Args:
            config_path: Path to configuration file
        """
        self.config_path = config_path
        self.map_searcher = MapSearcher(config_path)
        self.map_converter = MapConverter()
        self.traffic_generator = TrafficFlowGenerator(config_path)
        
    def generate_from_latlon(
        self, 
        lat: float, 
        lon: float, 
        bbox_size: int = 500,
        output_dir: str = "generated_scenarios",
        scenario_name: Optional[str] = None,
        traffic_densities: List[str] = ["low", "medium", "high"],
        convert_formats: List[str] = ["sumo", "opendrive"],
        av_route: Optional[List[Tuple[float, float]]] = None
    ) -> Dict:
        """
        Generate a complete simulation scenario from latitude/longitude coordinates.
        
        Args:
            lat: Latitude of the center point
            lon: Longitude of the center point
            bbox_size: Size of bounding box in meters (default: 500)
            output_dir: Base output directory for generated files
            scenario_name: Optional name for the scenario (auto-generated if None)
            traffic_densities: List of traffic density levels to generate
            convert_formats: List of formats to convert map to ("sumo", "opendrive", "lanelet2")
            av_route: Optional list of route coordinates [(lat, lon), ...] within the bounding box
            
        Returns:
            Dictionary containing paths to all generated files and status information
        """
        results = {
            "status": "success",
            "center_coordinates": (lat, lon),
            "bbox_size": bbox_size,
            "generated_files": {}
        }
        
        # Generate scenario name if not provided
        if scenario_name is None:
            scenario_name = f"scenario_{lat:.6f}_{lon:.6f}_{bbox_size}m"
        
        # Create output directory
        scene_dir = Path(output_dir) / scenario_name
        scene_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"Starting integrated scenario generation for {scenario_name}")
        logger.info(f"Center: ({lat}, {lon}), BBox: {bbox_size}m")
        
        # Step 1: Download OSM map and create visualization
        logger.info("Step 1: Downloading OSM map...")
        try:
            success = self.map_searcher.save_osm_and_visualization_for_point(
                point_data=(lat, lon),
                scene_dir=str(scene_dir),
                bbox_size=bbox_size,
                av_route=av_route
            )
            
            if not success:
                results["status"] = "failed"
                results["error"] = "Failed to download OSM map"
                return results
                
            osm_path = scene_dir / "map.osm"
            if osm_path.exists():
                results["generated_files"]["osm"] = str(osm_path)
                logger.info(f"OSM map saved to {osm_path}")
                
            # Check for visualization files
            preview_path = scene_dir / "preview.png"
            if preview_path.exists():
                results["generated_files"]["preview"] = str(preview_path)
                
        except Exception as e:
            logger.error(f"Error downloading OSM map: {e}")
            results["status"] = "failed"
            results["error"] = str(e)
            return results
        
        # Step 2: Convert map to requested formats
        logger.info(f"Step 2: Converting map to {convert_formats}...")
        
        try:
            # Set converter types based on requested formats
            converter_types = []
            if "sumo" in convert_formats:
                converter_types.append("sumo")
            if "opendrive" in convert_formats:
                converter_types.append("opendrive")
            if "lanelet2" in convert_formats:
                converter_types.append("lanelet2")
                
            self.map_converter.convert_types = converter_types
            
            # Call the convert method
            net_path, xodr_path, ll2_path = self.map_converter.convert(
                osm_path=str(osm_path),
                scene_id=scenario_name,
                scenario_name="autonomous_driving"
            )
            
            conversion_results = {}
            if net_path:
                conversion_results["sumo_network"] = net_path
                logger.info(f"SUMO network saved to {net_path}")
            if xodr_path:
                conversion_results["opendrive"] = xodr_path
                logger.info(f"OpenDRIVE map saved to {xodr_path}")
            if ll2_path:
                conversion_results["lanelet2"] = ll2_path
                logger.info(f"Lanelet2 map saved to {ll2_path}")
                
            results["generated_files"].update(conversion_results)
            
        except Exception as e:
            logger.error(f"Error converting map: {e}")
            results["conversion_error"] = str(e)
        
        # Step 3: Generate traffic flows for different densities
        if "sumo_network" in results["generated_files"]:
            logger.info(f"Step 3: Generating traffic flows for densities: {traffic_densities}...")
            try:
                # Use generate_multi_level_flows if we want all standard levels
                if set(traffic_densities) == {"low", "medium", "high"}:
                    logger.info("Generating all traffic density levels...")
                    flow_results = self.traffic_generator.generate_multi_level_flows(str(scene_dir))
                    
                    if flow_results:
                        traffic_files = {}
                        for net_file, levels in flow_results.items():
                            for level, route_files in levels.items():
                                if route_files:
                                    # Find corresponding config file
                                    config_pattern = str(scene_dir / f"sumo_{level}_*.sumocfg")
                                    import glob
                                    config_files = glob.glob(config_pattern)
                                    config_file = config_files[0] if config_files else None
                                    
                                    traffic_files[f"traffic_{level}"] = {
                                        "route_files": route_files,
                                        "config_file": config_file
                                    }
                                    logger.info(f"{level.capitalize()} density traffic generated")
                        
                        results["generated_files"]["traffic"] = traffic_files
                else:
                    # Generate individual levels
                    traffic_files = {}
                    net_file = results["generated_files"]["sumo_network"]
                    
                    for density in traffic_densities:
                        logger.info(f"Generating {density} density traffic...")
                        route_files, config_files = self.traffic_generator.generate_flows(
                            net_path=net_file,
                            traffic_level=density,
                            end_time=3600  # 1 hour simulation
                        )
                        
                        if route_files:
                            traffic_files[f"traffic_{density}"] = {
                                "route_files": route_files,
                                "config_files": config_files
                            }
                            logger.info(f"{density.capitalize()} density traffic generated")
                    
                    results["generated_files"]["traffic"] = traffic_files
                
            except Exception as e:
                logger.error(f"Error generating traffic flows: {e}")
                results["traffic_error"] = str(e)
        
        # Step 4: Generate metadata summary
        metadata_path = scene_dir / "generation_summary.json"
        try:
            import json
            with open(metadata_path, 'w') as f:
                json.dump(results, f, indent=2)
            results["generated_files"]["summary"] = str(metadata_path)
            logger.info(f"Generation summary saved to {metadata_path}")
        except Exception as e:
            logger.error(f"Error saving metadata: {e}")
        
        logger.info(f"Scenario generation completed for {scenario_name}")
        return results
    
    def generate_batch(
        self,
        coordinates_list: List[Tuple[float, float]],
        bbox_size: int = 500,
        output_dir: str = "generated_scenarios",
        **kwargs
    ) -> List[Dict]:
        """
        Generate multiple scenarios from a list of coordinate pairs.
        
        Args:
            coordinates_list: List of (lat, lon) tuples
            bbox_size: Size of bounding box in meters
            output_dir: Base output directory
            **kwargs: Additional arguments passed to generate_from_latlon
            
        Returns:
            List of result dictionaries for each scenario
        """
        results = []
        
        for i, (lat, lon) in enumerate(coordinates_list):
            logger.info(f"Processing scenario {i+1}/{len(coordinates_list)}")
            scenario_name = kwargs.pop("scenario_name", None) or f"batch_scenario_{i+1}"
            
            result = self.generate_from_latlon(
                lat=lat,
                lon=lon,
                bbox_size=bbox_size,
                output_dir=output_dir,
                scenario_name=scenario_name,
                **kwargs
            )
            results.append(result)
            
        return results


def generate_scenario_from_latlon(
    lat: float,
    lon: float, 
    bbox_size: int = 500,
    output_dir: str = "generated_scenarios",
    scenario_name: Optional[str] = None,
    config_path: str = None,
    traffic_densities: List[str] = ["low", "medium", "high"],
    convert_formats: List[str] = ["sumo", "opendrive"]
) -> Dict:
    """
    Convenience function to generate a complete scenario from lat/lon coordinates.
    
    This is a wrapper around IntegratedScenarioGenerator for simple one-off generations.
    
    Args:
        lat: Latitude of the center point
        lon: Longitude of the center point
        bbox_size: Size of bounding box in meters
        output_dir: Output directory for generated files
        scenario_name: Optional name for the scenario
        config_path: Path to configuration file
        traffic_densities: Traffic density levels to generate
        convert_formats: Map formats to convert to
        
    Returns:
        Dictionary with generated file paths and status
    """
    generator = IntegratedScenarioGenerator(config_path)
    return generator.generate_from_latlon(
        lat=lat,
        lon=lon,
        bbox_size=bbox_size,
        output_dir=output_dir,
        scenario_name=scenario_name,
        traffic_densities=traffic_densities,
        convert_formats=convert_formats
    )


if __name__ == "__main__":
    # Example usage
    import dotenv
    dotenv.load_dotenv()
    
    # Test with Austin police case coordinates
    result = generate_scenario_from_latlon(
        lat=30.230930876683622,
        lon=-97.73349014424019,
        bbox_size=500,
        output_dir="test_integrated",
        scenario_name="austin_police_integrated",
        config_path="test_config.yaml",
        traffic_densities=["low", "medium"],
        convert_formats=["sumo", "opendrive"]
    )
    
    print("\nGenerated files:")
    for category, files in result.get("generated_files", {}).items():
        print(f"\n{category}:")
        if isinstance(files, dict):
            for key, value in files.items():
                print(f"  {key}: {value}")
        else:
            print(f"  {files}")