#!/usr/bin/env python3
"""
TeraSim Experiment Generator

Generate complete simulation scenarios from latitude/longitude coordinates.
This tool combines map downloading, format conversion, and traffic flow generation
to create ready-to-use simulation environments for autonomous vehicle testing.

Usage Examples:
    # Single scenario generation
    python generate_experiments.py --lat 42.2803162048774 --lon -83.72897525866705 --bbox 6000 --name ann_arbor
    
    # Batch generation from file
    python generate_experiments.py --batch coordinates.txt --bbox 300
    
    # Custom traffic densities
    python generate_experiments.py --lat 30.2309 --lon -97.7335 --traffic low medium
    
    # Specify output directory and formats
    python generate_experiments.py --lat 37.7749 --lon -122.4194 --output experiments/sf --formats sumo opendrive
"""

import argparse
import json
import logging
import sys
from pathlib import Path
from typing import List, Tuple, Optional, Dict
import time

try:
    from terasim_envgen import IntegratedScenarioGenerator
except ImportError as e:
    print(f"Error importing terasim_envgen: {e}")
    print("Please ensure terasim-envgen is installed: poetry install")
    sys.exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="Generate TeraSim simulation scenarios from geographic coordinates",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    # Input options (mutually exclusive)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument(
        "--lat", 
        type=float,
        help="Latitude of the center point for single scenario"
    )
    input_group.add_argument(
        "--batch",
        type=str,
        help="Path to file containing lat,lon pairs (one per line)"
    )
    input_group.add_argument(
        "--coordinates",
        nargs="+",
        type=float,
        help="List of coordinates as: lat1 lon1 lat2 lon2 ..."
    )
    
    # Longitude (required with --lat)
    parser.add_argument(
        "--lon",
        type=float,
        help="Longitude of the center point (required with --lat)"
    )
    
    # Common parameters
    parser.add_argument(
        "--bbox",
        type=int,
        default=500,
        help="Size of bounding box in meters (default: 500)"
    )
    
    parser.add_argument(
        "--output",
        type=str,
        default="generated_experiments",
        help="Output directory for generated files (default: generated_experiments)"
    )
    
    parser.add_argument(
        "--name",
        type=str,
        help="Name for the scenario (auto-generated if not provided)"
    )
    
    parser.add_argument(
        "--traffic",
        nargs="+",
        choices=["low", "medium", "high"],
        default=["low", "medium", "high"],
        help="Traffic density levels to generate (default: all)"
    )
    
    parser.add_argument(
        "--formats",
        nargs="+",
        choices=["sumo", "opendrive", "lanelet2"],
        default=["sumo"],
        help="Map formats to convert to (default: sumo)"
    )
    
    parser.add_argument(
        "--config",
        type=str,
        help="Path to configuration file for advanced settings"
    )
    
    parser.add_argument(
        "--route",
        type=str,
        help="Path to file containing AV route coordinates (lat,lon per line)"
    )
    
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging"
    )
    
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show what would be generated without actually generating"
    )
    
    return parser.parse_args()


def load_coordinates_from_file(filepath: str) -> List[Tuple[float, float]]:
    """
    Load coordinates from a text file.
    
    Expected format: lat,lon (one pair per line)
    Lines starting with # are treated as comments.
    
    Args:
        filepath: Path to the coordinates file
        
    Returns:
        List of (lat, lon) tuples
    """
    coordinates = []
    
    try:
        with open(filepath, 'r') as f:
            for line_num, line in enumerate(f, 1):
                line = line.strip()
                
                # Skip empty lines and comments
                if not line or line.startswith('#'):
                    continue
                
                # Parse lat,lon
                try:
                    parts = line.split(',')
                    if len(parts) != 2:
                        logger.warning(f"Line {line_num}: Invalid format (expected 'lat,lon'): {line}")
                        continue
                    
                    lat = float(parts[0].strip())
                    lon = float(parts[1].strip())
                    coordinates.append((lat, lon))
                    
                except ValueError as e:
                    logger.warning(f"Line {line_num}: Could not parse coordinates: {line} ({e})")
                    continue
                    
    except FileNotFoundError:
        logger.error(f"Coordinates file not found: {filepath}")
        sys.exit(1)
    except Exception as e:
        logger.error(f"Error reading coordinates file: {e}")
        sys.exit(1)
    
    return coordinates


def load_route_from_file(filepath: str) -> Optional[List[Tuple[float, float]]]:
    """Load AV route coordinates from file."""
    if not filepath:
        return None
    
    return load_coordinates_from_file(filepath)


def generate_single_scenario(
    generator: IntegratedScenarioGenerator,
    lat: float,
    lon: float,
    args: argparse.Namespace,
    scenario_name: Optional[str] = None
) -> Dict:
    """
    Generate a single scenario.
    
    Args:
        generator: The integrated scenario generator instance
        lat: Latitude of center point
        lon: Longitude of center point
        args: Command line arguments
        scenario_name: Optional name for the scenario
        
    Returns:
        Dictionary with generation results
    """
    # Load AV route if provided
    av_route = load_route_from_file(args.route) if hasattr(args, 'route') else None
    
    # Use provided name or generate one
    if not scenario_name:
        scenario_name = args.name if args.name else f"scenario_{lat:.6f}_{lon:.6f}"
    
    logger.info(f"Generating scenario: {scenario_name}")
    logger.info(f"  Location: ({lat}, {lon})")
    logger.info(f"  Bounding box: {args.bbox}m")
    logger.info(f"  Traffic densities: {args.traffic}")
    logger.info(f"  Map formats: {args.formats}")
    
    if args.dry_run:
        logger.info("  [DRY RUN] Would generate files to: %s/%s", args.output, scenario_name)
        return {"status": "dry_run", "scenario_name": scenario_name}
    
    # Generate the scenario
    start_time = time.time()
    
    try:
        result = generator.generate_from_latlon(
            lat=lat,
            lon=lon,
            bbox_size=args.bbox,
            output_dir=args.output,
            scenario_name=scenario_name,
            traffic_densities=args.traffic,
            convert_formats=args.formats,
            av_route=av_route
        )
        
        elapsed_time = time.time() - start_time
        logger.info(f"  Generation completed in {elapsed_time:.2f} seconds")
        
        # Log generated files
        if result.get("status") == "success":
            logger.info("  Generated files:")
            for category, files in result.get("generated_files", {}).items():
                if isinstance(files, dict):
                    logger.info(f"    {category}:")
                    for key, value in files.items():
                        logger.info(f"      {key}: {value}")
                else:
                    logger.info(f"    {category}: {files}")
        else:
            logger.error(f"  Generation failed: {result.get('error', 'Unknown error')}")
            
    except Exception as e:
        logger.error(f"  Error generating scenario: {e}")
        result = {"status": "error", "error": str(e)}
    
    return result


def main():
    """Main function."""
    args = parse_arguments()
    
    # Set logging level
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # Validate arguments
    if args.lat is not None and args.lon is None:
        logger.error("Longitude (--lon) is required when using --lat")
        sys.exit(1)
    
    # Initialize generator
    logger.info("Initializing TeraSim scenario generator...")
    try:
        generator = IntegratedScenarioGenerator(config_path=args.config)
    except Exception as e:
        logger.error(f"Failed to initialize generator: {e}")
        sys.exit(1)
    
    # Determine coordinates to process
    coordinates = []
    
    if args.lat is not None:
        # Single coordinate
        coordinates = [(args.lat, args.lon)]
        
    elif args.batch:
        # Load from file
        logger.info(f"Loading coordinates from: {args.batch}")
        coordinates = load_coordinates_from_file(args.batch)
        logger.info(f"Loaded {len(coordinates)} coordinate pairs")
        
    elif args.coordinates:
        # Parse from command line
        if len(args.coordinates) % 2 != 0:
            logger.error("Coordinates must be provided in lat,lon pairs")
            sys.exit(1)
        
        coordinates = [
            (args.coordinates[i], args.coordinates[i+1]) 
            for i in range(0, len(args.coordinates), 2)
        ]
        logger.info(f"Processing {len(coordinates)} coordinate pairs")
    
    if not coordinates:
        logger.error("No valid coordinates to process")
        sys.exit(1)
    
    # Generate scenarios
    logger.info("="*60)
    logger.info(f"Starting generation of {len(coordinates)} scenario(s)")
    logger.info("="*60)
    
    results = []
    success_count = 0
    
    for i, (lat, lon) in enumerate(coordinates, 1):
        logger.info(f"\n[{i}/{len(coordinates)}] Processing coordinates: ({lat}, {lon})")
        
        # Generate scenario name for batch processing
        scenario_name = None
        if len(coordinates) > 1:
            scenario_name = f"batch_{i:03d}_{lat:.6f}_{lon:.6f}"
        
        result = generate_single_scenario(generator, lat, lon, args, scenario_name)
        results.append(result)
        
        if result.get("status") == "success":
            success_count += 1
    
    # Summary
    logger.info("\n" + "="*60)
    logger.info("GENERATION SUMMARY")
    logger.info("="*60)
    logger.info(f"Total scenarios: {len(coordinates)}")
    logger.info(f"Successful: {success_count}")
    logger.info(f"Failed: {len(coordinates) - success_count}")
    
    if not args.dry_run:
        # Save summary to file
        summary_file = Path(args.output) / "generation_summary.json"
        summary_file.parent.mkdir(parents=True, exist_ok=True)
        
        summary_data = {
            "total_scenarios": len(coordinates),
            "successful": success_count,
            "failed": len(coordinates) - success_count,
            "parameters": {
                "bbox_size": args.bbox,
                "traffic_densities": args.traffic,
                "map_formats": args.formats,
                "output_directory": args.output
            },
            "results": results
        }
        
        with open(summary_file, 'w') as f:
            json.dump(summary_data, f, indent=2)
        
        logger.info(f"\nSummary saved to: {summary_file}")
    
    # Exit with appropriate code
    sys.exit(0 if success_count == len(coordinates) else 1)


if __name__ == "__main__":
    main()