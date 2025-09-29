from pathlib import Path
import sys
import argparse

from terasim_cosmos import TeraSimToCosmosConverter


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Convert TeraSim simulation data to Cosmos-Drive compatible format"
    )

    # Required parameters
    parser.add_argument("--path_to_output", type=Path, required=True,
                        help="Output directory path")
    parser.add_argument("--path_to_fcd", type=Path, required=True,
                        help="Path to FCD (Floating Car Data) XML file")
    parser.add_argument("--path_to_map", type=Path, required=True,
                        help="Path to SUMO network map XML file")
    parser.add_argument("--time_start", type=float, required=True,
                        help="Start time in seconds")
    parser.add_argument("--time_end", type=float, required=True,
                        help="End time in seconds")
    parser.add_argument("--vehicle_id", type=str, required=True,
                        help="ID of the ego vehicle")

    # Optional parameters
    parser.add_argument("--camera_setting_name", type=str, choices=["default", "waymo"],
                        default="default",
                        help="Camera configuration setting (default: %(default)s)")
    parser.add_argument("--agent_clip_distance", type=float, default=80.0,
                        help="Distance threshold to show agents (default: %(default)s) meters")
    parser.add_argument("--map_clip_distance", type=float, default=100.0,
                        help="Distance threshold to show map features (default: %(default)s) meters")

    # Processing options
    parser.add_argument("--streetview_retrieval", action="store_true",
                        help="Retrieve street view imagery and generate text descriptions")
    parser.add_argument("--no_streetview_retrieval", dest="streetview_retrieval", action="store_false",
                        help="Disable street view retrieval")
    parser.set_defaults(streetview_retrieval=True)

    args = parser.parse_args()

    # Create config dictionary from command line arguments
    config_dict = {
        "path_to_output": str(args.path_to_output),
        "path_to_fcd": str(args.path_to_fcd),
        "path_to_map": str(args.path_to_map),
        "camera_setting_name": args.camera_setting_name,
        "vehicle_id": args.vehicle_id,
        "time_start": args.time_start,
        "time_end": args.time_end,
        "agent_clip_distance": args.agent_clip_distance,
        "map_clip_distance": args.map_clip_distance,
        "streetview_retrieval": args.streetview_retrieval,
    }

    # Create converter and run conversion
    converter = TeraSimToCosmosConverter.from_config_dict(config_dict)
    converter.convert()