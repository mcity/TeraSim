from pathlib import Path
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import yaml
from terasim_envgen.core.map_searcher import MapSearcher

config = {
    "map_search": {
        "default_city": "Ann Arbor, Michigan, USA",
        "bbox_size": 1000,
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
        "base_dir": "test_output",
    },
}

# Save test config
with open("test_config.yaml", "w") as f:
    yaml.dump(config, f)

def test_get_maps_through_route(origin_str, destination_str, output_dir=None):
    map_searcher = MapSearcher("test_config.yaml")
    data = map_searcher.get_maps_through_route(origin_str, destination_str, output_dir=output_dir)
    # print(data)

def list_valid_maps(output_dir):
    # check all preview.png under output_dir including all subdirectories
    output_dir = Path(output_dir)
    preview_files = list(output_dir.glob("**/preview.png"))
    print(f"Found {len(preview_files)} preview.png files")
    total_subfolders = len(list(output_dir.glob("**/")))
    print(f"Total number of subfolders: {total_subfolders}")

if __name__ == "__main__":
    output_dir = "test_output_texas_1km_bbox_beginend"
    # output_dir = "test_output_AA_500m_bbox"
    test_get_maps_through_route(origin_str="US-90, Sealy, TX 77474, USA", destination_str="I-10 Frontage Rd, San Antonio, TX 78244, USA", output_dir=output_dir)
    # list_valid_maps(output_dir)