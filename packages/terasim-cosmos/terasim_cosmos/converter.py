"""
TeraSim to Cosmos-Drive Converter

Main converter class that orchestrates the conversion pipeline from TeraSim simulation
outputs (SUMO map and FCD) to Cosmos-Drive compatible inputs for world model training.
"""

from pathlib import Path
import json
import yaml
from typing import Optional

from .convert_terasim_to_rds_hq import convert_terasim_to_wds
from .render_from_rds_hq import render_sample_hdmap
from .street_view_analysis import StreetViewRetrievalAndAnalysis


class TeraSimToCosmosConverter:
    """
    Main converter class for converting TeraSim simulation outputs to Cosmos-Drive inputs.

    This class orchestrates the entire pipeline:
    1. Load configuration
    2. Extract vehicle information from collision records
    3. Optionally retrieve and analyze street view imagery
    4. Convert TeraSim data to WebDataset (WDS) format compatible with Cosmos-Drive
    5. Render HD map videos and sensor data for world model training
    """

    def __init__(self, config_path: Optional[Path] = None, config_dict: Optional[dict] = None):
        """
        Initialize the converter with configuration.

        Args:
            config_path: Path to YAML configuration file
            config_dict: Configuration dictionary (alternative to config_path)
        """
        if config_path and config_dict:
            raise ValueError("Provide either config_path or config_dict, not both")

        if config_path:
            with open(config_path, "r") as f:
                self.config = yaml.safe_load(f)
        elif config_dict:
            self.config = config_dict
        else:
            raise ValueError("Must provide either config_path or config_dict")

        # Extract configuration parameters
        self.path_to_output = Path(self.config["path_to_output"])
        self.path_to_fcd = Path(self.config["path_to_fcd"])
        self.path_to_map = Path(self.config["path_to_map"])
        self.camera_setting_name = self.config["camera_setting_name"]
        self.vehicle_id = self.config.get("vehicle_id")
        self.time_start = self.config["time_start"]
        self.time_end = self.config["time_end"]
        self.agent_clip_distance = self.config.get("agent_clip_distance", 30.0)
        self.map_clip_distance = self.config.get("map_clip_distance", 100.0)
        self.streetview_retrieval = self.config.get("streetview_retrieval", True)

        self.path_to_output = self.path_to_output / f"{self.vehicle_id}_{self.time_start:.1f}_{self.time_end:.1f}".replace(".", "_")

        # Initialize street view analyzer
        self.streetview_analyzer = StreetViewRetrievalAndAnalysis()

    def _load_vehicle_id_from_monitor(self) -> str:
        """
        Load vehicle ID from monitor.json collision record.

        Returns:
            Vehicle ID from collision record

        Raises:
            Exception: If monitor.json cannot be loaded or parsed
        """
        try:
            path_to_collision_record = self.path_to_output / "monitor.json"
            with open(path_to_collision_record, "r") as f:
                collision_record = json.load(f)
            vehicle_id = collision_record["veh_1_id"]
            print(f"Using vehicle id from monitor.json: {vehicle_id}")
            return vehicle_id
        except Exception as e:
            raise Exception(f"Error loading monitor.json: {e}")

    def _get_camera_settings(self) -> dict:
        """
        Load camera settings based on camera_setting_name.

        Returns:
            Camera settings dictionary

        Raises:
            ValueError: If invalid camera setting name is provided
        """
        # Get path to config directory within this package
        package_root = Path(__file__).parent
        config_dir = package_root / "config"

        if self.camera_setting_name == "waymo":
            config_path = config_dir / "dataset_waymo_mv_pinhole.json"
        elif self.camera_setting_name == "default":
            config_path = config_dir / "dataset_rds_hq_mv_terasim.json"
        else:
            raise ValueError(f"Invalid camera setting name: {self.camera_setting_name}")

        with open(config_path, "r") as f:
            settings = json.load(f)
        return settings

    def convert(self) -> bool:
        """
        Execute the full conversion pipeline.
        """
        print(f"Processing fcd: {self.path_to_fcd}")
        print(f"Processing map: {self.path_to_map}")

        # Create output directory
        self.path_to_output.mkdir(parents=True, exist_ok=True)

        # Resolve vehicle ID
        if self.vehicle_id is None:
            print("No vehicle id provided, trying to load from monitor.json")
            self.vehicle_id = self._load_vehicle_id_from_monitor()

        # Street view retrieval and analysis
        if self.streetview_retrieval:
            print("Retrieving and analyzing street view imagery...")
            self.streetview_analyzer.get_streetview_image_and_description(
                path_to_output=self.path_to_output,
                path_to_fcd=self.path_to_fcd,
                path_to_map=self.path_to_map,
                vehicle_id=self.vehicle_id,
                target_time=self.time_start
            )

        # Convert TeraSim data to WebDataset format
        print("Converting TeraSim data to WebDataset format...")
        convert_terasim_to_wds(
            terasim_record_root=self.path_to_output,
            path_to_fcd=self.path_to_fcd,
            path_to_map=self.path_to_map,
            output_wds_path=self.path_to_output / "wds",
            single_camera=False,
            camera_setting_name=self.camera_setting_name,
            av_id=self.vehicle_id,
            time_start=self.time_start,
            time_end=self.time_end,
            agent_clip_distance=self.agent_clip_distance,
            map_clip_distance=self.map_clip_distance
        )

        # Load camera settings and render HD maps
        print("Rendering HD maps and sensor data...")
        settings = self._get_camera_settings()

        render_sample_hdmap(
            input_root=self.path_to_output / "wds",
            output_root=self.path_to_output / "render",
            clip_id=self.path_to_output.stem,
            settings=settings,
            camera_type=settings['CAMERA_TYPE']
        )

        print("Conversion completed successfully!")
        return True

    @classmethod
    def from_config_file(cls, config_path: Path) -> 'TeraSimToCosmosConverter':
        """
        Create converter instance from configuration file.

        Args:
            config_path: Path to YAML configuration file

        Returns:
            TeraSimToCosmosConverter instance
        """
        return cls(config_path=config_path)

    @classmethod
    def from_config_dict(cls, config_dict: dict) -> 'TeraSimToCosmosConverter':
        """
        Create converter instance from configuration dictionary.

        Args:
            config_dict: Configuration dictionary

        Returns:
            TeraSimToCosmosConverter instance
        """
        return cls(config_dict=config_dict)