from pathlib import Path

from loguru import logger
from omegaconf import DictConfig, OmegaConf
from terasim.logger.infoextractor import InfoExtractor
from terasim.simulator import Simulator

from terasim_nde_nade.envs import NADEWithAV, NADE
from terasim_nde_nade.vehicle import NDEVehicleFactory
from terasim_nde_nade.vru import NDEVulnerableRoadUserFactory
import shutil
import json
from typing import Tuple, Optional

class TerSimCornerCaseGenerator:
    """
    Class to handle TeraSim corner case generation experiments
    """
    def __init__(self, config_path: str = "src/core/conf", config_name: str = "config"):
        self.config_path = config_path
        self.config_name = config_name
    
    def parse_adversities(self, adversities_str: str) -> dict:
        """Parse adversities string into dictionary format
        
        Args:
            adversities_str: String in format "category1:adv1,adv2;category2:adv3,adv4"
            
        Returns:
            Dictionary of selected adversities by category
        """
        if not adversities_str:
            return {}
            
        selected = {}
        categories = adversities_str.split(';')
        for category in categories:
            if ':' not in category:
                continue
            cat_name, adv_list = category.split(':')
            selected[cat_name] = [adv.strip() for adv in adv_list.split(',')]
        return selected

    def load_config_with_adversities(self, base_config: DictConfig, selected_adversities: dict) -> DictConfig:
        """Load configuration with selected adversities
        
        Args:
            base_config: Base configuration loaded by hydra
            selected_adversities: Dictionary of selected adversities by category
            
        Returns:
            Updated configuration with selected adversities
        """
        # Convert DictConfig to dict for manipulation
        config_dict = OmegaConf.to_container(base_config, resolve=True)
        
        # Initialize adversity config if not exists
        if 'adversity_cfg' not in config_dict:
            config_dict['adversity_cfg'] = {}
        
        # Load selected adversities
        for category, adversities in selected_adversities.items():
            if category not in config_dict['adversity_cfg']:
                config_dict['adversity_cfg'][category] = {}
                
            for adversity in adversities:
                if category == "static":
                    file_path = Path(base_config.road_path) / f"{adversity}.yaml"
                else:
                    file_path = Path("src/core/conf/adversity") / category / f"{adversity}.yaml"
                if file_path.exists():
                    adversity_config = OmegaConf.load(file_path)
                    # Extract the specific adversity config
                    if 'adversity_cfg' in adversity_config and category in adversity_config['adversity_cfg']:
                        config_dict['adversity_cfg'][category].update(
                            adversity_config['adversity_cfg'][category]
                        )
                else:
                    logger.error(f"Adversity config file {file_path} not found")
        
        # Convert back to DictConfig
        return OmegaConf.create(config_dict)
    
    def load_base_config(self, road_path: str, adversities: str, output_folder_name: str) -> DictConfig:
        """Load base configuration from hydra config files
        
        Args:
            road_path: Path to the road directory
            adversities: Adversity string
            output_folder_name: Name of the output folder
            
        Returns:
            Loaded configuration
        """
        # Create a temporary config override
        config_overrides = [
            f"road_path={road_path}",
            f"adversities={adversities}",
            f"output_folder_name={output_folder_name}"
        ]
        
        # Load base config using OmegaConf directly
        try:
            config_file = Path(self.config_path) / f"{self.config_name}.yaml"
            if config_file.exists():
                cfg = OmegaConf.load(config_file)
            else:
                # Create a minimal config if file doesn't exist
                cfg = OmegaConf.create({})
            
            # Apply overrides
            cfg.road_path = road_path
            cfg.adversities = adversities
            cfg.output_folder_name = output_folder_name
                
            return cfg
        except Exception as e:
            logger.error(f"Error loading config: {e}")
            # Return minimal config
            return OmegaConf.create({
                'road_path': road_path,
                'adversities': adversities,
                'output_folder_name': output_folder_name,
                'sumo_net_file_path': str(Path(road_path) / "map.net.xml"),
                'sumo_cfg_file_path': str(Path(road_path) / "simulation.sumocfg")
            })
    
    def run_experiment(self, road_path: str, adversity_type: str, output_folder_name: str) -> bool:
        """
        Run a single experiment with the given parameters.
        
        Args:
            road_path: Path to the road directory
            adversity_type: Type of adversity to test
            output_folder_name: Name of the output folder
            
        Returns:
            bool: Success flag
        """
        try:
            print(f"\nTesting adversity: {adversity_type} for {road_path}")
            
            # Load configuration
            cfg = self.load_base_config(road_path, adversity_type, output_folder_name)
            
            # Get output directory
            output_dir = Path(cfg.road_path)
            print("\033[91m" + str(output_dir) + "\033[0m")
            
            # Create base directory for logs
            base_dir = output_dir / "simulation_output" / cfg.output_folder_name
            base_dir.mkdir(parents=True, exist_ok=True)
            
            # Setup logging
            log_files = [base_dir / "loguru_run.log"]
            log_levels = ["TRACE"]
            
            for log_file, log_level in zip(log_files, log_levels):
                logger.add(
                    log_file,
                    level=log_level,
                    enqueue=True,
                    backtrace=True,
                    serialize=True,
                )
            
            # Parse adversities from config
            selected_adversities = {}
            if hasattr(cfg, 'adversities'):
                selected_adversities = self.parse_adversities(cfg.adversities)
            
            # Load configuration with selected adversities
            if selected_adversities:
                cfg = self.load_config_with_adversities(cfg, selected_adversities)
            
            # Create environment
            env = NADE(
                # av_cfg=cfg.AV_cfg,
                vehicle_factory=NDEVehicleFactory(cfg=cfg),
                vru_factory=NDEVulnerableRoadUserFactory(cfg=cfg),
                info_extractor=InfoExtractor,
                log_flag=True,
                log_dir=base_dir,
                warmup_time_lb=150,
                warmup_time_ub=200,
                run_time=200,
                configuration=cfg,
            )

            # Create simulator
            sim = Simulator(
                sumo_net_file_path=cfg.sumo_net_file_path,
                sumo_config_file_path=cfg.sumo_cfg_file_path,
                num_tries=10,
                gui_flag=False,
                realtime_flag=False,
                output_path=base_dir,
                sumo_output_file_types=["fcd_all", "collision", "tripinfo"],
            )
            sim.bind_env(env)

            terasim_logger = logger.bind(name="terasim_nde_nade")
            terasim_logger.info("terasim_nde_nade: Experiment started")
            terasim_logger.info(f"Selected adversities: {selected_adversities}")

            # Run simulation
            sim.run()
            
            print(f"Successfully completed experiment for {road_path} with {adversity_type}")
            return True
            
        except Exception as e:
            logger.exception(f"terasim_nde_nade: Running error caught: {e}")
            print(f"Error running experiment for {road_path} with {adversity_type}: {e}")
            return False

def run_single_experiment(params: Tuple[str, str, str]) -> Tuple[str, str, bool]:
    """
    Run a single experiment with the given parameters using the class-based approach.
    
    Args:
        params: Tuple containing (road_path, adversity_type, output_folder_name)
    
    Returns:
        Tuple[str, str, bool]: (road_path, adversity_type, success_flag)
    """
    road_path, adversity_type, output_folder_name = params
    
    print(f"\nTesting adversity: {adversity_type} for {road_path}")
    road_path_obj = Path(road_path)
    
    try:
        # Create generator instance and run experiment
        generator = TerSimCornerCaseGenerator()
        success = generator.run_experiment(road_path, adversity_type, output_folder_name)
        
        if not success:
            # Clean up failed experiment directory
            try:
                shutil.rmtree(road_path_obj / "simulation_output" / output_folder_name)
            except:
                pass
            return road_path, adversity_type, False

        # Check monitor.json for validation
        monitor_path = road_path_obj / "simulation_output" / output_folder_name / "monitor.json"
        try:
            # Load monitor.json and make sure "veh_1_id" is not null
            with open(monitor_path, "r") as f:
                monitor = json.load(f)
            if monitor["veh_1_id"] is None:
                # If no veh_id is there, then it is a failure and we should remove the folder
                shutil.rmtree(road_path_obj / "simulation_output" / output_folder_name)
                return road_path, adversity_type, False
        except Exception as e:
            print(f"Error loading monitor.json for {road_path} with {adversity_type}: {e}")
            try:
                shutil.rmtree(road_path_obj / "simulation_output" / output_folder_name)
            except:
                pass
            return road_path, adversity_type, False
        
        return road_path, adversity_type, True
        
    except Exception as e:
        print(f"Error running experiment for {road_path} with {adversity_type}: {e}")
        # Clean up failed experiment directory
        try:
            shutil.rmtree(road_path_obj / "simulation_output" / output_folder_name)
        except:
            pass
        return road_path, adversity_type, False

if __name__ == "__main__":
    # Example usage:
    run_single_experiment((
        "path/to/road/directory", 
        "vehicle:intersection_tfl", 
        "intersection_tfl_0"
    ))
