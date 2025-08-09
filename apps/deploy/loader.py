from pathlib import Path
from omegaconf import OmegaConf
import os
import yaml


class ConfigLoader:
    def __init__(self, project_root=None):
        """
        Initialize configuration loader
        
        Args:
            project_root (Path, optional): Project root directory path. If None, will be auto-detected.
        """
        self.project_root = project_root
        self.config_dir = self.project_root / "configs"
        self.examples_dir = self.project_root / "examples"
    
    def load_config(self, map_path, behavior_adversity_list=None, static_adversity_list=None):
        """
        Load and merge configurations
        
        Args:
            map_id (str): Map ID
            adversity_name (str, optional): Specific scenario name
            
        Returns:
            OmegaConf: Merged configuration
        """
        # 1. Load base configurations
        base_config = OmegaConf.load(self.config_dir / "base.yaml")
        env_config = OmegaConf.load(self.config_dir / "environment.yaml")
        av_config = OmegaConf.load(self.config_dir / "av_config.yaml") # This is the default AV config and can be overrided by the map-specific config
        
        map_av_config_path = map_path / "config" / "av_config.yaml"
        if map_av_config_path.exists():
            map_av_config = OmegaConf.load(map_av_config_path)
            av_config = OmegaConf.merge(av_config, map_av_config)
        
        # 4. Set map file paths
        input_cfg = {
            "input": {
                "sumo_net_file": str(map_path / "map.net.xml"),
                "sumo_config_file": str(map_path / "sumo_medium.sumocfg")
            }
        }
        input_config = OmegaConf.create(input_cfg)
        
        # 5. Merge base configurations
        config = OmegaConf.merge(
            base_config,
            env_config,
            av_config,
            input_config
        )
        
        # 6. Load scenario config if specified
        if static_adversity_list:
            for static_adversity in static_adversity_list:
                adv_path = map_path / "config" / "adversities" / f"{static_adversity}.yaml"
                if adv_path.exists():
                    adv_config = OmegaConf.load(adv_path)
                # Merge adversity config under environment.parameters
                if "environment" not in config:
                    config.environment = {}
                if "parameters" not in config.environment:
                    config.environment.parameters = {}
                config.environment.parameters = OmegaConf.merge(config.environment.parameters, adv_config)
        if behavior_adversity_list:
            for behavior_adversity in behavior_adversity_list:
                adv_path = self.config_dir / "adversities" / "vehicle" / f"{behavior_adversity}.yaml"
                if adv_path.exists():
                    adv_config = OmegaConf.load(adv_path)
                # Merge adversity config under environment.parameters
                if "environment" not in config:
                    config.environment = {}
                if "parameters" not in config.environment:
                    config.environment.parameters = {}
                config.environment.parameters = OmegaConf.merge(config.environment.parameters, adv_config)
        return config
    
    def save_config(self, config, output_path):
        """
        Save configuration to file
        
        Args:
            config (OmegaConf): Configuration object
            output_path (str): Output file path
        """
        with open(output_path, "w") as f:
            yaml.dump(OmegaConf.to_container(config), f)


# Usage example
if __name__ == "__main__":
    # Initialize config loader
    config_loader = ConfigLoader()
    
    # Load base config (map 0)
    base_config = config_loader.load_config("0")
    
    # Load specific scenario config
    # highway_config = config_loader.load_config("0", "highway_1")
    
    # Save config file
    config_loader.save_config(base_config, "dynamic_config.yaml") 