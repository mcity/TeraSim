import argparse
import hydra
from loguru import logger
from omegaconf import DictConfig, OmegaConf
from pathlib import Path
from terasim.logger.infoextractor import InfoExtractor
from terasim.simulator import Simulator

from terasim_nde_nade.envs import NADE, NADEWithAV
from terasim_nde_nade.vehicle import NDEVehicleFactory
from terasim_nde_nade.vru import NDEVulnerableRoadUserFactory

# Import resolve_config_paths function
from terasim_service.utils.base import resolve_config_paths

# Add packages directory to sys path if needed
# sys.path.append(str(Path(__file__).resolve().parent.parent))


def main(config_path: str) -> None:
    config = OmegaConf.load(config_path)
    # Convert OmegaConf to dict for path resolution
    config_dict = OmegaConf.to_container(config, resolve=True)
    # Resolve all paths in config
    config_dict = resolve_config_paths(config_dict, config_path)

    # Convert back to OmegaConf for attribute access
    config = OmegaConf.create(config_dict)

    base_dir = Path(config.output.dir) / config.output.name / "raw_data" / config.output.nth
    base_dir.mkdir(parents=True, exist_ok=True)
    env = NADEWithAV( # NADEWithAV or NADE
        av_cfg = config.environment.parameters.AV_cfg,
        vehicle_factory=NDEVehicleFactory(cfg=config.environment.parameters),
        vru_factory=NDEVulnerableRoadUserFactory(cfg=config.environment.parameters),
        info_extractor=InfoExtractor,
        log_flag=True,
        log_dir=base_dir,
        warmup_time_lb=config.environment.parameters.warmup_time_lb,
        warmup_time_ub=config.environment.parameters.warmup_time_ub,
        run_time=1200,
        configuration=config.environment.parameters,
        av_debug_control=True, # Enable debug control for AV, will use SUMO
    )

    # Paths already resolved in config
    sumo_net_file = config.input.sumo_net_file
    sumo_config_file = config.input.sumo_config_file

    sim = Simulator(
        sumo_net_file_path=sumo_net_file,
        sumo_config_file_path=sumo_config_file,
        num_tries=10,
        gui_flag=config.simulator.parameters.gui_flag,
        realtime_flag=config.simulator.parameters.realtime_flag,
        output_path=base_dir,
        sumo_output_file_types=config.simulator.parameters.sumo_output_file_types,
        traffic_scale=config.simulator.parameters.traffic_scale if hasattr(config.simulator.parameters, "traffic_scale") else 1,
        additional_sumo_args=[
            "--device.bluelight.explicit",
            "true",
        ],
    )
    sim.bind_env(env)

    terasim_logger = logger.bind(name="terasim_nde_nade")
    terasim_logger.info(f"terasim_nde_nade: Experiment started")

    sim.run()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run TeraSim debug experiments with YAML configuration")
    parser.add_argument(
        "--config",
        type=str,
        default="configs/simulation/test.yaml",
        help="Path to YAML configuration file (default: configs/simulation/test.yaml)"
    )

    args = parser.parse_args()

    # Convert to Path object and validate
    config_path = Path(args.config)
    if not config_path.exists():
        logger.error(f"Configuration file not found: {config_path}")
        exit(1)

    if not config_path.suffix.lower() in ['.yaml', '.yml']:
        logger.error(f"Configuration file must be a YAML file: {config_path}")
        exit(1)

    logger.info(f"Running experiment with config: {config_path}")
    main(str(config_path))
