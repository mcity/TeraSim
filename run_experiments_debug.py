import argparse
import random
import hydra
from loguru import logger
from omegaconf import DictConfig, OmegaConf
from pathlib import Path
from tqdm import tqdm
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
    env = NADEWithAV(
        av_cfg=config.environment.parameters.AV_cfg,
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
    # sumo_additional_file = config.input.sumo_additional_file
    sumo_additional_file = "./vTypeDistributions.add.xml"

    sim = Simulator(
        sumo_net_file_path=sumo_net_file,
        sumo_config_file_path=sumo_config_file,
        sumo_additional_file_path=sumo_additional_file,
        num_tries=10,
        gui_flag=config.simulator.parameters.gui_flag,
        realtime_flag=config.simulator.parameters.realtime_flag,
        output_path=base_dir,
        sumo_output_file_types=["collision"],
        traffic_scale=(
            config.simulator.parameters.traffic_scale
            if hasattr(config.simulator.parameters, "traffic_scale")
            else 1
        ),
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
    # Get all yaml files in examples/scenarios directory
    config_dir = Path(__file__).parent / "examples" / "scenarios"
    # yaml_files = sorted(config_dir.glob("*.yaml"), key=lambda x: int(''.join(filter(str.isdigit, x.stem)) or '0'))
    # yaml_files = ["examples/scenarios/cutin.yaml"]
    yaml_files = [Path("/home/sdai/harry/TeraSim/jupiter/eb/test_config.yaml")]
    # Randomly shuffle yaml files
    random.shuffle(yaml_files)

    # Run experiments for each yaml file
    for yaml_file in tqdm(yaml_files):
        print(yaml_file)
        logger.info(f"Running experiment with config: {yaml_file}")
        main(str(yaml_file))
        # try:
        #     main(str(yaml_file))
        # except Exception as e:
        #     logger.error(f"Error running {yaml_file}: {e}")
        #     # yaml_file.unlink()  # Delete the yaml file
        # continue
