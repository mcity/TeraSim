<!-- PROJECT LOGO -->
<p align="center">
  <h3 align="center">TeraSim</h3>
  <p align="center">
    An autonomous vehicle testing and training platform based on SUMO
    <br />
    <a href="https://github.com/michigan-traffic-lab/TeraSim">View Demo</a>
    ·
    <a href="https://github.com/michigan-traffic-lab/TeraSim/issues">Report Bug</a>
    ·
    <a href="https://github.com/michigan-traffic-lab/TeraSim/issues">Request Feature</a>
  </p>
</p>


<!-- ABOUT THE PROJECT -->
# Introduction

## About
This project aims at providing a microsim simulation environment based on SUMO. With APIs provided, users can easily build vehicle model and apply test on the simulation platform.

## Code Structure

- docs: extra documentations on this repo
- examples: examples of using terasim to run driving simulations of vehicle models
- terasim: main contents
  - envs: uses functions from 'vehicle' and create an integrated driving environment with multiple vehicles in which the given autonomous vehicles are tested
  - logger: tracks the information in the environment
  - measure: measures to take along with simulations
  - network: transforms the maps to SUMO API to be used by simulator
  - vehicle: contains the 'sensors', 'controllers' and 'decision_models' of a vehicle, and uses 'factories' to combine all three parts and creat a vehicle.
  - configs.py: contains configurations
  - simulator.py: applies SUMO to our environment and run simulations
  - utils.py: utility functions
- setup.py: required packages to run this repo

![Architecture](docs/figure/Simulation_Platform_Architecture.svg)
<!-- GETTING STARTED -->

# Installation
The project is developed and tested on Ubuntu 22.04 LTS. Therefore, we recommend running the following process on Ubuntu 22.04 LTS. You are recommended to create a virtual environment to install the simulation platform.

## Prerequisites

Recommended environment:
- Python 3.10
- SUMO 1.19.0

Minimum versioned environment:
- Python 3.10
- SUMO 1.19.0

We recommend using Poetry to manage dependencies and create a virtual environment for terasim.
```bash
# Install Poetry if you haven't already
curl -sSL https://install.python-poetry.org | python3 -
```

The simulation requires map files in SUMO format (.net.xml). Please refer to '/examples' for detailed guidelines.

## Download terasim
- Download from Github: `git clone https://github.com/michigan-traffic-lab/TeraSim.git`

## Install TeraSim
Navigate to the directory of the project (`cd TeraSim`), and then

```bash
# Install dependencies and create virtual environment
poetry install
```

## Commonly Seen Errors

After the installation, if you encounter the following error and the SUMO gui does not show up:
```bash
libGL error: failed to load driver: swrast
X Error: code 2 major 152 minor 3: BadValue (integer parameter out of range for operation).
```
You can run the following command to fix the error (using Anaconda):
```bash
conda install -c conda-forge libstdcxx-ng
```

<!-- USAGE EXAMPLES -->
# Usage

The package consists of multiple classes, including simulator, environment, vehicle and controller. for basic usage, we only need the Simulator class and the Environment class. For example, to build a simulation with one dummy AV running in a 3lane highway, we neeed the following script:

```python
from pathlib import Path
from terasim.simulator import Simulator
from terasim.envs.template import EnvTemplate
from terasim.logger.infoextractor import InfoExtractor
from terasim.vehicle.factories.vehicle_factory import VehicleFactory
from terasim.vehicle.sensors.ego import EgoSensor
from terasim.vehicle.sensors.local import LocalSensor
from terasim.vehicle.controllers.high_efficiency_controller import (
    HighEfficiencyController,
)
from terasim.vehicle.vehicle import Vehicle
from terasim.vehicle.decision_models.idm_model import IDMModel

current_path = Path(__file__).parent
maps_path = current_path / "maps" / "3LaneHighway"


class ExampleVehicleFactory(VehicleFactory):

    def create_vehicle(self, veh_id, simulator):
        """Generate a vehicle with the given vehicle id in the simulator, composed of a decision model, a controller, and a list of sensors, which should be defined or customized by the user.

        Args:
            veh_id (_type_): vehicle id
            simulator (_type_): simulator (sumo)

        Returns:
            Vehicle: the contructed vehicle object
        """
        sensor_list = [EgoSensor(), LocalSensor(obs_range=40)]
        # decision_model = DummyDecisionModel(mode="random")  # mode="random" "constant"
        decision_model = IDMModel(MOBIL_lc_flag=False, stochastic_acc_flag=True)
        control_params = {
            "v_high": 40,
            "v_low": 20,
            "acc_duration": 0.1,  # the acceleration duration will be 0.1 second
            "lc_duration": 1,  # the lane change duration will be 1 second
        }
        controller = HighEfficiencyController(simulator, control_params)
        return Vehicle(
            veh_id,
            simulator,
            sensors=sensor_list,
            decision_model=decision_model,
            controller=controller,
        )


env = EnvTemplate(vehicle_factory=ExampleVehicleFactory(), info_extractor=InfoExtractor)
sim = Simulator(
    sumo_net_file_path=maps_path / "3LaneHighway.net.xml",
    sumo_config_file_path=maps_path / "3LaneHighway.sumocfg",
    num_tries=10,
    gui_flag=True,
    output_path=current_path / "output" / "0",
    sumo_output_file_types=["fcd_all"],
)
sim.bind_env(env)
sim.run()

```

In the script, we build the environment, the simulator, and bind them together. Then we run the simulator, the sumo interface will show up. After clicking the "run" button in the sumo-gui, the simulation will run with one autonomous vehicle(CAV) and multiple background vehicles(BVs).

# Specific Tutorials
For the detailed tutorials, please refer to the following:
- [Traffic light simulation](docs/tutorials/traffic_light_simulation.md)

<!-- CONTRIBUTING -->
# Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

# Credits

## Developer

Haowei Sun: haoweis@umich.edu
Haojie Zhu: zhuhj@umich.edu

## Reviewer

Haojie Zhu: zhuhj@umich.edu

## License

Distributed under the MIT License.

## Contact

- Haowei Sun - haoweis@umich.edu - Michigan Traffic Lab
- Haojie Zhu - zhuhj@umich.edu - Michigan Traffic Lab
- Shuo Feng - fshuo@umich.edu - Michigan Traffic Lab
- Henry Liu - henryliu@umich.edu - Michigan Traffic Lab

Project Link: [https://github.com/michigan-traffic-lab/TeraSim](https://github.com/michigan-traffic-lab/TeraSim)
