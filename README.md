<div align="center">
<p align="center">

<img src="docs/figure/logo.png" height="100px">

</p>
</div>


---

# TeraSim

**Generative AI–Driven Autonomous Vehicle Simulation for Unknown Unsafe Events Discovery**

---

## Overview

TeraSim is an open-source platform for automated autonomous-vehicle (AV) simulation using generative AI.
Its primary objective is to **efficiently uncover real-world unknown unsafe events** by automatically creating diverse and statistically realistic traffic environments.

The framework has evolved from its initial focus on planning-and-control testing to a **complete simulation workflow**, which now includes:

1. **High-fidelity HD map generation** for large-scale, accurate simulation environments
2. **Generative traffic environment creation** for naturalistic and adversarial scenario testing
3. **Generative sensor simulation** for camera and LiDAR perception validation

This expanded scope enables a unified pipeline from map generation to perception and planning validation.

## 🚀 **Updates**

- **[09/29/2025]**: TeraSim-World source codes are available. See [TeraSim_World.md](docs/TeraSim_World.md) to get started.



## **🌎 New Feature: TeraSim-World**


<img src="docs/figure/TeraSim_World.png" height="400px">


**TeraSim-World** automatically synthesizes geographically grounded, safety-critical data for End-to-End autonomous driving **anywhere in the world**. See 📄 [paper](https://arxiv.org/abs/2503.03629) and 🌐 [videos](https://wjiawei.com/terasim-world-web/) for details.

✨ **Key Capabilities:**
- 🗺️ **Global Coverage**: Generate realistic driving scenarios for any location worldwide
- 🎯 **Safety-Critical Data**: Automatically create safety-critical events for E2E AV safety testing
- 🔄 **NVIDIA Cosmos-Drive Compatible**: Direct integration with video generation model training platforms

🚀 **Source code is now available!** See [TeraSim_World.md](docs/TeraSim_World.md) for getting started guide.

---

## Key Capabilities

### 1. High-Fidelity HD Map Generation

* Tools for building **city-scale, high-resolution digital twins** suitable for AV testing.
* Automated conversion of real-world survey data into simulation-ready HD maps.
* Provides accurate lane geometry and traffic-control metadata for downstream simulations.

### 2. Generative Traffic Environment Creation

* Automated scenario generation based on **large-scale naturalistic driving data**.
* **Adversarial scenario synthesis** to reveal rare or high-risk interactions (e.g., aggressive cut-ins, unexpected pedestrian crossings).
* Integration with [SUMO](https://www.eclipse.org/sumo/) and third-party simulators such as [CARLA](https://carla.org/) and Autoware.

### 3. Generative Sensor Simulation

* **`terasim-cosmos`** integrates TeraSim-World with **generative AI–based camera and LiDAR simulation**.
* Enables perception validation and sensor pipeline testing under diverse conditions.
* **Ongoing work:** support for fully **custom sensor models and configurable realism levels** is under active development.

---

## System Architecture

TeraSim uses a modular monorepo design. Each package can be used independently or combined into a complete simulation pipeline.

```
TeraSim/
├── packages/
│   ├── terasim/            # Core simulation engine
│   ├── terasim-envgen/     # HD map and environment generation
│   ├── terasim-nde-nade/   # Naturalistic & adversarial environment algorithms
│   ├── terasim-cosmos/     # TeraSim-World integration & generative AI sensor simulation
│   ├── terasim-sensor/     # Baseline sensor utilities
│   ├── terasim-datazoo/    # Data processing utilities for real driving datasets
│   ├── terasim-service/    # RESTful API for external simulators
│   └── terasim-vis/        # Visualization and analysis tools
├── examples/               # Example configurations and scenarios
├── docs/                   # Documentation and figures
└── tests/                  # Test suites
```

---

## Installation

### Quick Setup

```bash
git clone https://github.com/mcity/TeraSim.git
cd TeraSim
./setup_environment.sh
```

This script installs all required Python packages and dependencies, including [SUMO](https://www.eclipse.org/sumo/).

### Docker Installation (Recommended for Production)

For a containerized environment with all dependencies pre-installed:

```bash
git clone https://github.com/mcity/TeraSim.git
cd TeraSim
docker-compose up -d --build
docker-compose exec terasim bash
```

See [README_DOCKER.md](README_DOCKER.md) for detailed Docker deployment instructions.

### Development Environment

```bash
conda create -n terasim python=3.10
conda activate terasim
git clone https://github.com/mcity/TeraSim.git
cd TeraSim
./setup_environment.sh
```

**Requirements**

* Python 3.10–3.12
* SUMO 1.23.1 (installed by the setup script)
* Redis for service components
* gcc/g++ compilers (for Cython extensions)

---

## Quick Start Example

```python
from terasim import Simulator
from terasim.envs import EnvTemplate

sim = Simulator("examples/maps/Mcity/sim.sumocfg")
env = EnvTemplate()
sim.bind_env(env)

sim.start()
sim.run(steps=1000)
sim.close()
```

Additional examples are available in the [`examples/`](examples/) directory.

---

## Contributing

Contributions are welcome. Please read the [CONTRIBUTING.md](CONTRIBUTING.md) guidelines and join the [GitHub discussions](https://github.com/mcity/TeraSim/discussions) for feedback or proposals.

---


## Publications

Explore our other research on autonomous driving testing!

* **NDE** – Learning naturalistic driving environment with statistical realism
  [Paper](https://doi.org/10.1038/s41467-023-37677-5) | [Code](https://github.com/michigan-traffic-lab/Learning-Naturalistic-Driving-Environment)

* **NADE** – Intelligent driving intelligence test with naturalistic and adversarial environment
  [Paper](https://doi.org/10.1038/s41467-021-21007-8) | [Code](https://github.com/michigan-traffic-lab/Naturalistic-and-Adversarial-Driving-Environment)

* **D2RL** – Dense deep reinforcement learning for AV safety validation
  [Paper](https://doi.org/10.1038/s41586-023-05732-2) | [Code](https://github.com/michigan-traffic-lab/Dense-Deep-Reinforcement-Learning)

## **📄 License**

- **TeraSim Core and other packages**: Apache 2.0 License
- **Visualization Tools**: MIT License

This project includes modified code from [SumoNetVis](https://github.com/patmalcolm91/SumoNetVis) licensed under the MIT License.
