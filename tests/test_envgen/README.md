# TeraSim Configuration System

This document explains how to use the separated adversity configuration system.

## Configuration File Structure

The configuration files have been separated into individual YAML files for different types of adversities while maintaining the original nested structure:

```
conf/
  ├── config.yaml                      # Main configuration file
  └── adversity/                      # Adversity configuration directory
      ├── vehicle/                    # Vehicle-related adversity
      │   ├── highway_merge.yaml
      │   ├── highway_cutin_abort.yaml
      │   ├── highway_cutin.yaml
      │   ├── highway_rearend_decel.yaml
      │   └── highway_rearend_accel.yaml
      ├── vulnerable_road_user/       # Pedestrian-related adversity
      │   └── jaywalking.yaml
      └── static/                     # Static object-related adversity
          ├── construction.yaml
          ├── collision.yaml
          └── stalled_object.yaml
```

Each configuration file maintains the original nested structure, for example:

```yaml
# Vehicle type configuration
adversity_cfg:
  vehicle:
    highway_merge:
      _target_: terasim_nde_nade.adversity.vehicles.MergeAdversity
      # ... other parameters

# Pedestrian type configuration
adversity_cfg:
  vulnerable_road_user:
    jaywalking:
      _target_: terasim_nde_nade.adversity.vru.JaywalkingAdversity
      # ... other parameters

# Static object type configuration
adversity_cfg:
  static:
    construction:
      _target_: terasim_nde_nade.adversity.ConstructionAdversity
      # ... other parameters
```

## Usage Methods

### 1. Specify via Command Line Arguments

You can use command line arguments to specify different types of adversity:

```bash
# Use only vehicle adversity
python terasim_corner_case_generator.py --vehicle_adversity highway_cutin

# Combine multiple adversity types
python terasim_corner_case_generator.py --vehicle_adversity highway_merge --vru_adversity jaywalking

# Use all types of adversities
python terasim_corner_case_generator.py \
    --vehicle_adversity highway_cutin \
    --vru_adversity jaywalking \
    --static_adversity collision
```

### 2. Specify via Hydra Configuration System

You can also override the default configuration through Hydra's configuration system:

```bash
# Use a different vehicle adversity configuration
python terasim_corner_case_generator.py adversity/vehicle=highway_cutin_abort

# Override specific parameters
python terasim_corner_case_generator.py adversity_cfg.vehicle.highway_merge.probability=0.01

# Combine multiple configurations
python terasim_corner_case_generator.py \
    adversity/vehicle=highway_cutin \
    adversity/vulnerable_road_user=jaywalking
```

### 3. Dynamic Combination in Code

You can also dynamically combine different adversity configurations in your Python code:

```python
from omegaconf import OmegaConf

# Load base configuration
base_cfg = OmegaConf.load('conf/config.yaml')

# Load adversity configurations
vehicle_cfg = OmegaConf.load('conf/adversity/vehicle/highway_cutin.yaml')
vru_cfg = OmegaConf.load('conf/adversity/vulnerable_road_user/jaywalking.yaml')

# Merge configurations
cfg = OmegaConf.merge(base_cfg, vehicle_cfg, vru_cfg)

# Use the merged configuration to run simulation
# ...
```

## Adding New Adversity Configurations

If you want to add a new adversity configuration, you need to create a new YAML file according to the corresponding nested structure. For example:

```yaml
# conf/adversity/vehicle/my_new_adversity.yaml
adversity_cfg:
  vehicle:
    my_new_adversity:
      _target_: terasim_nde_nade.adversity.vehicles.MyNewAdversity
      _convert_: 'all'
      location: 'highway'
      ego_type: 'vehicle'
      probability: 0.001
      predicted_collision_type: "my_collision_type"
```

Then you can use this new configuration via command line:

```bash
python terasim_corner_case_generator.py --vehicle_adversity my_new_adversity
``` 