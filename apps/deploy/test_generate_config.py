#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Example script for generating configuration
"""

from loader import ConfigLoader
from pathlib import Path


project_root = Path(__file__).resolve().parent
config_loader = ConfigLoader(project_root)

map_path = # Path to your map

# Load configuration
config = config_loader.load_config(map_path, ["highway_cutin", "highway_rearend", "highway_rearend_decel"], ["stalledvehicle"])

# Save configuration
config_loader.save_config(config, "test_config.yaml")
print(f"Configuration saved to: test_config.yaml")