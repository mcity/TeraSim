#!/usr/bin/env python3
"""
Traffic generation script for TeraSim
Generates traffic flows for SUMO network files
"""
import os
import sys

# Add SUMO tools to Python path to fix sumolib import in subprocess
sumo_home = os.getenv("SUMO_HOME", "/home/sdai/.terasim/deps/sumo")
sumo_tools = os.path.join(sumo_home, "tools")
if sumo_tools not in sys.path:
    sys.path.insert(0, sumo_tools)

# Set PYTHONPATH environment variable so subprocess calls can also import sumolib
os.environ["PYTHONPATH"] = f"{sumo_tools}:{os.environ.get('PYTHONPATH', '')}"

from terasim_envgen.core.traffic_flow_generator import TrafficFlowGenerator

# Initialize traffic generator
traffic_gen = TrafficFlowGenerator("configs/config.yaml")

# Generate traffic for single map
print("Generating traffic flows for jupiter_eb.net.xml...")
routes_file = traffic_gen.generate_flows(
    net_path="/home/sdai/harry/TeraSim/jupiter/eb/eb.net.xml",
    end_time=3600,
    traffic_level="high",
    vehicle_types=["vehicle"]
)

if routes_file:
    print(f"\n✅ Traffic generation complete!")
    print(f"   Routes file: {routes_file}")
else:
    print("\n⚠️  Traffic generation completed with fallback routes")

# Note: generate_multi_level_flows() removed because it searches for map.net.xml
# which doesn't exist in this directory. Use generate_flows() above for specific network files.