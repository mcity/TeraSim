from terasim_envgen.core.traffic_flow_generator import TrafficFlowGenerator

# Initialize traffic generator
traffic_gen = TrafficFlowGenerator("configs/config.yaml")

# Generate traffic for single map
routes_file = traffic_gen.generate_flows(
    net_path="/home/sdai/harry/TeraSim/jupiter/eb/jupiter_eb.net.xml",
    end_time=3600,
    traffic_level="medium",  
    vehicle_types=["vehicle", "pedestrian", "bicycle"]
)

# Batch generate for all maps
traffic_gen.generate_multi_level_flows("/home/sdai/harry/TeraSim/jupiter/eb/")