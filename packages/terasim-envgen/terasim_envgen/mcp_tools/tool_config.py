"""
Configuration for automatic MCP tool registration
Maps core modules to MCP tools
"""

TOOL_MAPPINGS = {
    "map_searcher": {
        "module": "terasim_envgen.core.map_searcher",
        "class": "MapSearcher",
        "methods": {
            "search_roads": {
                "description": "Search for roads in specified city using OpenStreetMap data",
                "parameters": {
                    "city": {"type": "string", "description": "City name to search for maps"},
                    "road_types": {"type": "array", "description": "List of road types (highway, roundabout, etc.)", "optional": True},
                    "bbox_size": {"type": "number", "description": "Bounding box size in meters", "default": 500},
                    "max_samples": {"type": "number", "description": "Maximum number of map samples", "default": 5},
                    "save_plots": {"type": "boolean", "description": "Whether to save preview plots", "default": True}
                }
            },
            "find_junction": {
                "description": "Find road junctions/intersections in specified city",
                "parameters": {
                    "city": {"type": "string", "description": "City name to search"},
                    "filters": {"type": "object", "description": "Junction filtering criteria", "optional": True},
                    "max_samples": {"type": "number", "description": "Maximum number of samples", "default": 5},
                    "save_plots": {"type": "boolean", "description": "Whether to save preview plots", "default": True}
                }
            },
            "get_maps_through_route": {
                "description": "Generate maps along a specific route between two points",
                "parameters": {
                    "origin": {"type": "string", "description": "Starting point address"},
                    "destination": {"type": "string", "description": "Ending point address"},
                    "mode": {"type": "string", "description": "Transportation mode", "default": "driving"},
                    "target_split_distance": {"type": "number", "description": "Target distance between map segments", "default": 2000},
                    "bbox_size": {"type": "number", "description": "Bounding box size in meters", "default": 1000}
                }
            }
        },
        "init_args": {"config_path": "test_config.yaml"}
    },
    
    "map_converter": {
        "module": "terasim_envgen.core.map_converter",
        "class": "MapConverter",
        "methods": {
            "convert": {
                "description": "Convert OSM maps to simulation formats (SUMO, OpenDRIVE, Lanelet2)",
                "parameters": {
                    "osm_path": {"type": "string", "description": "Path to OSM file"},
                    "scene_id": {"type": "string", "description": "Scene identifier", "default": "default"},
                    "scenario_name": {"type": "string", "description": "Scenario name", "default": "autonomous_driving"}
                }
            }
        },
        "init_args": {"config_path": "config/config.yaml"}
    },
    
    "traffic_generator": {
        "module": "terasim_envgen.core.traffic_flow_generator",
        "class": "TrafficFlowGenerator",
        "methods": {
            "generate_flows": {
                "description": "Generate realistic traffic flows for SUMO simulation",
                "parameters": {
                    "net_path": {"type": "string", "description": "Path to SUMO network file"},
                    "end_time": {"type": "number", "description": "Simulation end time in seconds", "default": 3600},
                    "traffic_level": {"type": "string", "description": "Traffic density level (low/medium/high)", "default": "medium"},
                    "vehicle_types": {"type": "array", "description": "Types of vehicles to generate", "default": ["vehicle"]}
                }
            },
            "generate_multi_level_flows": {
                "description": "Generate traffic flows for multiple density levels",
                "parameters": {
                    "base_dir": {"type": "string", "description": "Base directory containing map files"},
                    "levels": {"type": "array", "description": "Traffic levels to generate", "default": ["low", "medium", "high"]}
                }
            }
        },
        "init_args": {"config_path": "config/config.yaml"}
    },
    
    "corner_case_generator": {
        "module": "terasim_envgen.core.terasim_corner_case_generator",
        "class": "TerSimCornerCaseGenerator",
        "methods": {
            "parse_adversities": {
                "description": "Parse adversities string into structured format",
                "parameters": {
                    "adversities_str": {"type": "string", "description": "Adversity types (e.g., 'vehicle:highway_cutin;vru:jaywalking')"}
                }
            }
        },
        "init_args": {}
    },
    
    "llm_parser": {
        "module": "terasim_envgen.core.llm_parser",
        "class": "LLMParser",
        "methods": {
            "parse_scenario_description": {
                "description": "Parse natural language description into structured simulation parameters",
                "parameters": {
                    "description": {"type": "string", "description": "Natural language description of simulation requirements"}
                }
            }
        },
        "init_args": {}
    }
}

# Tool categories for better organization
TOOL_CATEGORIES = {
    "map": ["map_searcher", "map_converter"],
    "traffic": ["traffic_generator"],
    "scenarios": ["corner_case_generator"],
    "parsing": ["llm_parser"]
}