import os
import json
import random
from typing import Dict, List, TypedDict, Literal, Optional, Union
from anthropic import Anthropic
from dotenv import load_dotenv
from jsonschema import validate, ValidationError
from dotenv import load_dotenv
load_dotenv(override=True)

# Type definitions
class WeatherSettings(TypedDict):
    type: Literal["sunny", "rainy", "snowy", "foggy"]
    intensity: float

class MapType(TypedDict):
    primary_type: str  # Will be validated against ROAD_TYPES or JUNCTION_TYPES
    traffic_density: Literal["low", "medium", "high"]

class CornerCases(TypedDict):
    enabled: bool
    description: str  # Natural language description of corner cases

class SimulationSettings(TypedDict):
    weather: WeatherSettings
    time_of_day: Literal["day", "night", "dawn", "dusk"]
    sensors: List[Literal["camera", "lidar", "radar", "gps"]]

class SimulationConfig(TypedDict):
    map_type: MapType
    corner_cases: CornerCases
    simulation_settings: SimulationSettings
    cities: List[str]

# Road and junction type mappings
ROAD_TYPES = ["highway", "highway_ramp", "arterial", "collector", "local", "roundabout"]
JUNCTION_TYPES = ["signalized", "stop_sign", "t_intersection", "crossroads"]

# Default US cities if none provided
DEFAULT_US_CITIES = [
    "Ann Arbor, MI, USA",
    "San Francisco, CA, USA",
    "Austin, TX, USA",
    "New York, NY, USA",
    "Los Angeles, CA, USA",
    "Chicago, IL, USA",
    "Houston, TX, USA",
    "Phoenix, AZ, USA",
    "Philadelphia, PA, USA",
    "San Antonio, TX, USA",
    "San Diego, CA, USA",
    "Dallas, TX, USA",
    "San Jose, CA, USA"
]

ADVERSITY_TYPES = [
    "vehicle:intersection_headon",
    "vehicle:intersection_tfl",
    "vehicle:roundabout_cutin",
    "vehicle:roundabout_fail_to_yield",
    "vehicle:roundabout_rearend",
    "vehicle:highway_cutin_abort",
    "vehicle:highway_cutin",
    "vehicle:highway_rearend_accel",
    "vehicle:highway_rearend",
    "vehicle:intersection_rearend",
    "vehicle:intersection_cutin",
    "vehicle:highway_merge",
    "vehicle:highway_rearend_decel",
    "static:construction_zone",
]

json_schema = {
    "type": "object",
    "properties": {
        "cities": {"type": "array", "items": {"type": "string"}},
        "map_type": {"type": "string", "enum": ROAD_TYPES + JUNCTION_TYPES},
        "adversity_types": {"type": "array", "items": {"type": "string", "enum": ADVERSITY_TYPES}}
    }
}

class LLMParser:
    """
    Parser that converts natural language descriptions into structured simulation parameters.
    Uses Claude to understand user requirements and generate appropriate configuration.
    """

    def __init__(self):
        """Initialize the parser with Claude API client."""
        load_dotenv(override=True)
        api_key = os.getenv("CLAUDE_API_KEY")
        if not api_key:
            raise ValueError("CLAUDE_API_KEY not found in environment variables")
        self.client = Anthropic(api_key=api_key)

    def parse_scenario_description(self, description: str) -> Dict:
        """
        Convert natural language description into structured simulation parameters.
        """
        """
        Convert natural language description into structured simulation parameters.

        Args:
            description: Natural language description of desired simulation scenario

        Returns:
            Dict containing structured simulation parameters with the following fields:
            - cities: List of city names
            - map_type: One of the valid road or junction types
            - adversity_types: List of valid adversity types
        """
        # 1. Parse the description into a structured format as a JSON object
        prompt = f"""
        Convert the following natural language description into a structured JSON object.

        Required JSON schema:
        {{
            "traffic_density": "low", # "low" or "medium" or "high"
            "adversity_types": ["type1", "type2"]  # Types of challenging scenarios. Must be from: {ADVERSITY_TYPES}
        }}

        Important rules:
        1. Return ONLY the JSON object, with no additional text
        2. For any field that is not clearly specified in the description, use an empty list []
        3. Make sure map_type is exactly one of the valid types listed above
        4. Make sure all adversity_types are from the valid list above
        5. Cities should be in "City, State, Country" format

        Description to convert:
        {description}
        """
        
        response = self.client.messages.create(
            model="claude-3-5-sonnet-20240620",
            max_tokens=1000,
            messages=[{"role": "user", "content": prompt}],
        )
        
        # Extract JSON from response
        try:
            result = json.loads(response.content[0].text)
        except (json.JSONDecodeError, IndexError) as e:
            raise ValueError(f"Failed to parse LLM response as JSON: {e}")
            
        # Validate against schema
        try:
            validate(instance=result, schema=json_schema)
        except ValidationError as e:
            raise ValueError(f"LLM response does not match required schema: {e}")
        
        # Fill in default values
        result = fill_default_values(result)
            
        return result

    def parse_scenario_description_old(self, description: str) -> Dict:
        """
        Convert natural language description into structured simulation parameters.

        Args:
            description: Natural language description of desired simulation scenario

        Returns:
            Dict containing structured simulation parameters with the following fields:
            - cities: List of city names
            - map_type: One of the valid road or junction types
            - adversity_types: List of valid adversity types
        """
        # 1. Parse the description into a structured format as a JSON object
        prompt = f"""
        Convert the following natural language description into a structured JSON object.

        Required JSON schema:
        {{
            "cities": ["city1, state1, country1", "city2, state2, country2"],  # List of cities where the scenario takes place
            "map_type": "one_of_valid_types",  # The type of road or intersection. Must be one of: {ROAD_TYPES + JUNCTION_TYPES}
            "adversity_types": ["type1", "type2"]  # Types of challenging scenarios. Must be from: {ADVERSITY_TYPES}
        }}

        Important rules:
        1. Return ONLY the JSON object, with no additional text
        2. For any field that is not clearly specified in the description, use an empty list []
        3. Make sure map_type is exactly one of the valid types listed above
        4. Make sure all adversity_types are from the valid list above
        5. Cities should be in "City, State, Country" format

        Description to convert:
        {description}
        """
        
        response = self.client.messages.create(
            model="claude-sonnet-4-20250514",
            max_tokens=1000,
            messages=[{"role": "user", "content": prompt}],
        )
        
        # Extract JSON from response
        try:
            result = json.loads(response.content[0].text)
        except (json.JSONDecodeError, IndexError) as e:
            raise ValueError(f"Failed to parse LLM response as JSON: {e}")
            
        # Validate against schema
        try:
            validate(instance=result, schema=json_schema)
        except ValidationError as e:
            raise ValueError(f"LLM response does not match required schema: {e}")
        
        # Fill in default values
        result = fill_default_values(result)
            
        return result
    
def fill_default_values(config: Dict) -> Dict:
    """
    Fill in default values for missing fields in the configuration.
    
    Args:
        config: Dictionary containing structured simulation parameters
        
    Returns:
        Dictionary containing structured simulation parameters with default values filled in
    """
    # Check if cities are provided
    # if config["cities"] is None or len(config["cities"]) == 0:
    #     config["cities"] = DEFAULT_US_CITIES[:2]
    
    # Check if map_type is provided
    # if config["map_type"] is None or len(config["map_type"]) == 0:
    #     config["map_type"] = "signalized"

    if "traffic_density" not in config or config["traffic_density"] is None:
        config["traffic_density"] = "medium"
    
    # Check if adversity_types are provided
    if "adversity_types" not in config or config["adversity_types"] is None or len(config["adversity_types"]) == 0:
        config["adversity_types"] = ADVERSITY_TYPES
    
    return config

def test_real_llm():
    parser = LLMParser()
    
    # Test with default parameters (random US cities and signalized intersection)
    result1 = parser.parse_scenario_description(
        "I need a highway ramp scenario with high traffic density, with collisions happening on the ramp"
    )
    print("Scenario 1:", result1)
    
    # Test with specific cities and map types
    result2 = parser.parse_scenario_description(
        "I want roundabout fail-to-yield collisions in Ann Arbor and Chicago"
    )
    print("Scenario 2:", result2)

if __name__ == "__main__":
    test_real_llm()