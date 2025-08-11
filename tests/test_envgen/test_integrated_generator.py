"""
Test script for the integrated scenario generator.
Tests the complete pipeline from lat/lon input to final scenario generation.
"""

import pytest
import logging
from pathlib import Path

import dotenv
dotenv.load_dotenv()

from terasim_envgen.core.integrated_generator import IntegratedScenarioGenerator, generate_scenario_from_latlon


def test_generate_scenario_from_latlon(temp_dir):
    """Test scenario generation from lat/lon coordinates."""
    result = generate_scenario_from_latlon(
        lat=42.277547872840174,   
        lon=-83.73466185958247, 
        bbox_size=200, 
        output_dir=str(temp_dir / "test_ann_arbor"),
        scenario_name="ann_arbor"
    )
    
    # Add assertions here based on expected results
    assert result is not None

if __name__ == "__main__":
    test_generate_scenario_from_latlon(Path("outputs"))