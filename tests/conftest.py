"""
Pytest configuration and shared fixtures for TeraSim tests.

This file provides common test configurations and fixtures that can be
used across all test modules in the TeraSim test suite.
"""

import os
import sys
import tempfile
from pathlib import Path
from typing import Dict, Any

import pytest
from omegaconf import OmegaConf, DictConfig


# Add project root to Python path for imports
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


@pytest.fixture(scope="session")
def project_root() -> Path:
    """Provide the project root directory path."""
    return PROJECT_ROOT


@pytest.fixture(scope="session") 
def test_data_dir() -> Path:
    """Provide the test data directory path."""
    return PROJECT_ROOT / "tests" / "fixtures" / "data"


@pytest.fixture(scope="session")
def test_config_dir() -> Path:
    """Provide the test configuration directory path."""
    return PROJECT_ROOT / "tests" / "fixtures" / "configs"


@pytest.fixture
def temp_dir():
    """Provide a temporary directory for test outputs."""
    with tempfile.TemporaryDirectory() as tmpdir:
        yield Path(tmpdir)


@pytest.fixture
def base_config() -> DictConfig:
    """Provide a base configuration for testing."""
    config = {
        "simulator": {
            "parameters": {
                "gui_flag": False,
                "realtime_flag": False,
                "step_length": 0.1
            }
        },
        "environment": {
            "parameters": {
                "warmup_time_lb": 0,
                "warmup_time_ub": 10,
                "max_simulation_time": 300
            }
        },
        "output": {
            "dir": "/tmp/terasim_test",
            "name": "test_run",
            "nth": "001"
        }
    }
    return OmegaConf.create(config)


@pytest.fixture
def example_scenario_config() -> Path:
    """Provide path to an example scenario configuration."""
    return PROJECT_ROOT / "examples" / "scenarios" / "cutin.yaml"


@pytest.fixture
def test_maps_dir() -> Path:
    """Provide the test maps directory path."""
    return PROJECT_ROOT / "examples" / "maps"


@pytest.fixture
def mcity_map_path(test_maps_dir) -> Path:
    """Provide path to Mcity test map."""
    return test_maps_dir / "Mcity"


@pytest.fixture(autouse=True)
def setup_test_environment():
    """Set up test environment variables and cleanup after tests."""
    # Set test-specific environment variables
    original_env = {}
    test_env_vars = {
        "TERASIM_TEST_MODE": "1",
        "SUMO_HOME": os.environ.get("SUMO_HOME", "/usr/share/sumo"),
        "TERASIM_LOG_LEVEL": "WARNING"  # Reduce log noise during tests
    }
    
    for key, value in test_env_vars.items():
        original_env[key] = os.environ.get(key)
        os.environ[key] = value
    
    yield
    
    # Restore original environment
    for key, original_value in original_env.items():
        if original_value is None:
            os.environ.pop(key, None)
        else:
            os.environ[key] = original_value


def pytest_configure(config):
    """Configure pytest with custom markers."""
    config.addinivalue_line(
        "markers", "integration: mark test as an integration test (may be slow)"
    )
    config.addinivalue_line(
        "markers", "requires_sumo: mark test as requiring SUMO installation"
    )
    config.addinivalue_line(
        "markers", "requires_gui: mark test as requiring GUI (will be skipped in CI)"
    )
    config.addinivalue_line(
        "markers", "slow: mark test as slow running"
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection to add markers automatically."""
    for item in items:
        # Add integration marker to integration tests
        if "integration" in str(item.fspath):
            item.add_marker(pytest.mark.integration)
        
        # Add requires_sumo marker to tests that use SUMO
        if "sumo" in item.name.lower() or "simulation" in item.name.lower():
            item.add_marker(pytest.mark.requires_sumo)


# Skip tests that require SUMO if it's not available
def pytest_runtest_setup(item):
    """Skip tests that require unavailable dependencies."""
    if item.get_closest_marker("requires_sumo"):
        sumo_home = os.environ.get("SUMO_HOME")
        if not sumo_home or not Path(sumo_home).exists():
            pytest.skip("SUMO not found - set SUMO_HOME environment variable")
    
    if item.get_closest_marker("requires_gui") and os.environ.get("CI"):
        pytest.skip("GUI tests skipped in CI environment")