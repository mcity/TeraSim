# Test Fixtures

This directory contains test data and configuration files used by the TeraSim test suite.

## Directory Structure

- `configs/` - Test configuration files (YAML, etc.)
- `data/` - Test data files (maps, scenarios, expected outputs)

## Usage

Test fixtures are automatically available through pytest fixtures defined in `conftest.py`:

- `test_config_dir` - Path to test configuration directory
- `test_data_dir` - Path to test data directory
- `base_config` - Default configuration for tests
- `example_scenario_config` - Example scenario configuration

## Adding New Fixtures

When adding new test data:

1. Place configuration files in `configs/`
2. Place data files (maps, results) in `data/`
3. Update `conftest.py` if new fixtures are needed
4. Document the fixture purpose and usage