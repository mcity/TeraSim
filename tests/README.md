# TeraSim Test Suite

This directory contains the comprehensive test suite for the TeraSim monorepo.

## Test Structure

```
tests/
├── __init__.py                    # Test package initialization
├── conftest.py                    # Pytest configuration and shared fixtures
├── fixtures/                     # Test data and configuration files
│   ├── configs/                  # Test-specific configurations
│   ├── data/                     # Test data files
│   └── README.md
├── test_core/                    # Core simulation engine tests
│   ├── __init__.py
│   └── test_physics.py          # Physics and simulation mechanics
├── test_envgen/                  # Environment generation tests
│   ├── __init__.py
│   ├── config.yaml              # Test configuration
│   ├── README.md                # Configuration documentation
│   └── test_*.py                # Various envgen component tests
├── test_nde_nade/               # NDE-NADE component tests
├── test_service/                # Service API tests
└── test_integration/            # End-to-end integration tests
```

## Running Tests

### Using the Test Script (Recommended)
```bash
# Run all tests
./scripts/run_tests.sh

# Run specific test categories
./scripts/run_tests.sh --core
./scripts/run_tests.sh --envgen
./scripts/run_tests.sh --integration

# Additional options
./scripts/run_tests.sh --skip-slow --html
./scripts/run_tests.sh --verbose --no-coverage
```

### Using Make Commands
```bash
# Run all tests
make test

# Run specific categories
make test-core
make test-envgen
make test-integration

# Run fast tests (skip slow ones)
make test-fast
```

### Using pytest Directly
```bash
# Run all tests
pytest

# Run specific directories
pytest tests/test_core/
pytest tests/test_envgen/

# Run with specific markers
pytest -m "not slow"              # Skip slow tests
pytest -m "not requires_sumo"     # Skip SUMO-dependent tests
pytest -m integration             # Run only integration tests

# Generate coverage report
pytest --cov=terasim --cov-report=html
```

## Test Categories and Markers

### Test Categories
- **Core Tests** (`test_core/`): Basic simulation engine functionality
- **Environment Generation** (`test_envgen/`): Map generation, scenario creation
- **Service Tests** (`test_service/`): API endpoints, web interface
- **NDE-NADE Tests** (`test_nde_nade/`): Neural components, adversity generation
- **Integration Tests** (`test_integration/`): End-to-end system tests

### Test Markers
- `@pytest.mark.integration` - Integration tests (may be slow)
- `@pytest.mark.requires_sumo` - Tests requiring SUMO installation
- `@pytest.mark.requires_gui` - Tests requiring GUI (skipped in CI)
- `@pytest.mark.slow` - Slow-running tests

## Requirements

### Required Dependencies
- pytest >= 7.4.0
- pytest-cov >= 4.1.0 (for coverage reports)

### Optional Dependencies
- SUMO (for integration tests marked with `requires_sumo`)
- GUI environment (for tests marked with `requires_gui`)

### Environment Variables
- `SUMO_HOME` - Path to SUMO installation (required for SUMO tests)
- `TERASIM_TEST_MODE` - Automatically set to "1" during test runs
- `TERASIM_LOG_LEVEL` - Set to "WARNING" during tests to reduce noise

## Writing New Tests

### Basic Test Structure
```python
import pytest
from terasim_envgen.core.some_module import SomeClass

def test_some_functionality(temp_dir, base_config):
    \"\"\"Test description.\"\"\"
    # Use fixtures provided by conftest.py
    instance = SomeClass(config=base_config)
    result = instance.do_something()
    
    assert result is not None
    # Add more assertions

@pytest.mark.slow
def test_slow_operation():
    \"\"\"Mark slow tests appropriately.\"\"\"
    # Long-running test code
    pass

@pytest.mark.requires_sumo
def test_sumo_integration():
    \"\"\"Mark tests that require SUMO.\"\"\"
    # Test code that uses SUMO
    pass
```

### Available Fixtures
- `project_root` - Path to project root directory
- `temp_dir` - Temporary directory for test outputs
- `base_config` - Basic configuration for tests
- `test_data_dir` - Path to test data directory
- `test_config_dir` - Path to test configuration directory
- `example_scenario_config` - Path to example scenario
- `mcity_map_path` - Path to Mcity test map

### Best Practices
1. Use descriptive test names that explain what is being tested
2. Keep tests independent - they should not depend on other tests
3. Use appropriate markers for slow or special-requirement tests
4. Use fixtures for common setup code
5. Assert on specific expected behaviors, not just "no errors"
6. Clean up resources (temp files, connections) - fixtures handle this automatically

## Continuous Integration

The test suite is designed to work in CI environments:
- GUI tests are automatically skipped when `CI` environment variable is set
- SUMO tests are skipped if SUMO_HOME is not available
- Coverage reports are generated and can be uploaded to coverage services

## Troubleshooting

### Common Issues

**Import errors**: Make sure you're running tests from the project root and all dependencies are installed.

**SUMO tests failing**: Ensure SUMO is installed and SUMO_HOME environment variable is set correctly.

**Slow tests timing out**: Use `--skip-slow` flag or increase timeout limits.

**Coverage issues**: Ensure all packages are installed in development mode (`uv sync`).

### Getting Help
- Check test output for specific error messages
- Run with `--verbose` flag for more detailed output
- Look at `conftest.py` for available fixtures and configuration
- Check individual test files for specific test documentation