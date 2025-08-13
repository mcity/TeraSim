# TeraSim Experiment Generation Guide

## Overview

The `generate_experiments.py` script provides a comprehensive tool for generating complete simulation scenarios from geographic coordinates. It combines map downloading, format conversion, and traffic flow generation into a single, easy-to-use command-line interface.

## Features

- **Single and Batch Generation**: Generate one scenario or multiple scenarios at once
- **Flexible Input Methods**: Command line arguments, coordinate files, or batch files
- **Multiple Map Formats**: Support for SUMO, OpenDRIVE, and Lanelet2 formats
- **Traffic Density Control**: Generate low, medium, or high traffic density scenarios
- **AV Route Integration**: Include predefined autonomous vehicle routes
- **Comprehensive Output**: Maps, traffic flows, configuration files, and visualization

## Installation

Ensure TeraSim and its dependencies are installed:

```bash
# Install TeraSim with Poetry
poetry install

# Or ensure terasim-envgen is available
pip install -e packages/terasim-envgen
```

## Basic Usage

### Single Scenario Generation

Generate a scenario for a specific location:

```bash
python generate_experiments.py \
    --lat 42.277547 \
    --lon -83.734668 \
    --bbox 500 \
    --name ann_arbor
```

### Batch Generation from File

Create a file with coordinates (one lat,lon pair per line):

```text
# coordinates.txt
42.277547, -83.734668  # Ann Arbor
30.267153, -97.743057  # Austin
37.774929, -122.419418 # San Francisco
```

Then generate all scenarios:

```bash
python generate_experiments.py --batch coordinates.txt --bbox 400
```

### Multiple Coordinates via Command Line

```bash
python generate_experiments.py \
    --coordinates 37.7749 -122.4194 40.7128 -74.0060 \
    --bbox 300
```

## Advanced Options

### Traffic Density Control

Specify which traffic density levels to generate:

```bash
# Generate only low and high density
python generate_experiments.py \
    --lat 42.277547 --lon -83.734668 \
    --traffic low high
```

### Map Format Selection

Choose output map formats:

```bash
# Generate SUMO and OpenDRIVE formats
python generate_experiments.py \
    --lat 42.277547 --lon -83.734668 \
    --formats sumo opendrive
```

### Include AV Route

Define an AV route within the scenario:

```bash
# Create route file (av_route.txt)
# Format: lat,lon per line
42.278000, -83.735000
42.277800, -83.734500
42.277600, -83.734000

# Generate with route
python generate_experiments.py \
    --lat 42.277547 --lon -83.734668 \
    --route av_route.txt
```

### Custom Output Directory

```bash
python generate_experiments.py \
    --lat 42.277547 --lon -83.734668 \
    --output experiments/my_scenarios
```

### Dry Run Mode

Preview what would be generated without actually creating files:

```bash
python generate_experiments.py \
    --lat 42.277547 --lon -83.734668 \
    --dry-run --verbose
```

## Command Line Arguments

| Argument | Description | Default |
|----------|-------------|---------|
| `--lat` | Latitude of center point | Required* |
| `--lon` | Longitude of center point | Required with --lat |
| `--batch` | Path to coordinates file | Alternative to --lat |
| `--coordinates` | List of lat lon pairs | Alternative to --lat |
| `--bbox` | Bounding box size in meters | 500 |
| `--output` | Output directory | generated_experiments |
| `--name` | Scenario name | Auto-generated |
| `--traffic` | Traffic density levels | low medium high |
| `--formats` | Map formats to generate | sumo |
| `--route` | Path to AV route file | None |
| `--config` | Configuration file path | None |
| `--verbose` | Enable verbose logging | False |
| `--dry-run` | Preview without generating | False |

*One of `--lat`, `--batch`, or `--coordinates` is required

## Output Structure

Generated scenarios follow this structure:

```
generated_experiments/
├── scenario_name/
│   ├── map.osm                # Original OSM data
│   ├── map.net.xml            # SUMO network file
│   ├── map.xodr               # OpenDRIVE file (if requested)
│   ├── vehicles.rou.xml       # Traffic routes
│   ├── simulation.sumocfg     # SUMO configuration
│   ├── preview.png            # Map visualization
│   └── generation_summary.json # Generation metadata
└── generation_summary.json    # Overall summary
```

## Examples

### Example 1: Urban Intersection Testing

Generate a 300m scenario around a busy intersection:

```bash
python generate_experiments.py \
    --lat 37.7749 --lon -122.4194 \
    --bbox 300 \
    --name sf_intersection \
    --traffic high
```

### Example 2: Highway Testing

Generate multiple highway scenarios:

```bash
# Create highway_coords.txt with highway locations
python generate_experiments.py \
    --batch highway_coords.txt \
    --bbox 1000 \
    --traffic medium high \
    --formats sumo opendrive
```

### Example 3: Complete Test Suite

Generate a comprehensive test suite for multiple cities:

```bash
python generate_experiments.py \
    --batch examples/coordinates/us_cities.txt \
    --bbox 500 \
    --traffic low medium high \
    --formats sumo \
    --output test_suite/us_cities
```

## Integration with TeraSim

Generated scenarios can be directly used with TeraSim simulations:

```python
# Using generated scenario in run_experiments_debug.py
yaml_files = [Path("generated_experiments/ann_arbor/config.yaml")]
```

## Troubleshooting

### Common Issues

1. **ImportError for terasim_envgen**
   - Ensure the package is installed: `poetry install`
   - Check Python path includes the packages directory

2. **OSM Download Failures**
   - Check internet connection
   - Verify coordinates are valid
   - Try smaller bounding box size

3. **SUMO Conversion Errors**
   - Ensure SUMO is installed and configured
   - Run `setup_environment.sh` if needed
   - Check SUMO_HOME environment variable

4. **Memory Issues with Large Areas**
   - Reduce bounding box size
   - Generate fewer traffic density levels
   - Process coordinates in smaller batches

### Debug Mode

Enable verbose logging for detailed debug information:

```bash
python generate_experiments.py --lat 42.2775 --lon -83.7347 --verbose
```

## Performance Tips

1. **Batch Processing**: Use batch mode for multiple scenarios to optimize resource usage
2. **Selective Generation**: Only generate needed traffic densities and formats
3. **Parallel Processing**: Run multiple instances with different coordinate subsets
4. **Caching**: Generated OSM data is cached for 15 minutes for repeated generations

## See Also

- [TeraSim Documentation](../README.md)
- [Example Coordinates](../examples/coordinates/)
- [Example Scripts](../examples/scripts/generate_experiments_example.sh)