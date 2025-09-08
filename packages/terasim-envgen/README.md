# TeraSim-Agent: Autonomous Driving Scenario Generation Platform

TeraSim-Agent is a comprehensive platform for generating realistic autonomous driving scenarios by combining real-world map data with AI-driven traffic simulation and corner case generation. The platform provides a complete workflow from map acquisition to accident scenario simulation.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Workflow Overview](#workflow-overview)
- [Detailed Usage Guide](#detailed-usage-guide)
- [Understanding the Output](#understanding-the-output)
- [API Usage](#api-usage)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)

## Overview

TeraSim-Agent implements a complete pipeline for autonomous driving scenario generation:

1. **Map Search & Download**: Search and download real-world map data from OpenStreetMap
2. **Map Conversion**: Convert OSM data to simulation formats (SUMO, OpenDRIVE, Lanelet2)
3. **Traffic Flow Generation**: Generate realistic traffic patterns and vehicle flows
4. **Corner Case Generation**: Create accident scenarios and edge cases for testing

The platform supports multiple road types (highways, roundabouts, signalized intersections) across major US cities and generates comprehensive test datasets for autonomous vehicle validation.

TeraSim-Agent is an intelligent system that converts natural language descriptions and AV task configurations into executable autonomous driving simulation scenarios through multi-round conversations.

## 🚀 New Features

### Multi-Round Conversation System
- **Smart Input Classification**: Automatically detects JSON AV task configurations vs. natural language requirements
- **Session Data Management**: Accumulates multiple inputs across conversation rounds
- **Professional UI States**: Enhanced loading states and status indicators during generation

### Professional User Experience
- **Intelligent Button States**: All controls are disabled during simulation generation
- **Progress Indicators**: Real-time status updates with estimated completion times
- **Professional Notifications**: Detailed success/error messages with actionable guidance
- **Loading Animations**: Smooth transitions and professional loading states

## 🎯 Usage Flow

### 1. Multi-Round Input
Users can naturally input different types of information:

```json
// Example AV Task Configuration
{
  "task_id": "highway_lane_change",
  "scenario_type": "highway",
  "objectives": ["lane_change", "obstacle_avoidance"],
  "constraints": {
    "speed_limit": 65,
    "weather": "clear"
  }
}
```

```text
// Example Natural Language Requirements
"I need rainy weather conditions with high traffic density during rush hour"
```

### 2. Session Management
- **AV Tasks**: Each JSON configuration is added to the task list
- **Environment Requirements**: Natural language inputs are merged into a comprehensive description
- **Real-time Status**: Session status is displayed with current data summary

### 3. Professional Generation Process
When "Start Simulation Generation" is clicked:

1. **Initiation Phase**
   - All UI controls are disabled
   - Professional loading state is displayed
   - Estimated time (2-5 minutes) is shown

2. **Processing Phase**
   - Backend processes session data
   - Natural language is parsed using LLM
   - Scenarios are generated for each AV task

3. **Completion Phase**
   - Success notification with detailed summary
   - Visualization environment becomes available
   - All controls are re-enabled

## 🔧 Technical Architecture

### Frontend (React + TypeScript)
- **Session Data Structure**: Manages AV tasks and natural language requirements
- **Smart Classification**: Client-side input type detection
- **Professional UI States**: Enhanced loading and status indicators

### Backend (FastAPI + Python)
- **Input Classification API**: `/api/classify-and-store`
- **Simulation Generation API**: `/api/generate-simulation`
- **Session Management**: In-memory session storage with unique IDs

## 🎨 UI/UX Enhancements

### During Generation
- ✅ **Start Simulation Generation** button shows loading spinner
- ✅ **Open Map Navigation** button is disabled with clear messaging
- ✅ **Input controls** are disabled with appropriate placeholders
- ✅ **Status indicator** shows "Generating Environment" with animation
- ✅ **Progress panel** displays estimated time and current status

### Professional Messaging
- 🚀 **Initiation**: "Simulation Generation Initiated" with detailed status
- ⏱️ **Progress**: Real-time updates with estimated completion time
- ✅ **Success**: Comprehensive summary with next steps
- ❌ **Error**: Detailed error information with troubleshooting guidance

## 📁 Session Management

### Automatic Session Saving
- ✅ **Auto-save**: Every simulation generation automatically saves session data
- ✅ **File format**: JSON files with timestamp naming (`session_data_YYYYMMDD_HHMMSS.json`)
- ✅ **Storage location**: `sessions/` directory
- ✅ **Complete data**: Includes AV tasks, natural language requirements, and metadata

### Session Management APIs
- 📋 **List sessions**: `GET /api/sessions` - View all saved sessions
- 🔍 **Get session**: `GET /api/sessions/{filename}` - Retrieve specific session data
- 🗑️ **Delete session**: `DELETE /api/sessions/{filename}` - Remove session file

### Command Line Tools
```bash
# List all saved sessions
python scripts/view_sessions.py list

# View detailed session information
python scripts/view_sessions.py view session_data_20241225_120000.json

# Delete a session file
python scripts/view_sessions.py delete session_data_20241225_120000.json
```

### Use Cases
- 🔍 **Debugging**: Analyze failed simulation configurations
- 🔄 **Replication**: Re-run previous simulation setups
- 📊 **Analytics**: Study user behavior and usage patterns
- 💾 **Backup**: Preserve important simulation configurations

## 🌟 Key Benefits

1. **Natural Workflow**: Users can input information in any order
2. **Professional Experience**: Enterprise-grade UI with clear status indicators
3. **Error Prevention**: Disabled controls prevent user confusion during generation
4. **Clear Communication**: Professional messaging keeps users informed
5. **Flexible Input**: Supports both structured JSON and natural language

## 🚀 Getting Started

### Prerequisites

### System Requirements

- **Operating System**: Ubuntu 20.04+ (tested) or similar Linux distribution
- **Python**: 3.8 or higher
- **Memory**: At least 8GB RAM recommended
- **Storage**: 10GB+ free space for generated datasets

### External Dependencies

#### CommonRoad Scenario Designer (for Map conversion)
```bash
# Install CommonRoad Scenario Designer
git clone https://github.com/miracelplus/commonroad-scenario-designer.git
cd commonroad-scenario-designer
pip install -r requirements.txt
python setup.py install
```

#### SUMO (for map generation using OSMWebWizard)
```bash
git clone https://github.com/miracelplus/sumo.git
```

#### Google Maps API (for route-based generation)
```bash
# Create .env file in project root
echo "GOOGLE_MAPS_API_KEY=your_api_key_here" > .env
# Note: Inside .env file, no quotes needed around the API key value
```

#### TomTom API (for route-based generation)
```bash
# Add to .env file (use >> to append, not overwrite)
echo "TOMTOM_API_KEY=your_api_key_here" >> .env
# Note: Inside .env file, no quotes needed around the API key value
```

#### TeraSim, TeraSim-NDE-NADE (for corner case generation)

```bash
# Install TeraSim-NDE-NADE
git clone https://github.com/mcity/TeraSim.git
git clone https://github.com/mcity/TeraSim-NDE-NADE.git
cd TeraSim
poetry install
cd ../TeraSim-NDE-NADE
poetry install
```

#### Optional: SUMO_Traj_Visualization (for visualization)
```bash
# Install SUMO_Traj_Visualization
git clone https://github.com/miracelplus/sumo_traj_visualization.git
cd sumo_traj_visualization
pip install -r requirements.txt
pip install -e .
```

## Installation

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/TeraSim-Agent.git
cd TeraSim-Agent
```

### 2. Install Python Dependencies
```bash
# Install dependencies
pip install -r requirements.txt

# Or use the installation script
python scripts/install_dependencies.py
```

### 3. Verify Installation
```bash
# Quick test to verify setup
python -c "import osmnx, sumolib; print('Installation successful!')"
```

## Workflow Overview

The TeraSim-Agent workflow consists of four main stages:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  1. Map Search  │───►│ 2. Map Convert  │───►│ 3. Traffic Flow │───►│ 4. Corner Cases │
│    & Download   │    │                 │    │   Generation    │    │   Generation    │
└─────────────────┘    └─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       │                       │                       │
        ▼                       ▼                       ▼                       ▼
   OSM map data          SUMO/OpenDRIVE           Vehicle routes         Accident scenarios
   + metadata           network files            + traffic flows         + trajectory data
```

## Detailed Usage Guide

### Stage 1: Map Search and Download

The platform can search for and download maps in multiple ways:

#### Option A: Search by City and Road Type
```python
from terasim_envgen.core.map_searcher import MapSearcher

# Initialize searcher
searcher = MapSearcher("test_config.yaml")

# Search for highway sections
highway_maps = searcher.search_roads(
    city="Ann Arbor, Michigan, USA",
    road_types=["motorway_link"],
    save_plots=True,
    max_samples=5,
    bbox_size=500
)

# Search for roundabouts
roundabout_maps = searcher.search_roads(
    city="Chicago, Illinois, USA",
    junction_tag="roundabout",
    save_plots=True,
    max_samples=3
)

# Search for signalized intersections
intersection_maps = searcher.find_junction(
    city="San Francisco, California, USA",
    filters={"min_street_count": 3},
    save_plots=True,
    max_samples=5
)
```

#### Option B: Route-Based Generation
```python
# Generate maps along a specific route
route_maps = searcher.get_maps_through_route(
    origin="1600 Amphitheatre Parkway, Mountain View, CA",
    destination="1 Hacker Way, Menlo Park, CA",
    mode="driving",
    output_dir="route_output",
    target_split_distance=2000,
    bbox_size=1000
)
```

#### Option C: Batch Generation (Recommended)
```python
# Run the test script to generate comprehensive dataset
python tests/test_map_searcher.py
```

This will generate maps for multiple cities and road types:
- **Cities**: Ann Arbor, Chicago, San Francisco, New York, Los Angeles, Houston, Phoenix, Philadelphia, San Antonio, San Diego
- **Road Types**: Highway, Roundabout, Signalized Intersection
- **Output**: `test_output_validate/` directory

### Stage 2: Map Conversion

Convert downloaded OSM maps to simulation formats:

```python
from terasim_envgen.core.map_converter import MapConverter

converter = MapConverter("config/config.yaml")

# Convert single map
net_file, xodr_file, ll2_file = converter.convert(
    osm_path="test_output_validate/Ann_Arbor_Michigan_USA_highway_c8189017/map.osm",
    scene_id="highway_test",
    scenario_name="autonomous_driving"
)
```

**Output Files:**
- `map.net.xml`: SUMO network file
- `map.xodr`: OpenDRIVE format (for advanced simulators)
- `map.lanelet2.osm`: Lanelet2 format (for ROS-based systems)
- `map.poly.xml`: Polygon file for visualization

### Stage 3: Traffic Flow Generation

Generate realistic traffic patterns:

```python
from terasim_envgen.core.traffic_flow_generator import TrafficFlowGenerator

# Initialize traffic generator
traffic_gen = TrafficFlowGenerator("config/config.yaml")

# Generate traffic for single map
routes_file = traffic_gen.generate_flows(
    net_path="test_output_validate/Ann_Arbor_Michigan_USA_highway_c8189017/map.net.xml",
    end_time=3600,
    traffic_level="medium",  # options: low, medium, high
    vehicle_types=["vehicle", "pedestrian", "bicycle"]
)

# Batch generate for all maps
traffic_gen.generate_multi_level_flows("test_output_validate/")
```

**Traffic Levels:**
- **Low**: 5 seconds between vehicles
- **Medium**: 2 seconds between vehicles  
- **High**: 0.3 seconds between vehicles

**Generated Files:**
- `vehicles_[level].rou.xml`: Vehicle routes
- `pedestrians_[level].rou.xml`: Pedestrian routes
- `bicycles_[level].rou.xml`: Bicycle routes
- `sumo_[level].sumocfg`: SUMO configuration
- `trips_[level].trips.xml`: Trip definitions

### Stage 4: Corner Case and Accident Generation

Generate realistic accident scenarios:

```python
# Single scenario generation
python src/core/terasim_corner_case_generator.py \
    adversities="vehicle:highway_cutin" \
    road_path="test_output_validate/Ann_Arbor_Michigan_USA_highway_c8189017" \
    output_folder_name="highway_cutin_0"

# Multiple adversity types
python src/core/terasim_corner_case_generator.py \
    adversities="vehicle:highway_merge;vru:jaywalking" \
    road_path="test_output_validate/Chicago_Illinois_USA_roundabout_7eed809c" \
    output_folder_name="mixed_scenario_0"
```

#### Available Adversity Types

**Vehicle Adversities:**
- `highway_cutin`: Lane change cut-in maneuvers
- `highway_merge`: Highway merging scenarios
- `highway_rearend_accel`: Rear-end collision from acceleration
- `highway_rearend_decel`: Rear-end collision from sudden braking
- `roundabout_cutin`: Cut-in maneuvers in roundabouts
- `roundabout_fail_to_yield`: Failure to yield in roundabouts
- `intersection_tfl`: Traffic light violations
- `intersection_headon`: Head-on collisions
- `intersection_rearend`: Intersection rear-end collisions

**Vulnerable Road User (VRU) Adversities:**
- `jaywalking`: Pedestrian jaywalking scenarios
- `stopcrossing`: Pedestrian stopping mid-crossing

#### Batch Corner Case Generation
```python
# Generate all scenarios for all maps
python tests/test_corner_case_generator.py
```

## Understanding the Output

### Directory Structure

After running the complete pipeline, your output will be organized as:

```
test_output_validate/
├── Ann_Arbor_Michigan_USA_highway_c8189017/
│   ├── metadata.json                     # Map metadata and center coordinates
│   ├── preview.png                       # Satellite + graph view preview
│   ├── map.osm                          # Original OpenStreetMap data
│   ├── map.net.xml                      # SUMO network file
│   ├── map.xodr                         # OpenDRIVE format
│   ├── map.lanelet2.osm                 # Lanelet2 format
│   ├── vehicles_medium.rou.xml          # Vehicle routes (medium traffic)
│   ├── sumo_medium.sumocfg              # SUMO configuration
│   └── simulation_output/                        # Corner case scenarios
│       ├── highway_cutin_0/
│       │   ├── fcd_all.xml              # Vehicle trajectory data
│       │   ├── collision.xml            # Collision events
│       │   ├── monitor.json             # Scenario metadata
│       │   ├── scenario.xosc            # OpenSCENARIO file
│       │   └── video.mp4                # Simulation video
│       ├── highway_cutin_1/
│       └── highway_merge_0/
├── Chicago_Illinois_USA_roundabout_7eed809c/
└── [other cities and scenarios...]
```

### Key Output Files

**metadata.json**: Contains map information
```json
{
  "center_coordinates": [42.2387894, -83.7514961],
  "bbox_size": 500,
  "scene_id": "Ann_Arbor_Michigan_USA_highway_c8189017",
  "av_route": [[42.240, -83.751], [42.241, -83.752], ...]
}
```

**monitor.json**: Contains scenario outcome data
```json
{
  "veh_1_id": "vehicle_123",
  "veh_2_id": "vehicle_456", 
  "collision_time": 1450.2,
  "collision_type": "highway_cutin"
}
```

## API Usage

TeraSim-Agent provides a REST API for integration with other systems:

### Start the API Server
```bash
python main.py
```

### API Endpoints

#### Parse Natural Language Descriptions
```bash
curl -X POST "http://localhost:8000/api/parse" \
  -H "Content-Type: application/json" \
  -d '{"prompt": "Generate a highway scenario with aggressive lane changing"}'
```

#### Launch Visualization
```bash
curl -X POST "http://localhost:8000/api/visualize" \
  -H "Content-Type: application/json" \
  -d '{
    "root_dir": "test_output_validate",
    "filter_type": "highway_cutin",
    "dataset_name": "TeraSim-Dataset"
  }'
```

### Frontend Interface

A web-based frontend is available in the `frontend/` directory:

```bash
cd frontend
npm install
npm start
```

Access the interface at `http://localhost:3000`

## Configuration

### Main Configuration (test_config.yaml)
```yaml
map_search:
  bbox_size: 500                    # Map bounding box size in meters
  default_city: Ann Arbor, Michigan, USA
  max_results: 5                    # Maximum maps per search

output:
  base_dir: test_output_validate    # Output directory

visualization:
  preview:
    dpi: 300                        # Image resolution
    satellite_view: true            # Include satellite imagery
    randomize:
      enabled: true                 # Randomize visualization styles
      seed: 33                      # Random seed for reproducibility

traffic:
  end_time: 3600                    # Simulation duration (seconds)
  vehicle_period: 0.5               # Base vehicle insertion period
```

### Adversity Configuration

Individual adversity types are configured in `src/core/conf/adversity/`:

```yaml
# Example: highway_cutin.yaml
adversity_cfg:
  vehicle:
    highway_cutin:
      _target_: terasim_nde_nade.adversity.vehicles.LanechangeAdversity
      location: 'highway'
      ego_type: 'vehicle'
      probability: 0.001
      predicted_collision_type: "highway_cutin"
```

## Troubleshooting

### Common Issues

**1. SUMO_HOME not found**
```bash
export SUMO_HOME=/usr/share/sumo
export PATH=$PATH:$SUMO_HOME/bin
```

**2. OSMnx download failures**
- Check internet connection
- Try different time periods (OSM servers may be busy)
- Reduce bbox_size if maps are too large

**3. Traffic generation fails**
```python
# Enable fringe factor fallback
generator = TrafficFlowGenerator(allow_fringe=True)
```

**4. Corner case generation timeout**
- Increase timeout in test scripts
- Reduce number of parallel workers
- Check SUMO configuration files

### Performance Optimization

**Parallel Processing**: Use multiple CPU cores
```python
# In test scripts, adjust max_workers
run_all_experiments_parallel(max_workers=8)
```

**Memory Management**: For large datasets
```python
# Process in smaller batches
cities = ["Ann Arbor", "Chicago"]  # Start with fewer cities
```

## Contributing

We welcome contributions! Please see our contributing guidelines:

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Submit a pull request

### Development Setup
```bash
# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
python -m pytest tests/

# Format code
black src/ tests/
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Citation

If you use TeraSim-Agent in your research, please cite:

```bibtex
@software{terasim_agent,
  title={TeraSim-Agent: Autonomous Driving Scenario Generation Platform},
  author={Your Name},
  year={2024},
  url={https://github.com/your-org/TeraSim-Agent}
}
```

## Support

For questions and support:
- Create an issue on GitHub
- Check the documentation in `docs/`
- Review existing test examples in `tests/`

---

**Note**: This platform generates large amounts of data. Ensure you have sufficient storage space before running batch operations. A complete dataset for all cities and scenarios can require 50GB+ of storage.

### Prerequisites
```bash
# Backend dependencies
pip install fastapi uvicorn pydantic anthropic fiftyone

# Frontend dependencies
cd frontend && npm install
```

### Running the Application
```bash
# Start backend (Terminal 1)
python main.py

# Start frontend (Terminal 2)
cd frontend && npm run dev
```

### Access Points
- **Frontend**: http://localhost:5174 (or 5173)
- **Backend API**: http://localhost:8000
- **API Documentation**: http://localhost:8000/docs

## 📝 Example Conversation

```
User: {"task_id": "urban_intersection", "scenario": "traffic_light_compliance"}
System: ✅ AV task configuration recorded (Total tasks: 1)

User: Add heavy rain and reduced visibility conditions
System: ✅ Environment requirements added to scenario description

User: Also include pedestrian crossings
System: ✅ Environment requirements added to scenario description
        📊 Current session status:
        • AV Tasks: 1 configured
        • Environment requirements: Collected
        ✨ Ready to generate simulation scenarios!

[User clicks "Start Simulation Generation"]
System: 🚀 Simulation Generation Initiated
        Processing your configuration and generating test environments...
        ⏱️ Estimated Time: 2-5 minutes
        [All controls disabled during generation]

System: ✅ Simulation Generation Completed Successfully
        📈 Generation Summary:
        • Scenarios Generated: 1
        • Visualization Environment: Ready
        • Data Processing: Complete
```

This enhanced system provides a professional, user-friendly experience for creating complex autonomous driving simulation scenarios through natural conversation.

## System Requirements 