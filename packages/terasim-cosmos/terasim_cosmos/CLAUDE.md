# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with the TeraSim-World package.

## Project Overview

TeraSim-World is a bridge package that converts TeraSim traffic simulation data into world model inputs compatible with NVIDIA Cosmos Transfer1 models. This package enables the generation of photorealistic driving scenarios from TeraSim simulations for advanced autonomous vehicle testing and validation.

The package is adapted from [Cosmos-Drive-Dreams toolkits](https://github.com/nv-tlabs/Cosmos-Drive-Dreams/tree/main/cosmos-drive-dreams-toolkits) and extends NVIDIA's original capabilities with TeraSim-specific functionality for seamless integration with TeraSim's simulation outputs.

## Common Development Commands

### Environment Setup
```bash
# Create and activate conda environment
conda env create -f environment.yaml
conda activate cosmos-av-toolkits

# For Waymo Open Dataset support (optional)
pip install waymo-open-dataset-tf-2-11-0==1.6.1
```

### Running Conversions
```bash
# Convert TeraSim data to Cosmos input format
python terasim_to_cosmos_input.py

# Convert TeraSim to RDS-HQ format directly
python convert_terasim_to_rds_hq.py -i <terasim_output_dir> -o <output_wds_path>

# Convert Waymo data to RDS-HQ format
python convert_waymo_to_rds_hq.py -i <waymo_tfrecords_dir> -o <output_dir>/videos -n 16

# Render HD maps and LiDAR from RDS-HQ format
python render_from_rds_hq.py -d waymo -i <input_dir> -o <output_dir> -c pinhole -p True -n 8
```

### Visualization Tools
```bash
# Interactive 3D visualization of RDS-HQ dataset
python visualize_rds_hq.py -i <RDS_HQ_FOLDER> -c <CLIP_ID>

# TeraSim-specific visualization
python terasim_vis.py
```

### Text Embedding Generation
```bash
# Create T5 embeddings for multiview captions
python create_t5_embed_mv.py --text_file ./assets/waymo_multiview_texts.json --data_root <data_root>

# Create T5 embeddings for single view captions
python create_t5_embed.py --caption_file ./assets/waymo_caption.csv --data_root <data_root>
```

## Architecture Overview

### Core Data Flow Pipeline

**TeraSim → RDS-HQ → Cosmos Transfer1**

1. **Input Processing**: TeraSim outputs (FCD, map, monitor files)
2. **Data Conversion**: Transform to RDS-HQ format via WebDataset (WDS)
3. **Rendering**: Generate HD maps, LiDAR visualizations, and camera views
4. **Street View Integration**: Enhance realism with real-world imagery
5. **Output Generation**: Cosmos-compatible inputs for world model training

### Core Components

**Main Entry Point (`terasim_to_cosmos_input.py`)**
- Orchestrates the entire conversion pipeline
- Processes TeraSim FCD (Floating Car Data) and map files
- Extracts vehicle trajectories and collision information
- Converts data to WebDataset (WDS) format
- Renders HD maps and sensor data for world model input

**Data Conversion Layer**
- `convert_terasim_to_rds_hq.py`: Core TeraSim to RDS-HQ converter
  - `TeraSim_Dataset` class: Handles TeraSim XML parsing and iteration
  - Coordinate system conversion (SUMO → Waymo/FLU conventions)
  - Vehicle pose and bounding box extraction
  - HD map feature extraction using SUMO network data
- `convert_waymo_to_rds_hq.py`: Waymo Open Dataset converter for comparison/testing

**Rendering System (`render_from_rds_hq.py`)**
- HD map rendering with depth-based visualization
- LiDAR point cloud simulation
- Multi-camera view generation (f-theta and pinhole models)
- Bounding box overlay and trajectory visualization

**Street View Integration (`street_view_analysis.py`)**
- `StreetViewRetrievalAndAnalysis` class for real-world image enhancement
- Google Street View API integration
- GPT-4 Vision-based environment description generation
- Coordinate transformation from SUMO to geographic coordinates

**Visualization Tools**
- `visualize_rds_hq.py`: Interactive 3D visualization using Viser
- `terasim_vis.py`: TeraSim-specific visualization utilities
- Support for dynamic bounding boxes and trajectory playback

### Camera System Architecture

**Abstract Camera Base (`utils/camera/base.py`)**
- `CameraBase` abstract class defining camera projection interface
- Ray-pixel conversion methods for both PyTorch and NumPy
- 3D point transformation utilities
- Rendering methods for points, lines, and hulls with depth-based coloring

**Camera Implementations**
- `utils/camera/ftheta.py`: F-Theta (fisheye) camera model
- `utils/camera/pinhole.py`: Standard pinhole camera model
- Support for multiple camera viewpoints (front, left, right, rear)

**Camera Configuration**
- Waymo-style multi-camera setup (5 cameras: front, front_left, front_right, side_left, side_right)
- Default RDS-HQ camera configuration for TeraSim data
- Intrinsic parameter handling and pose interpolation

### Data Format Specifications

**Input Formats (TeraSim)**
- **FCD Files** (`fcd_all.xml`): Vehicle trajectory and state information in SUMO format
- **Map Files** (`map.net.xml`): Road network topology from SUMO
- **Monitor Files** (`monitor.json`): Collision and event records with vehicle IDs

**Intermediate Format (RDS-HQ/WebDataset)**
- **Poses**: Camera-to-world and vehicle-to-world transformation matrices
- **Intrinsics**: Camera calibration parameters (pinhole/f-theta)
- **HD Maps**: Lane lines, road boundaries, crosswalks, traffic signs
- **Bounding Boxes**: 3D object information with semantic labels
- **Metadata**: Temporal information and object tracking data

**Output Format (Cosmos Compatible)**
- **Videos**: Multi-camera rendered sequences at 30 FPS
- **HD Map Overlays**: Depth-encoded road structure visualizations
- **LiDAR Renderings**: Point cloud-based environmental structure
- **Text Embeddings**: T5-encoded scene descriptions for conditioning

### Key Design Patterns

1. **Adapter Pattern**: `TeraSim_Dataset` adapts SUMO/TeraSim data to Waymo-like interface
2. **Strategy Pattern**: Multiple camera models (pinhole, f-theta) with common interface
3. **Pipeline Pattern**: Sequential processing stages with intermediate format validation
4. **Factory Pattern**: Camera creation based on configuration settings
5. **Template Method**: Base rendering methods with camera-specific implementations

## Configuration System

### Environment Configuration (`environment.yaml`)
- **Python 3.10** with conda-forge dependencies
- **Core Libraries**: numpy, scipy, opencv-python-headless, trimesh
- **Deep Learning**: PyTorch 2.4.0 with CUDA 12.1 support
- **Geospatial**: pyproj for coordinate transformations
- **Data Processing**: webdataset, decord for video processing
- **Visualization**: viser for 3D interactive visualization
- **ML/NLP**: transformers for T5 text embeddings

### Asset Configuration
- **Default Intrinsics**: Pre-configured camera parameters in `config/default_ftheta_intrinsic/`
- **Example Data**: Sample TeraSim outputs in `assets/example/` with all data types
- **Camera Calibration**: F-theta intrinsic matrices for multi-view setup

### API Keys and External Services
- **Google Street View API**: For real-world image retrieval (requires `GOOGLE_MAPS_API_KEY`)
- **OpenAI API**: For GPT-4 Vision analysis (requires `OPENAI_API_KEY`)
- Both keys loaded via python-dotenv from `.env` file

## Common Development Tasks

### Adding New Camera Configuration
1. Define camera-to-vehicle transformation matrices in `convert_terasim_to_rds_hq.py:506-532`
2. Add intrinsic parameters for new camera setup
3. Update camera name lists and rendering pipeline
4. Test with sample TeraSim data

### Extending HD Map Features
1. Modify SUMO network parsing in `convert_terasim_hdmap()` function
2. Add new feature types to `hdmap_names_polyline` or `hdmap_names_polygon`
3. Update coordinate conversion and rendering methods
4. Ensure compatibility with Cosmos Transfer1 format requirements

### Integrating New Dataset Format
1. Create new dataset class following `TeraSim_Dataset` pattern
2. Implement required methods: `__iter__`, `__next__`, `_get_vehicle_pose`, `_get_all_agent_bbox`
3. Add coordinate system conversion utilities
4. Update main conversion pipeline to support new format

### Optimizing Rendering Performance
1. Use GPU acceleration for camera transformations with PyTorch tensors
2. Implement batch processing for multiple camera views
3. Cache frequently accessed data (rays, intrinsics) in camera classes
4. Use efficient data structures for large point clouds and polygon rendering

## Integration with TeraSim Ecosystem

### Data Flow from TeraSim Core
- Reads TeraSim simulation outputs (FCD XML, SUMO network files)
- Processes collision events from monitor.json for automated vehicle tracking
- Maintains compatibility with TeraSim's coordinate systems and data formats

### Output Integration with Cosmos Transfer1
- Generates training data compatible with NVIDIA Cosmos Transfer1 models
- Supports both single-view and multi-view camera configurations
- Provides text conditioning through street view analysis and description generation

### Quality Assurance and Validation
- HD map visualization for manual inspection of conversion accuracy
- Coordinate system validation through visual debugging tools
- Temporal consistency checks for pose interpolation and trajectory smoothness

## Dependencies and External Tools

### Core Dependencies
- **SUMO Integration**: Uses sumolib for network parsing and coordinate conversion
- **Computer Vision**: OpenCV for image processing and geometric operations
- **3D Graphics**: Trimesh for 3D mesh operations and spatial computations
- **Data Processing**: WebDataset for efficient large-scale data handling

### Optional Dependencies
- **Waymo Open Dataset**: For comparison and testing with industry-standard data
- **Ray**: For distributed processing of large datasets
- **Shapely**: For geometric operations on map features

### Development Tools
- **Viser**: Web-based 3D visualization for interactive debugging
- **Termcolor**: Colored console output for better development experience
- **TQDM**: Progress bars for long-running conversion operations

## Performance Considerations

### Memory Management
- Efficient WebDataset usage for large-scale data processing
- GPU memory management for PyTorch operations
- Streaming data processing to handle large FCD files

### Processing Speed
- Multi-threaded conversion with configurable worker counts
- Cached camera ray generation for repeated operations
- Optimized coordinate transformations using vectorized operations

### Storage Efficiency
- Compressed TAR archives for intermediate data storage
- Efficient NPZ format for numerical data arrays
- Progressive processing to avoid loading entire datasets into memory

## Troubleshooting Common Issues

### Coordinate System Problems
- Verify SUMO network file compatibility and coordinate reference system
- Check vehicle pose conversion from front-bumper to rear-axle positioning
- Validate HD map feature coordinates against visual inspection tools

### Camera Calibration Issues
- Ensure intrinsic parameters match expected camera specifications
- Verify camera-to-vehicle transformation matrices for multi-view setup
- Test ray-pixel conversion methods with known reference points

### API Integration Problems
- Check Google Street View API key validity and quota limits
- Verify OpenAI API key permissions for GPT-4 Vision access
- Handle network timeouts and rate limiting gracefully

### Data Format Compatibility
- Validate WebDataset TAR file structure and naming conventions
- Ensure temporal consistency in pose interpolation and frame indexing
- Check for missing or corrupted data files in conversion pipeline