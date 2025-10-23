# Tiled XODR to SUMO Conversion

## Overview

The XODR to SUMO converter now supports **spatial tiling** to handle large OpenDRIVE maps efficiently. This feature splits large XODR files into smaller tiles (default: 5mi × 5mi) before converting each tile to SUMO format.

## Why Use Tiled Conversion?

### Benefits:
1. **Memory Efficiency**: Process large maps without loading the entire network at once
2. **Parallel Processing**: Each tile can be processed independently (future enhancement)
3. **Incremental Updates**: Update specific regions without reprocessing the entire map
4. **Better Performance**: SUMO handles smaller networks more efficiently

### When to Use:
- Large road networks (> 10 sq miles)
- Memory-constrained systems
- Maps with distinct regions that can be processed separately

## Features

### Spatial Splitting
- Automatically calculates bounding box of the entire XODR file
- Creates a regular grid of tiles (configurable size)
- Assigns roads to tiles based on spatial overlap

### Junction Preservation
- **Automatic junction detection**: All roads belonging to a junction are kept together
- **Connection preservation**: Roads connected via predecessor/successor links are included
- **Merge zone handling**: Overlapping roads at tile boundaries are properly handled

### Output
For each non-empty tile, the converter generates:
- `{prefix}_tile_{row}_{col}.xodr` - Tile-specific OpenDRIVE file
- `{prefix}_tile_{row}_{col}.nod.xml` - SUMO nodes (Plain XML)
- `{prefix}_tile_{row}_{col}.edg.xml` - SUMO edges (Plain XML)
- `{prefix}_tile_{row}_{col}.con.xml` - SUMO connections (Plain XML)
- `{prefix}_tile_{row}_{col}.net.xml` - Final SUMO network

## Usage

### Basic Usage

```bash
# Convert with default 5-mile tiles
python xodr_to_sumo_converter.py \
  --input large_map.xodr \
  --output output_prefix \
  --tiled
```

### Custom Tile Size

```bash
# Use 2-mile tiles (3218.68 meters)
python xodr_to_sumo_converter.py \
  --input large_map.xodr \
  --output output_prefix \
  --tiled \
  --tile-size 3218.68
```

### With Verbose Output

```bash
# See detailed logging including tile assignments
python xodr_to_sumo_converter.py \
  --input large_map.xodr \
  --output output_prefix \
  --tiled \
  --verbose
```

### Without netconvert (Plain XML only)

```bash
# Generate only Plain XML files (skip netconvert step)
python xodr_to_sumo_converter.py \
  --input large_map.xodr \
  --output output_prefix \
  --tiled \
  --no-netconvert
```

## Command-Line Arguments

| Argument | Short | Description | Default |
|----------|-------|-------------|---------|
| `--input` | `-i` | Input OpenDRIVE file (.xodr) | Required |
| `--output` | `-o` | Output prefix | Based on input filename |
| `--tiled` | | Enable tiled conversion | False |
| `--tile-size` | | Tile size in meters | 8046.72 (5 miles) |
| `--no-netconvert` | | Skip netconvert (Plain XML only) | False |
| `--no-pyopendrive` | | Disable pyOpenDRIVE | False |
| `--verbose` | `-v` | Verbose logging | False |

## Tile Size Reference

| Distance | Meters | Use Case |
|----------|--------|----------|
| 1 mile | 1609.34 | Very fine-grained splitting |
| 2 miles | 3218.68 | Dense urban areas |
| 5 miles | 8046.72 | **Default** - balanced for most use cases |
| 10 miles | 16093.4 | Rural/highway networks |
| 20 miles | 32186.9 | Very large regions |

## How It Works

### 1. Bounds Calculation
```
Analyzing XODR file...
├─ Sample road geometries using pyOpenDRIVE
├─ Calculate bounding box for each road
└─ Determine overall map bounds
```

### 2. Tile Grid Creation
```
Creating spatial grid...
├─ Divide bounding box into tiles
├─ Assign tile IDs (tile_row_col)
└─ Calculate tile boundaries
```

### 3. Road Assignment
```
Assigning roads to tiles...
├─ Check spatial overlap for each road
├─ Identify junction memberships
├─ Add connected roads (predecessors/successors)
└─ Ensure junction integrity
```

### 4. Tile Processing
```
For each tile:
├─ Extract roads and junctions from XODR
├─ Create tile-specific XODR file
├─ Convert to Plain XML
└─ Run netconvert (optional)
```

## Junction Handling

The tiled conversion ensures junctions are never split across tiles:

1. **Junction Detection**: Identifies all roads belonging to each junction
2. **Complete Junction**: All junction roads are assigned to the same tile(s)
3. **Connecting Roads**: Roads referenced by junction connections are included
4. **Boundary Junctions**: Junctions near tile boundaries may be duplicated

## Example Output

```
============================================================
Starting tiled XODR to SUMO conversion
Tile size: 8046.72m (5.00 miles)
============================================================

Step 1: Calculating XODR bounds...
XODR bounds: (0.00, 0.00) to (12500.00, 8200.00)

Step 2: Creating spatial grid tiles...
Created 4 tiles of size 8046.72m x 8046.72m (5mi x 5mi)

Step 3: Assigning roads and junctions to tiles...
tile_0_0: 145 roads, 12 junctions
tile_0_1: 98 roads, 8 junctions
tile_1_0: 132 roads, 10 junctions
tile_1_1: 76 roads, 6 junctions

Step 4: Processing 4 tiles...

Processing tile_0_0...
Written tile XODR: output_prefix_tile_0_0.xodr (147 roads, 12 junctions)
✓ Successfully converted tile_0_0

Processing tile_0_1...
Written tile XODR: output_prefix_tile_0_1.xodr (99 roads, 8 junctions)
✓ Successfully converted tile_0_1

...

============================================================
Tiled conversion complete: 4/4 tiles successful
============================================================
```

## Implementation Details

### Key Methods

1. **`_get_road_bounds(py_road)`**: Calculate bounding box for a single road
2. **`_calculate_xodr_bounds(xodr_file)`**: Calculate overall map bounds
3. **`_create_tiles(bounds, tile_size_meters)`**: Generate tile grid
4. **`_assign_roads_to_tiles(tiles, xodr_file)`**: Assign roads/junctions to tiles
5. **`_write_xodr_tile(xodr_file, tile, output_file)`**: Create tile-specific XODR
6. **`convert_with_tiling(...)`**: Main tiled conversion workflow

### Data Structures

```python
@dataclass
class TileBounds:
    tile_id: str              # Unique identifier (e.g., "tile_0_1")
    min_x: float              # Tile bounding box
    min_y: float
    max_x: float
    max_y: float
    roads: List[str]          # Road IDs in this tile
    junctions: List[str]      # Junction IDs in this tile
```

## Troubleshooting

### Empty Tiles
Some tiles may be empty (no roads). These are automatically skipped.

### Missing Junctions
If junctions appear incomplete, check:
- Junction roads are within tile bounds
- Connected roads are included via predecessor/successor links

### Memory Issues
For very large maps, try smaller tile sizes:
```bash
--tile-size 1609.34  # 1 mile
```

### Performance
To speed up conversion:
- Skip netconvert in the first pass: `--no-netconvert`
- Process tiles in parallel (manual scripting required)

## Future Enhancements

- [ ] Automatic parallel processing of tiles
- [ ] Tile merging for adjacent regions
- [ ] Configurable overlap between tiles
- [ ] Boundary edge handling improvements
- [ ] Support for non-rectangular tile shapes

## Examples

See [test_tiled_conversion.sh](test_tiled_conversion.sh) for working examples.

## Requirements

- pyOpenDRIVE (for geometry processing)
- SUMO netconvert (for final network generation)
- Python 3.7+

## References

- OpenDRIVE Specification: https://www.asam.net/standards/detail/opendrive/
- SUMO Documentation: https://sumo.dlr.de/docs/
- pyOpenDRIVE: https://github.com/pyOpenDRIVE/pyOpenDRIVE
