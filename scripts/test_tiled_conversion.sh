#!/bin/bash
# Test script for tiled XODR to SUMO conversion

echo "=========================================="
echo "Testing Tiled XODR to SUMO Conversion"
echo "=========================================="

# Example 1: Small tile size (1 mile = 1609.34 meters) for testing
echo -e "\n1. Testing with 1-mile tiles on Town01.xodr..."
python xodr_to_sumo_converter.py \
  --input ../examples/xodr_sumo_maps/xodr/Town01.xodr \
  --output test_town01_1mi \
  --tiled \
  --tile-size 1609.34 \
  --verbose

# Example 2: Default 5-mile tiles
echo -e "\n2. Testing with 5-mile tiles (default) on Town01.xodr..."
python xodr_to_sumo_converter.py \
  --input ../examples/xodr_sumo_maps/xodr/Town01.xodr \
  --output test_town01_5mi \
  --tiled \
  --verbose

# Example 3: Custom tile size (2 miles)
echo -e "\n3. Testing with 2-mile tiles on Town02.xodr..."
python xodr_to_sumo_converter.py \
  --input ../examples/xodr_sumo_maps/xodr/Town02.xodr \
  --output test_town02_2mi \
  --tiled \
  --tile-size 3218.68 \
  --verbose

echo -e "\n=========================================="
echo "Test complete! Check the output files:"
echo "  - test_town01_1mi_tile_*"
echo "  - test_town01_5mi_tile_*"
echo "  - test_town02_2mi_tile_*"
echo "=========================================="
