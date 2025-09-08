#!/bin/bash

# Example usage of generate_experiments.py
# This script demonstrates various ways to generate TeraSim experiments

echo "TeraSim Experiment Generation Examples"
echo "======================================"

# Example 1: Single scenario generation with specific location
echo "Example 1: Generating Ann Arbor scenario..."
python generate_experiments.py \
    --lat 42.277547 \
    --lon -83.734668 \
    --bbox 500 \
    --name ann_arbor_university \
    --traffic low medium high \
    --formats sumo \
    --output generated_experiments/single

# Example 2: Batch generation from file
echo -e "\nExample 2: Batch generation from US cities file..."
python generate_experiments.py \
    --batch examples/coordinates/us_cities.txt \
    --bbox 300 \
    --traffic medium \
    --formats sumo \
    --output generated_experiments/batch

# Example 3: Multiple coordinates from command line
echo -e "\nExample 3: Multiple coordinates via command line..."
python generate_experiments.py \
    --coordinates 37.7749 -122.4194 40.7128 -74.0060 \
    --bbox 400 \
    --traffic low high \
    --output generated_experiments/multi

# Example 4: Generation with AV route
echo -e "\nExample 4: Scenario with predefined AV route..."
python generate_experiments.py \
    --lat 42.277547 \
    --lon -83.734668 \
    --bbox 600 \
    --name ann_arbor_with_route \
    --route examples/coordinates/av_route_example.txt \
    --traffic high \
    --output generated_experiments/with_route

# Example 5: Dry run to preview what would be generated
echo -e "\nExample 5: Dry run mode (preview only)..."
python generate_experiments.py \
    --lat 30.267153 \
    --lon -97.743057 \
    --bbox 500 \
    --name austin_downtown \
    --dry-run \
    --verbose

echo -e "\nAll examples completed!"
echo "Check the 'generated_experiments' directory for output files."