#!/bin/bash

# Define base data directory
DATA_DIR="outputs/Mcity/raw_data/2025-11-03_19-35-25"
BASE_CONFIG="configs/visulation/example.yaml"
VISUALIZE_SCRIPT="scripts/visualize_fcd.py"

# Define paths relative to the data directory
CONFLICT_FILE="$DATA_DIR/conflict_info.jsonl"
OUTPUT_DIR="$DATA_DIR/visualization"

# Ensure output directory exists
mkdir -p "$OUTPUT_DIR"

# Track processed pairs to avoid duplicates
declare -A PROCESSED_PAIRS

# Temporary config file
TEMP_CONFIG="$OUTPUT_DIR/temp_config.yaml"

# Read conflict scenarios from JSONL file
while IFS= read -r line; do
    # Extract timestamp and conflict vehicle info
    TIMESTAMP=$(echo "$line" | jq -r '.timestamp')
    CONFLICTS=$(echo "$line" | jq -c '.conflict_veh_info | to_entries')

    # Iterate over all ego-adversary pairs in the conflict_veh_info
    for conflict in $(echo "$CONFLICTS" | jq -c '.[]'); do
        ADV_VEHICLE=$(echo "$conflict" | jq -r '.key')
        EGO_VEHICLE=$(echo "$conflict" | jq -r '.value[0]')

        # Generate a unique key for the ego-adv pair
        PAIR_KEY="${EGO_VEHICLE}_${ADV_VEHICLE}"

        # Log the pair being processed
        echo "Checking pair: $PAIR_KEY"

        # Skip if this pair has already been processed
        if [[ -n "${PROCESSED_PAIRS[$PAIR_KEY]}" ]]; then
            echo "Skipping already processed pair: $PAIR_KEY"
            continue
        fi

        # Mark this pair as processed
        PROCESSED_PAIRS[$PAIR_KEY]=1
        echo "Processing pair: $PAIR_KEY"

        # Copy base config to temporary config file
        cp "$BASE_CONFIG" "$TEMP_CONFIG"

        # Update the config file with scenario-specific details using sed
        sed -i "s|^fcd:.*|fcd: \"$DATA_DIR/fcd_all.xml\"|" "$TEMP_CONFIG"
        sed -i "s|^net:.*|net: \"examples/maps/Mcity/mcity.net.xml\"|" "$TEMP_CONFIG"
        sed -i "s|^ego_vehicle_id:.*|ego_vehicle_id: \"$EGO_VEHICLE\"|" "$TEMP_CONFIG"
        sed -i "s|^adv_vehicle_id:.*|adv_vehicle_id: \"$ADV_VEHICLE\"|" "$TEMP_CONFIG"
        sed -i "s|^video_name:.*|video_name: \"scenario_${TIMESTAMP}_${EGO_VEHICLE}_vs_${ADV_VEHICLE}\"|" "$TEMP_CONFIG"
        sed -i "s|^start_time:.*|start_time: $TIMESTAMP|" "$TEMP_CONFIG"
        sed -i "s|^end_time:.*|end_time: $(echo "$TIMESTAMP + 6" | bc)|" "$TEMP_CONFIG"
        sed -i "s|^output_dir:.*|output_dir: \"$OUTPUT_DIR\"|" "$TEMP_CONFIG"
        sed -i "/^max_frames:/d" "$TEMP_CONFIG"

        # Run the visualization script
        python3 "$VISUALIZE_SCRIPT" "$TEMP_CONFIG"
    done
done < "$CONFLICT_FILE"

echo "All scenarios processed. Videos saved to $OUTPUT_DIR."
