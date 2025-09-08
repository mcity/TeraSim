import os
import sys
import logging
import glob

sys.path.append("src")

# Set up logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Add the project root directory to Python path
folder_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "test_output"
)

logger.info(f"Searching for maps and trajectories in {folder_path}")

start_time = 200.0
end_time = 215.0
logger.info(f"Using time window from {start_time}s to {end_time}s")

# iterate all subdirectories in the folder_path
for subdir in ["intersections"]:
    subdir_path = os.path.join(folder_path, subdir)
    if os.path.isdir(subdir_path):
        # Get list of all scenario directories
        scenario_dirs = [
            d
            for d in os.listdir(subdir_path)
            if os.path.isdir(os.path.join(subdir_path, d))
        ]
        logger.info(f"Found {len(scenario_dirs)} scenarios in {subdir_path}")

        # Process all scenarios
        for test_scenario in scenario_dirs:
            scenario_path = os.path.join(subdir_path, test_scenario)
            logger.info(f"Processing scenario: {scenario_path}")

            map_path = os.path.join(scenario_path, "map.xodr")
            fcd_path = os.path.join(scenario_path, "sumo_medium.fcd.xml")

            if os.path.exists(map_path) and os.path.exists(fcd_path):
                logger.info(f"Processing {scenario_path}")
                output_path = os.path.join(scenario_path, "openscenario_new.xosc")
                cmd = f"python -m src.openscenario_converter --map_path {map_path} --fcd_path {fcd_path} --output_path {output_path} --start_time {start_time} --end_time {end_time}"
                logger.info(f"Running command: {cmd}")

                exit_code = os.system(cmd)

                # Check if the conversion was successful
                if exit_code == 0 and os.path.exists(output_path):
                    logger.info(
                        f"Successfully created OpenSCENARIO file: {output_path}"
                    )

                    # Print file size
                    file_size = os.path.getsize(output_path) / (
                        1024 * 1024
                    )  # Size in MB
                    logger.info(f"File size: {file_size:.2f} MB")

                    # Print first few lines of the file
                    logger.info("First 10 lines of the OpenSCENARIO file:")
                    with open(output_path, "r", encoding="utf-8") as f:
                        for i, line in enumerate(f):
                            if i < 10:
                                logger.info(line.strip())
                            else:
                                break
                else:
                    logger.error(
                        f"Failed to create OpenSCENARIO file, exit code: {exit_code}"
                    )
            else:
                if not os.path.exists(map_path):
                    logger.warning(f"Map file not found: {map_path}")
                if not os.path.exists(fcd_path):
                    logger.warning(f"FCD file not found: {fcd_path}")

logger.info("Conversion test completed")
