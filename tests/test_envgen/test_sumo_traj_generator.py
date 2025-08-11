import os
import sys
import logging
import glob
from tqdm import tqdm

# Add the project root directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

result_path = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "test_output"
)

# Find all sumocfg files directly in test_output directory (no need to search in roads/ and intersections/)
sumocfg_files = glob.glob(
    os.path.join(result_path, "**/*sumo_medium.sumocfg"), recursive=True
)
print(f"Found {len(sumocfg_files)} SUMO configuration files to process")

# For each file, run sumo with 0.1s time resolution and output FCD data
for file in tqdm(sumocfg_files, desc="Processing SUMO configurations"):
    print(f"Processing {file}")
    os.system(
        f"sumo -c {file} --step-length 0.1 --fcd-output {file.replace('.sumocfg', '.fcd.xml')}"
    )
