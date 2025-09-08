import os
import pytest
import glob
from pathlib import Path
from tqdm import tqdm
import sys

# Add the project root directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from terasim_envgen.core.traffic_flow_generator import TrafficFlowGenerator, test_generate_flows_for_all_networks


    
if __name__ == "__main__":
    test_generate_flows_for_all_networks(output_dir="palo-alto", multi_process=False)
