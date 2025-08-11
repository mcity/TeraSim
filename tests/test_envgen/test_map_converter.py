import os
import sys
import logging
# Add the project root directory to Python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import dotenv
dotenv.load_dotenv()

# Now import the module after sys.path is modified
from terasim_envgen.core.map_converter import convert_all_osm_files

logger = logging.getLogger(__name__)


if __name__ == "__main__":
    convert_all_osm_files(output_dir="test_demo")
