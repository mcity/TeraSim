from pathlib import Path
import sys
import argparse

from terasim_cosmos import TeraSimToCosmosConverter
     

if __name__ == "__main__":

    # example configs
    # Example 1: config/converter/us_chicago_uncoordinatedleftturn.yaml
    # Example 2: config/converter/us_arizona_highwaycutin.yaml

    parser = argparse.ArgumentParser()
    parser.add_argument("--path_to_config", type=Path, 
                        default='configs/converter/us_chicago_uncoordinatedleftturn.yaml',
                        help="Path to configuration file, should be in configs/converter folder")
    parser.add_argument("--streetview_retrieval", type=bool, 
                        default=True, 
                        help="Whether to retrieve street view imagery and get text prompt")
    
    args = parser.parse_args()
    path_to_config = args.path_to_config
    streetview_retrieval = args.streetview_retrieval
    
    converter = TeraSimToCosmosConverter.from_config_file(path_to_config)
    converter.convert(streetview_retrieval=streetview_retrieval)