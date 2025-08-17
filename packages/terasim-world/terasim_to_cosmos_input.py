from pathlib import Path
import json

from convert_terasim_to_rds_hq import convert_terasim_to_wds
from render_from_rds_hq import render_sample_hdmap
from street_view_analysis import StreetViewRetrievalAndAnalysis


def terasim_to_cosmos_input(path_to_output: Path, 
                            path_to_fcd: Path,
                            path_to_map: Path,
                            camera_setting_name: str = "default",
                            vehicle_id: str = None,
                            timestep_start: int = 0,
                            timestep_end: int = 100):
    
    print(f"Processing fcd: {path_to_fcd}")
    print(f"Processing map: {path_to_map}")
    
    if vehicle_id is None:
        print("No vehicle id provided, trying to load from monitor.json")
        try:
            path_to_collition_record = path_to_output / "monitor.json"
            with open(path_to_collition_record, "r") as f:
                collision_record = json.load(f)
            vehicle_id = collision_record["veh_1_id"]
            print(f"Using vehicle id: {vehicle_id}")
        except Exception as e:
            raise Exception(f"Error loading monitor.json: {e}")
    
    # Use the new class for street view retrieval and analysis
    streetview_analyzer = StreetViewRetrievalAndAnalysis()
    streetview_analyzer.get_streetview_image_and_description(
        path_to_output=path_to_output, 
        path_to_fcd=path_to_fcd,
        path_to_map=path_to_map,
        vehicle_id=vehicle_id,
        timestep_start=timestep_start,
        timestep_end=timestep_end
    )
    
    convert_terasim_to_wds(
        terasim_record_root=path_to_output,
        output_wds_path=path_to_output / "wds",
        single_camera=False,
        camera_setting_name=camera_setting_name,
        av_id=vehicle_id,
        timestep_start=timestep_start,
        timestep_end=timestep_end
    )

    if camera_setting_name == "waymo":
        settings = json.load(open("config/dataset_waymo_mv.json", "r"))
    elif camera_setting_name == "default":
        settings = json.load(open("config/dataset_rds_hq_mv_terasim.json", "r"))
    else:
        raise ValueError(f"Invalid camera setting name: {camera_setting_name}")
    
    render_sample_hdmap(
        input_root=path_to_output / "wds",
        output_root=path_to_output / "render" / vehicle_id,
        clip_id=path_to_output.stem,
        settings=settings,
        camera_type="ftheta",
    )
    return True
        

if __name__ == "__main__":
    # Example usage
    path_to_output = Path("/path/to/terasim/output")  # Replace with your TeraSim output directory
    path_to_fcd = path_to_output / "fcd_all.xml"
    path_to_map = path_to_output / "map.net.xml"
    camera_setting_name = "default" # "waymo" or "default"
    vehicle_id = None  # Will auto-detect from monitor.json
    timestep_start = -40
    timestep_end = -1
    
    terasim_to_cosmos_input(path_to_output=path_to_output, 
                            path_to_fcd=path_to_fcd,
                            path_to_map=path_to_map,
                            camera_setting_name=camera_setting_name,
                            vehicle_id=vehicle_id,
                            timestep_start=timestep_start,
                            timestep_end=timestep_end)
