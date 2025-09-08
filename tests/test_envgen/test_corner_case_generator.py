import json
import os
import shutil
import subprocess
import pytest
from pathlib import Path
from tqdm import tqdm
import multiprocessing
from typing import List, Tuple, Optional
from concurrent.futures import ProcessPoolExecutor, as_completed
import random

from terasim_envgen.core.terasim_corner_case_generator import run_single_experiment



def get_adversity_types(folder_name):
    """
    Get adversity types based on the folder name.
    
    Args:
        folder_name (str): Name of the map folder
    
    Returns:
        list: List of matching adversity types
    """
    vehicle_adversity_dir = Path("src/core/conf/adversity/vehicle")
    vru_adversity_dir = Path("src/core/conf/adversity/vru")
    all_types = []
    
    # Check if directory exists
    if not vehicle_adversity_dir.exists():
        raise FileNotFoundError(f"Vehicle directory {vehicle_adversity_dir} does not exist")
        
    # Check if there are any yaml files
    yaml_files = list(vehicle_adversity_dir.glob("*.yaml"))
    if not yaml_files:
        raise FileNotFoundError(f"No yaml files found in {vehicle_adversity_dir}")
    
    # Logic 1: Check if folder name contains 'highway' or 'roundabout'
    # Logic 2: enable vru collisions in roundabout and intersection, not highway
    if "highway" in folder_name.lower():
        # Match all highway adversity types
        for file in vehicle_adversity_dir.glob("highway_*.yaml"):
            all_types.append(f"vehicle:{file.stem}")
    elif "roundabout" in folder_name.lower():
        # Match all roundabout adversity types
        for file in vehicle_adversity_dir.glob("roundabout_*.yaml"):
            all_types.append(f"vehicle:{file.stem}")
        for file in vru_adversity_dir.glob("*.yaml"):
            all_types.append(f"vru:{file.stem}")
    else:
        # Logic 2: Default to intersection types if no specific match
        for file in vehicle_adversity_dir.glob("intersection_*.yaml"):
            all_types.append(f"vehicle:{file.stem}")
        for file in vru_adversity_dir.glob("*.yaml"):
            all_types.append(f"vru:{file.stem}")
    return all_types



def prepare_all_tasks(base_dir: Path) -> List[Tuple[str, str, str]]:
    """
    Prepare all tasks by combining all subdirs and their corresponding adversity types.
    
    Args:
        base_dir (Path): Base directory containing all map scenarios
    
    Returns:
        List[Tuple[str, str, str]]: List of all tasks to be executed
    """
    all_tasks = []
    
    # Get all subdirectories (each represents a map)
    subdirs = [d for d in base_dir.iterdir() if d.is_dir()]
    
    if not subdirs:
        print(f"No subdirectories found in {base_dir}")
        return all_tasks
    
    # For each subdir, get its adversity types and create tasks
    for subdir in subdirs:
        folder_name = subdir.name
        try:
            adversity_types = get_adversity_types(folder_name)
            print(f"Found {len(adversity_types)} adversity types for {folder_name}")
            
            # Create tasks for each adversity type
            for adv_type in adversity_types:
                new_adv_type = adv_type.split(":")[1]
                for i in range(100):
                    output_folder_name = f"{new_adv_type}_{i}"
                    task = (str(subdir), adv_type, output_folder_name)
                    all_tasks.append(task)
        except Exception as e:
            print(f"Error preparing tasks for {folder_name}: {e}")
    
    return all_tasks

def run_all_experiments_parallel(
    base_dir: Path,
    max_workers: Optional[int] = None,
    timeout: Optional[int] = None
) -> List[Tuple[str, str, bool]]:
    """
    Run all experiments in parallel, combining both subdirs and adversity types.
    
    Args:
        base_dir (Path): Base directory containing all map scenarios
        max_workers (Optional[int]): Maximum number of parallel processes
        timeout (Optional[int]): Timeout in seconds for each experiment
    
    Returns:
        List[Tuple[str, str, bool]]: List of all experiment results
    """
    # Prepare all tasks
    all_tasks = prepare_all_tasks(base_dir)
    # shuffle the tasks
    random.shuffle(all_tasks)
    total_tasks = len(all_tasks)
    print(f"\nPrepared {total_tasks} total tasks to execute")
    
    if not all_tasks:
        return []
    
    # Determine number of workers
    if max_workers is None:
        max_workers = max(1, multiprocessing.cpu_count() - 1)
    print(f"Using {max_workers} parallel workers")
    
    results = []
    # Use ProcessPoolExecutor for parallel execution
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        # Submit all tasks
        future_to_params = {
            executor.submit(run_single_experiment, params): params
            for params in all_tasks
        }
        
        # Process completed tasks with progress bar
        with tqdm(total=total_tasks, desc="Running experiments") as pbar:
            for future in as_completed(future_to_params, timeout=timeout):
                try:
                    result = future.result()
                    results.append(result)
                except Exception as e:
                    params = future_to_params[future]
                    print(f"Experiment failed for {params[1]} in {params[0]}: {str(e)}")
                    results.append((params[0], params[1], False))
                pbar.update(1)
    
    return results

def main():
    """
    Main function to process all map scenarios in parallel.
    """
    # Base directory for test outputs
    base_dir = Path("austin_cases")
    
    print(f"\nProcessing all map scenarios...")
    
    # Run all experiments in parallel
    all_results = run_all_experiments_parallel(
        base_dir,
        max_workers=None,  # Use default (CPU count - 1)
        timeout=3600      # 1 hour timeout per experiment
    )
    
    # Print summary
    successful = sum(1 for _, _, success in all_results if success)
    total = len(all_results)
    print(f"\nExecution Summary:")
    print(f"Total experiments: {total}")
    print(f"Successful: {successful}")
    print(f"Failed: {total - successful}")
    
    # Print detailed results
    print("\nDetailed Results:")
    for road_path, adv_type, success in all_results:
        status = "✓" if success else "✗"
        print(f"{status} {Path(road_path).name} - {adv_type}")

if __name__ == "__main__":
    main()
    # run_single_experiment((
    #     'test_output/Ann_Arbor_Michigan_USA_roundabout_96d51d77',  # road_path
    #     'vru:jaywalking',                                      # adversity_type
    #     'jaywalking_0',                                    # output_folder_name
    # ))