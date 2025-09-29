import requests
import json
import time


def get_av_state(base_url, simulation_id):
    av_route_response = requests.get(f"{base_url}/av_route/{simulation_id}")
    state_response = requests.get(f"{base_url}/simulation/{simulation_id}/state")
    state_json = state_response.json()
    av_state = state_json["agent_details"]["vehicle"]["AV"]
    return {
        "state": av_state,
        "route": av_route_response.json()
    }



def run_simulation(config_file="examples/scenarios/police_pullover_case.yaml", auto_run=False, initialize_timeout=3600, tick_timeout=3600, enable_viz=False, viz_port=8050, viz_update_freq=1):
    """
    Run simulation and provide HTTP API interface calls
    
    Args:
        config_file (str): Path to configuration file
        auto_run (bool): Whether to run simulation automatically
        initialize_timeout (int): Timeout for initialization in seconds
        tick_timeout (int): Timeout for each tick in seconds
        enable_viz (bool): Whether to enable visualization
        viz_port (int): Port for visualization server
        viz_update_freq (int): Visualization update frequency
    
    Returns:
        dict: Simulation results
    """
    base_url = "http://localhost:8000"
    
    # Start simulation
    start_response = requests.post(
        f"{base_url}/start_simulation",
        json={
            "config_file": config_file,
            "auto_run": auto_run
        },
        params={
            "enable_viz": enable_viz,
            "viz_port": viz_port,
            "viz_update_freq": viz_update_freq
        }
    )
    response_data = start_response.json()
    simulation_id = response_data["simulation_id"]
    
    # Print visualization URL if enabled
    if enable_viz and "visualization_url" in response_data:
        print(f"🎨 Visualization available at: {response_data['visualization_url']}")
        print(f"   Open this URL in your browser to see real-time visualization")

    start_time = time.time()
    while True:
        # Get simulation status
        try:
            status_response = requests.get(f"{base_url}/simulation_status/{simulation_id}")
            # print(f"Simulation status: {status_response.json()}")
            # Break if simulation is waiting for tick
            if status_response.json()["status"] == "wait_for_tick":
                break
            if time.time() - start_time > initialize_timeout:  # 10 seconds timeout
                print("Simulation initialization timeout, stopping...")
                requests.post(f"{base_url}/simulation_control/{simulation_id}", json={"command": "stop"})
                raise TimeoutError("Simulation initialization timeout")
        except Exception as e:
            print(f"Simulation status not ready: {e}")
            time.sleep(0.01)

    # Get AV state
    # av_state = get_av_state(base_url, simulation_id)
    
    while True:
        # Tick simulation to advance one step
        tick_response = requests.post(f"{base_url}/simulation_tick/{simulation_id}")
        # get simulation status
        start_time = time.time()
        while True:
            status_response = requests.get(f"{base_url}/simulation_status/{simulation_id}")
            if status_response.json()["status"] == "ticked" or status_response.json()["status"] == "finished":
                break
            if time.time() - start_time > tick_timeout:
                print("Simulation stuck for more than 1 second, stopping...")
                requests.post(f"{base_url}/simulation_control/{simulation_id}", json={"command": "stop"})
                return {"error": "Simulation timeout"}
            time.sleep(0.01)
        state_response = requests.get(f"{base_url}/simulation/{simulation_id}/state")
        # print(f"Simulation state: {state_response.json()}")
        if status_response.json()["status"] == "finished":
            break
    
    # Get simulation results
    result_response = requests.get(f"{base_url}/simulation_result/{simulation_id}")
    return result_response.json()

if __name__ == "__main__":
    # Example 1: Run simulation without visualization (default)
    # result = run_simulation(config_file="config_yamls/config_yaml_with_static/config_2_002.yaml")
    
    # Example 2: Run simulation with visualization
    result = run_simulation(
        config_file="texas_example/test_configs/cutin.yaml",
        enable_viz=True,  # Enable visualization
        viz_port=8050,    # Visualization port
        viz_update_freq=2 # Update every 2 simulation steps (reduce load)
    )
    
    print(f"Final simulation result: {result}")
