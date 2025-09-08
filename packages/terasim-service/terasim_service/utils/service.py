import requests


def start_terasim(host, port, command):
    url = f"http://{host}:{port}/start_simulation"
    response = requests.post(url, json=command)
    return response.json()

def control_terasim(host, port, simulationId, command):
    url = f"http://{host}:{port}/simulation_control/{simulationId}"
    response = requests.post(url, json=command)
    return response.json()

def stop_terasim(host, port, simulationId):
    response = control_terasim(host, port, simulationId, {"command": "stop"})
    return response

def tick_terasim(host, port, simulationId):
    url = f"http://{host}:{port}/simulation_tick/{simulationId}"
    response = requests.post(url)
    return response.json()

def get_terasim_status(host, port, simulationId):
    url = f"http://{host}:{port}/simulation_status/{simulationId}"
    response = requests.get(url)
    return response.json()

def get_terasim_states(host, port, simulationId):
    url = f"http://{host}:{port}/simulation/{simulationId}/state"
    response = requests.get(url)
    return response.json()

def control_agent(host, port, simulationId, command):
    url = f"http://{host}:{port}/simulation/{simulationId}/agent_command"
    response = requests.post(url, json=command)
    return response.json()
