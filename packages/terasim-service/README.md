# TeraSim Service

A HTTP service for TeraSim simulation control and monitoring, built with FastAPI.

## Overview

TeraSim Service provides a comprehensive HTTP API for managing simulation instances, enabling remote control and real-time monitoring capabilities. It offers the following core functionalities:
- Start new simulation instances
- Check simulation status
- Control simulations (pause, resume, stop)
- Manually advance simulation steps (tick)
- Retrieve simulation states
- Send commands to agents

## Requirements

- Python 3.9+
- TeraSim and its dependencies

## Setup

1. Install Redis:
   For details, check [this link](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/)

2. Start Redis server:
   ```bash
   sudo systemctl enable redis-server
   sudo systemctl start redis-server
   # Verify Redis is running
   redis-cli ping
   # Should return "PONG"
   ```

3. Install TeraSim-Service package:
   ```bash
   poetry install
   ```

## Usage

There are several ways to start the TeraSim Service:

### Method 1: Using the command-line tool

After installation, you can use the provided command-line tool:

```bash
terasim-service
```

### Method 2: Using the Python module

You can run the service as a Python module:

```bash
python -m terasim_service
```

### Method 3: Running the example

You can also run the example file to start the service:

```bash
python examples/main.py
```

All of these methods will start the service on `http://localhost:8000` by default.

### Example REST API
The following is an overview of the REST API requests used to interact with the TeraSim simulation service.

1. **Start a Simulation**
   
   **Endpoint**: `POST /start_simulation`
   
   **Description**: Starts a new simulation with the specified configuration file.

   **Request**:
   ```
   POST http://localhost:8000/start_simulation
   Content-Type: application/json

   {
      "config_file": "examples/simulation_config.yaml", 
      "auto_run": false
   }
   ```
   Response:
   - **200 OK**: Returns the `simulation_id` for the newly started simulation.
   - Example:
   ```
   {
      "simulation_id": "0fd93635-b2ea-4b17-818a-55cad6bb6691",
      "message": "Simulation started"
   }
   ```

2. **Store Simulation ID**
   
   The `simulation_id` returned from the `start_simulation` request is stored in a variable for use in subsequent requests.

3. **Check Simulation Status**
   
   **Endpoint**: `GET /simulation_status/{simulationId}`
   
   **Description**: Retrieves the current status of the simulation.

   **Request**:
   ```
   GET http://localhost:8000/simulation_status/{{simulationId}}
   ```

   **Response**:
   - **200 OK**: Returns the current status of the simulation.
   - **Example**:
   ```
   {
      "id": "0fd93635-b2ea-4b17-818a-55cad6bb6691",
      "status": "running",
      "progress": 0.0
   }
   ```
   The simulation will have the following statuses and their meanings are shown below:
      | Simulation Status | Meaning                                                                 |
      |-------------------|-------------------------------------------------------------------------|
      | "initializing"         | The simulation has started for traffic flow initialization.             |
      | "wait_for_tick"     | The traffic flow has been initialized and the simulation is waiting for the "tick" command. |
      | "running"         | After receiving the "tick" command, the TeraSim simulation has started to advance for one step and it will take hundreds of milliseconds to finish the calculation. |
      | "ticked"         | The TeraSim simulation has finished the one-step advance and it is waiting for the next "tick" command. |
      | "finished"        | The TeraSim simulation has come to an end.                              |

4. **Stop Simulation**

   **Endpoint**: `POST /simulation_control/{simulationId}`

   **Description**: Stops the simulation.

   **Request**:
   ```
   POST http://localhost:8000/simulation_control/{{simulationId}}
   Content-Type: application/json

   {
      "command": "stop"
   }
   ```

5. **Tick the Simulation**
   
   **Endpoint**: `POST /simulation_tick/{simulationId}`
   
   **Description**: Advances the simulation by one step (only works when auto_run is false).

   **Request**:
   ```
   POST http://localhost:8000/simulation_tick/{{simulationId}}
   ```

6. **Get Simulation States**
   
   **Endpoint**: `GET /simulation/{simulationId}/state`
   
   **Description**: Retrieves states of all interested agents inside the simulation.

   **Request**:
   ```
   GET http://localhost:8000/simulation/{{simulationId}}/state
   ```

   **Response**:
   - **200 OK**: Returns the current states of the simulation.
   - **Example**: 
   ```
   {
      "header": {"timestamp": [...], "information": [...]},
      "simulation_time": [...],
      "agent_count": {"vehicle": [...], "vru": [...]},
      "agent_details": {
         "vehicle": {
            "BV_bike_flow_5.23": {
               "x": 58.01538972502436,
               "y": 60.92,
               "z": 0.0,
               "lat": 0.0,
               "lon": 0.0,
               "sumo_angle": 90.0, 
               "length": 1.6,
               "width": 0.65,
               "height": 1.7,
               "speed": 5.591859473845301,
               "type": "DEFAULT_BIKETYPE"
            },
            ...
         }
         "vru": [...],
      },
      "traffic_light_details": [...],
   }
   ```

   **Note**:
   - The `x`, `y`, and `z` coordinates of the agent are in the SUMO map coordinate system and represent the position of the agent's front bumper center.
   - The `sumo_angle` of the agent is measured in degrees, ranging from 0 to 360. A value of 0 represents north, and the sumo_angle increases clockwise.

7. **Control a Specific agent**

   **Endpoint**: `POST /simulation/{simulationId}/agent_command`
   
   **Description**: Sends a control command to a specific agent in the simulation.

   **Request**:
   ```
   POST http://localhost:8000/simulation/{{simulationId}}/agent_command
   Content-Type: application/json

   {
      "agent_id": "AV",
      "agent_type": "vehicle",
      "command_type": "set_state",
      "data": {
         "position": [112, 45.85],
         "lonlat": [-96.21622, 29.759548],
         
         "speed": 0.0
      }
   }
   ```
   **Note**: 
   - Either use `position` or `lonlat` in the data field, but not both at the same time
   - `position` represents coordinates in the SUMO map coordinate system
   - `lonlat` represents GPS coordinates in decimal degrees format (longitude, latitude)
   - For vehicles, `position` or `lonlat` denotes the front bumper center of the vehicle
   - `speed` is measured in meters per second (m/s)

   **Response**:
   - **200 OK**: Confirms that the command was successfully sent.
   - **Example**:
   ```
   {
      "message": "Agent command sent for agent AV"
   }
   ```

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For support, please open an issue in the GitHub repository.