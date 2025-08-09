"""
DataRecorderInfoExtractor - Data recorder based on CoSim logic

Reference CoSim plugin's data collection approach to implement complete simulation data recording functionality.
Store complete information for each timestep, including vehicles, VRUs, traffic lights, etc.
"""

import json
import time
from pathlib import Path
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from datetime import datetime

import numpy as np
from loguru import logger

from terasim.logger.infoextractor import InfoExtractor
from terasim.overlay import traci


@dataclass
class TimeStamp:
    """Timestamp data structure"""
    simulation_time: float      # SUMO simulation time (seconds)
    wall_clock_time: float      # Real time timestamp (Unix timestamp)
    step_id: int               # Simulation step ID
    dt: float                  # Time step length


@dataclass
class AgentStateSimplified:
    """Simplified agent state data structure, referencing CoSim's AgentStateSimplified"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    lon: float = 0.0
    lat: float = 0.0
    sumo_angle: float = 0.0
    orientation: float = 0.0
    speed: float = 0.0
    acceleration: float = 0.0
    angular_velocity: float = 0.0
    length: float = 0.0
    width: float = 0.0
    height: float = 0.0
    type: str = ""


@dataclass
class TrafficLightState:
    """Traffic light state data structure"""
    x: float = 0.0
    y: float = 0.0
    tls: str = ""           # Traffic light state string (r/y/g/R/Y/G)
    information: str = ""   # Program information JSON string


@dataclass
class SimulationSnapshot:
    """Simulation snapshot for a single timestep"""
    timestamp: TimeStamp
    agent_count: Dict[str, int]
    agent_details: Dict[str, Dict[str, AgentStateSimplified]]
    traffic_light_details: Dict[str, TrafficLightState]
    construction_zone_details: Optional[Dict[str, Any]] = None


class DataRecorderInfoExtractor(InfoExtractor):
    """Data recorder InfoExtractor implementation"""
    
    def __init__(self, env, config: Optional[Dict] = None):
        """Initialize data recorder
        
        Args:
            env: Environment object
            config: Configuration dictionary, including output path and other configs
        """
        super().__init__(env)
        
        # Configuration
        self.config = config or {}
        self.output_path = Path(self.config.get("output_path", "./recordings"))
        self.output_path.mkdir(parents=True, exist_ok=True)
        
        # Data storage
        self.simulation_metadata = {}
        self.snapshots: List[SimulationSnapshot] = []
        self.step_counter = 0
        
        # Cache previous orientations for angular velocity calculation
        self.last_orientations: Dict[str, tuple] = {}  # {agent_id: (orientation, time)}
        
        logger.info(f"DataRecorderInfoExtractor initialized, output_path: {self.output_path}")
    
    def add_initialization_info(self):
        """Add initialization information"""
        try:
            # Get basic simulation information
            self.simulation_metadata = {
                "simulation_id": f"sim_{int(time.time())}",
                "start_time": self.env.episode_info.get("start_time", time.time()),
                "scenario_name": getattr(self.env, 'scenario_name', 'unknown'),
                "network_file": str(getattr(self.env.simulator, 'sumo_net_file_path', '')),
                "sumo_version": "1.22.0",  # Get from pyproject.toml
                "created_at": datetime.now().isoformat(),
                "configuration": self.config
            }
            
            logger.info(f"Recording initialization info: {self.simulation_metadata['simulation_id']}")
            
        except Exception as e:
            logger.error(f"Error in add_initialization_info: {e}")
    
    def get_snapshot_info(self, control_info):
        """Get snapshot information - called every timestep"""
        try:
            # Check if simulator exists
            if not hasattr(self.env, 'simulator') or not self.env.simulator:
                return
            
            # Create timestamp
            simulation_time = traci.simulation.getTime()
            timestamp = TimeStamp(
                simulation_time=simulation_time,
                wall_clock_time=time.time(),
                step_id=self.step_counter,
                dt=getattr(self.env.simulator, 'step_length', 0.1)
            )
            
            # Collect agent data
            agent_count, agent_details = self._collect_agent_data(simulation_time)
            
            # Collect traffic light data
            traffic_light_details = self._collect_traffic_light_data()
            
            # Create snapshot
            snapshot = SimulationSnapshot(
                timestamp=timestamp,
                agent_count=agent_count,
                agent_details=agent_details,
                traffic_light_details=traffic_light_details,
                construction_zone_details=None  # Not implemented yet
            )
            
            self.snapshots.append(snapshot)
            self.step_counter += 1
            
            # Periodically save data to avoid memory overload
            if self.step_counter % 1000 == 0:
                logger.info(f"Recorded {self.step_counter} snapshots")
                
        except Exception as e:
            logger.error(f"Error in get_snapshot_info: {e}")
    
    def get_terminate_info(self, stop, reason, additional_info):
        """Get termination information and save all data"""
        try:
            # Update simulation metadata
            self.simulation_metadata.update({
                "end_time": self.env.episode_info.get("end_time", time.time()),
                "total_steps": self.step_counter,
                "stop_reason": reason,
                "additional_info": additional_info,
                "completed_at": datetime.now().isoformat()
            })
            
            # Save data
            self._save_recording_data()
            
            logger.info(f"Recording completed: {self.step_counter} steps saved")
            
        except Exception as e:
            logger.error(f"Error in get_terminate_info: {e}")
    
    def _collect_agent_data(self, simulation_time: float) -> tuple:
        """Collect agent data, referencing CoSim logic"""
        try:
            # Get all agent IDs
            vehicle_ids, vru_ids = self._get_vehicle_vru_ids()
            
            agent_count = {
                "vehicle": len(vehicle_ids),
                "vru": len(vru_ids),
            }
            
            agent_details = {
                "vehicle": {},
                "vru": {}
            }
            
            # Collect vehicle data
            for vid in vehicle_ids:
                agent_details["vehicle"][vid] = self._collect_vehicle_data(vid, simulation_time)
            
            # Collect VRU data
            for vru_id in vru_ids:
                agent_details["vru"][vru_id] = self._collect_vru_data(vru_id, simulation_time)
            
            return agent_count, agent_details
            
        except Exception as e:
            logger.error(f"Error collecting agent data: {e}")
            return {"vehicle": 0, "vru": 0}, {"vehicle": {}, "vru": {}}
    
    def _collect_vehicle_data(self, vehicle_id: str, simulation_time: float) -> AgentStateSimplified:
        """Collect single vehicle data"""
        try:
            vehicle_state = AgentStateSimplified()
            
            # Position information
            vehicle_state.x, vehicle_state.y, vehicle_state.z = traci.vehicle.getPosition3D(vehicle_id)
            vehicle_state.lon, vehicle_state.lat = traci.simulation.convertGeo(vehicle_state.x, vehicle_state.y)
            
            # Direction and speed
            vehicle_state.sumo_angle = traci.vehicle.getAngle(vehicle_id)
            vehicle_state.orientation = np.radians((90 - vehicle_state.sumo_angle) % 360)
            vehicle_state.speed = traci.vehicle.getSpeed(vehicle_id)
            vehicle_state.acceleration = traci.vehicle.getAcceleration(vehicle_id)
            
            # Geometric information
            vehicle_state.length = traci.vehicle.getLength(vehicle_id)
            vehicle_state.width = traci.vehicle.getWidth(vehicle_id)
            vehicle_state.height = traci.vehicle.getHeight(vehicle_id)
            vehicle_state.type = traci.vehicle.getTypeID(vehicle_id)
            
            # Calculate angular velocity
            vehicle_state.angular_velocity = self._calculate_angular_velocity(
                vehicle_id, vehicle_state.orientation, simulation_time
            )
            
            return vehicle_state
            
        except Exception as e:
            logger.error(f"Error collecting vehicle {vehicle_id} data: {e}")
            return AgentStateSimplified()
    
    def _collect_vru_data(self, vru_id: str, simulation_time: float) -> AgentStateSimplified:
        """Collect single VRU data"""
        try:
            vru_state = AgentStateSimplified()
            
            # Check if VRU is vehicle or pedestrian
            current_vehicle_list = traci.vehicle.getIDList()
            current_person_list = traci.person.getIDList()
            
            if vru_id in current_vehicle_list:
                # VRU is actually a vehicle
                vru_state.x, vru_state.y, vru_state.z = traci.vehicle.getPosition3D(vru_id)
                vru_state.lon, vru_state.lat = traci.simulation.convertGeo(vru_state.x, vru_state.y)
                vru_state.sumo_angle = traci.vehicle.getAngle(vru_id)
                vru_state.speed = traci.vehicle.getSpeed(vru_id)
                vru_state.acceleration = traci.vehicle.getAcceleration(vru_id)
                vru_state.length = traci.vehicle.getLength(vru_id)
                vru_state.width = traci.vehicle.getWidth(vru_id)
                vru_state.height = traci.vehicle.getHeight(vru_id)
                vru_state.type = traci.vehicle.getTypeID(vru_id)
                vru_state.orientation = np.radians((90 - vru_state.sumo_angle) % 360)
                
            elif vru_id in current_person_list:
                # VRU is actually a pedestrian
                vru_state.x, vru_state.y, vru_state.z = traci.person.getPosition3D(vru_id)
                vru_state.lon, vru_state.lat = traci.simulation.convertGeo(vru_state.x, vru_state.y)
                vru_state.sumo_angle = traci.person.getAngle(vru_id)
                vru_state.speed = traci.person.getSpeed(vru_id)
                vru_state.acceleration = traci.person.getAcceleration(vru_id)
                vru_state.length = traci.person.getLength(vru_id)
                vru_state.width = traci.person.getWidth(vru_id)
                vru_state.height = traci.person.getHeight(vru_id)
                vru_state.type = traci.person.getTypeID(vru_id)
                vru_state.orientation = np.radians((90 - vru_state.sumo_angle) % 360)
            
            # Calculate angular velocity
            vru_state.angular_velocity = self._calculate_angular_velocity(
                vru_id, vru_state.orientation, simulation_time
            )
            
            return vru_state
            
        except Exception as e:
            logger.error(f"Error collecting VRU {vru_id} data: {e}")
            return AgentStateSimplified()
    
    def _collect_traffic_light_data(self) -> Dict[str, TrafficLightState]:
        """Collect traffic light data"""
        try:
            traffic_lights = {}
            
            for tl_id in traci.trafficlight.getIDList():
                tl_state = TrafficLightState()
                
                # Basic information
                tl_state.x, tl_state.y = 0.0, 0.0  # Set to 0 for now, position info can be added later
                tl_state.tls = traci.trafficlight.getRedYellowGreenState(tl_id)
                
                # Get program information
                tls_information = {"programs": {}}
                
                try:
                    # Get detailed traffic light information
                    if hasattr(self.env.simulator, 'sumo_net'):
                        tls = self.env.simulator.sumo_net.getTLS(tl_id)
                        programs = tls.getPrograms()
                        
                        for program_id, program in programs.items():
                            program_parameters = program.getParams()
                            tls_information["programs"][program_id] = {
                                "parameters": program_parameters
                            }
                except Exception as e:
                    logger.debug(f"Could not get detailed info for traffic light {tl_id}: {e}")
                
                tl_state.information = json.dumps(tls_information)
                traffic_lights[tl_id] = tl_state
                
            return traffic_lights
            
        except Exception as e:
            logger.error(f"Error collecting traffic light data: {e}")
            return {}
    
    def _get_vehicle_vru_ids(self) -> tuple:
        """Get all vehicle and VRU IDs"""
        try:
            all_ids = list(set(traci.vehicle.getIDList() + traci.person.getIDList()))
            
            # Classify according to CoSim logic
            vehicle_ids = [id for id in all_ids if "BV" in id or "AV" in id]
            vru_ids = [id for id in all_ids if "VRU" in id]
            
            return vehicle_ids, vru_ids
            
        except Exception as e:
            logger.error(f"Error getting vehicle/VRU IDs: {e}")
            return [], []
    
    def _calculate_angular_velocity(self, agent_id: str, current_orientation: float, current_time: float) -> float:
        """Calculate angular velocity"""
        try:
            if agent_id not in self.last_orientations:
                self.last_orientations[agent_id] = (current_orientation, current_time)
                return 0.0
            
            last_orientation, last_time = self.last_orientations[agent_id]
            dt = current_time - last_time
            
            if dt > 0:
                # Calculate angle difference, handle periodicity
                dtheta = np.arctan2(
                    np.sin(current_orientation - last_orientation), 
                    np.cos(current_orientation - last_orientation)
                )
                angular_velocity = dtheta / dt
            else:
                angular_velocity = 0.0
            
            # Update cache
            self.last_orientations[agent_id] = (current_orientation, current_time)
            
            return angular_velocity
            
        except Exception as e:
            logger.error(f"Error calculating angular velocity for {agent_id}: {e}")
            return 0.0
    
    def _save_recording_data(self):
        """Save recording data to JSON file"""
        try:
            # Create complete recording data structure
            recording_data = {
                "metadata": self.simulation_metadata,
                "snapshots": [self._snapshot_to_dict(snapshot) for snapshot in self.snapshots]
            }
            
            # Generate filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"simulation_recording_{timestamp}.json"
            filepath = self.output_path / filename
            
            # Save JSON file
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(recording_data, f, indent=2, ensure_ascii=False)
            
            logger.info(f"Recording saved to: {filepath}")
            
            # Save summary information
            summary = {
                "simulation_id": self.simulation_metadata.get("simulation_id", "unknown"),
                "total_steps": len(self.snapshots),
                "duration": self.simulation_metadata.get("end_time", 0) - self.simulation_metadata.get("start_time", 0),
                "file_path": str(filepath),
                "created_at": datetime.now().isoformat()
            }
            
            summary_path = self.output_path / f"summary_{timestamp}.json"
            with open(summary_path, 'w', encoding='utf-8') as f:
                json.dump(summary, f, indent=2, ensure_ascii=False)
            
        except Exception as e:
            logger.error(f"Error saving recording data: {e}")
    
    def _snapshot_to_dict(self, snapshot: SimulationSnapshot) -> Dict[str, Any]:
        """Convert snapshot to dictionary for JSON serialization"""
        try:
            return {
                "timestamp": asdict(snapshot.timestamp),
                "agent_count": snapshot.agent_count,
                "agent_details": {
                    agent_type: {
                        agent_id: asdict(agent_state) 
                        for agent_id, agent_state in agents.items()
                    }
                    for agent_type, agents in snapshot.agent_details.items()
                },
                "traffic_light_details": {
                    tl_id: asdict(tl_state)
                    for tl_id, tl_state in snapshot.traffic_light_details.items()
                },
                "construction_zone_details": snapshot.construction_zone_details
            }
        except Exception as e:
            logger.error(f"Error converting snapshot to dict: {e}")
            return {}