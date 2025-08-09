#!/usr/bin/env python3
"""
Standalone SUMO test script for debugging vehicle placement issues.
This script loads a SUMO configuration and uses traci to control the simulation,
adding/moving a static vehicle to specific coordinates at each step.

Usage:
    python test_sumo_standalone.py

Requirements:
    - SUMO installed with traci support
    - sumocfg file at: examples/texas_highway_maps/2/simulation.sumocfg
"""

# import traci
from terasim.overlay import traci
# import libsumo as traci
import sys
import os
from pathlib import Path

def test_sumo_with_static_vehicle():
    """
    Test SUMO simulation with manual vehicle placement using traci.
    """
    
    # Configuration
    SUMO_CONFIG = "examples/texas_highway_maps/2/simulation.sumocfg"
    
    # Problematic coordinates from your yaml config
    TARGET_X = 1090.68
    TARGET_Y = 621.98
    TARGET_ANGLE = 250  # degrees
    
    VEHICLE_ID = "test_static_vehicle"
    VEHICLE_TYPE = "DEFAULT_VEHTYPE"
    
    # Check if config file exists
    if not Path(SUMO_CONFIG).exists():
        print(f"ERROR: SUMO config file not found: {SUMO_CONFIG}")
        print("Please run this script from the TeraSim-Deploy directory")
        return False
    
    try:
        print("="*60)
        print("Starting SUMO with traci...")
        print(f"Config file: {SUMO_CONFIG}")
        print(f"Target coordinates: x={TARGET_X}, y={TARGET_Y}, angle={TARGET_ANGLE}")
        print("="*60)
        
        # Start SUMO with GUI for visual debugging
        sumo_cmd = [
            "sumo-gui",  # Use GUI for visual feedback
            "-c", SUMO_CONFIG,
            # "--trace-file", "trace.xml"
            # "--verbose",
            # "--log-level", "DEBUG",
            # "--ignore-route-errors",
            # "--ignore-accidents",
            # "--start",  # Start simulation immediately
            # "--quit-on-end"  # Close when simulation ends
        ]
        
        print(f"SUMO command: {' '.join(sumo_cmd)}")
        traci.start(sumo_cmd)
        
        print("SUMO started successfully!")
        
        # Get simulation info
        print(f"Simulation time: {traci.simulation.getTime()}")
        print(f"Network boundary: {traci.simulation.getNetBoundary()}")
        
        step = 0
        max_steps = 1000  # Limit for testing
        
        vehicle_added = False
        
        while step < max_steps and traci.simulation.getMinExpectedNumber() > 0:
            print(f"\n--- Step {step} (time: {traci.simulation.getTime()}) ---")
            
            # Step 1: Add vehicle only once in the first step
            if not vehicle_added and VEHICLE_ID not in traci.vehicle.getIDList():
                print(f"Vehicle {VEHICLE_ID} not found, adding to route")
                try:
                    print(f"Adding vehicle {VEHICLE_ID}...")
                    routes = traci.route.getIDList()
                    if routes:
                        traci.vehicle.add(VEHICLE_ID, routes[0], typeID=VEHICLE_TYPE)
                        traci.vehicle.moveToXY(VEHICLE_ID, "", -1, TARGET_X, TARGET_Y, TARGET_ANGLE, 2)
                        print(f"Vehicle {VEHICLE_ID} added successfully with route {routes[0]}")
                        vehicle_added = True
                    else:
                        print("No routes available")
                except Exception as e:
                    print(f"Failed to add vehicle: {e}")
            
            # Step 2: Move vehicle to target position every step (if vehicle exists)
            if VEHICLE_ID in traci.vehicle.getIDList():
                print(f"Vehicle {VEHICLE_ID} exists, moving to target position")
                try:
                    # Get current position
                    current_pos = traci.vehicle.getPosition(VEHICLE_ID)
                    print(f"Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
                    
                    # Move to target position
                    print(f"Moving to target: ({TARGET_X}, {TARGET_Y})")
                    traci.vehicle.moveToXY(
                        VEHICLE_ID, 
                        "",  # edge ID (empty = anywhere)
                        -1,  # lane index (-1 = any lane)
                        TARGET_X, 
                        TARGET_Y, 
                        TARGET_ANGLE,
                        2  # keepRoute flag (2 = ignore route, keep position)
                    )
                    
                    # Verify new position
                    new_pos = traci.vehicle.getPosition(VEHICLE_ID)
                    print(f"New position: ({new_pos[0]:.2f}, {new_pos[1]:.2f})")
                    
                    # Check if we successfully moved to target
                    distance = ((new_pos[0] - TARGET_X)**2 + (new_pos[1] - TARGET_Y)**2)**0.5
                    print(f"Distance from target: {distance:.2f} meters")
                    
                    if distance < 1.0:  # Within 1 meter
                        print("✓ Vehicle successfully positioned at target!")
                    else:
                        print("⚠ Vehicle position differs from target")
                        
                except Exception as e:
                    print(f"Failed to move vehicle: {e}")
                    # Don't break, continue trying
            
            # Print current vehicles in simulation
            vehicles = traci.vehicle.getIDList()
            print(f"Total vehicles in simulation: {len(vehicles)}")
            print(f"Vehicles: {vehicles}")
            
            # Advance simulation
            traci.simulationStep()
            step += 1
            
            # Break after testing vehicle movement for several steps
            if vehicle_added and step > 1000:
                print(f"\nTest completed after {step} steps")
                if VEHICLE_ID in traci.vehicle.getIDList():
                    final_pos = traci.vehicle.getPosition(VEHICLE_ID)
                    print(f"Final vehicle position: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
                    final_distance = ((final_pos[0] - TARGET_X)**2 + (final_pos[1] - TARGET_Y)**2)**0.5
                    print(f"Final distance from target: {final_distance:.2f} meters")
                break
        
        print(f"\nSimulation completed after {step} steps")
        
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        try:
            traci.close()
            print("SUMO closed successfully")
        except:
            pass
    
    return True

def test_coordinates_validity():
    """
    Test if the coordinates are within the network bounds.
    """
    SUMO_CONFIG = "examples/texas_highway_maps/2/simulation.sumocfg"
    TARGET_X = 1089.77
    TARGET_Y = 609.98
    
    try:
        print("Testing coordinate validity...")
        
        # Start SUMO without GUI for quick check
        traci.start(["sumo", "-c", SUMO_CONFIG, "--no-step-log"])
        
        # Get network boundaries
        boundary = traci.simulation.getNetBoundary()
        print(f"Network boundary: {boundary}")
        print(f"X range: {boundary[0]} to {boundary[2]}")
        print(f"Y range: {boundary[1]} to {boundary[3]}")
        
        # Check if target coordinates are within bounds
        x_in_bounds = boundary[0] <= TARGET_X <= boundary[2]
        y_in_bounds = boundary[1] <= TARGET_Y <= boundary[3]
        
        print(f"Target coordinates ({TARGET_X}, {TARGET_Y}):")
        print(f"  X in bounds: {x_in_bounds}")
        print(f"  Y in bounds: {y_in_bounds}")
        print(f"  Overall valid: {x_in_bounds and y_in_bounds}")
        
        # Try to find nearest edge to the coordinates
        try:
            edges = traci.edge.getIDList()
            print(f"Total edges in network: {len(edges)}")
            
            # Sample a few edges to see typical coordinates
            for i, edge_id in enumerate(edges[:5]):
                try:
                    shape = traci.edge.getShape(edge_id)
                    if shape:
                        print(f"Edge {edge_id}: start={shape[0]}, end={shape[-1]}")
                except:
                    pass
        except Exception as e:
            print(f"Could not analyze edges: {e}")
        
        traci.close()
        return x_in_bounds and y_in_bounds
        
    except Exception as e:
        print(f"Coordinate test failed: {e}")
        try:
            traci.close()
        except:
            pass
        return False

if __name__ == "__main__":
    print("SUMO Standalone Test Script")
    print("==========================")
    
    # First test coordinate validity
    print("\n1. Testing coordinate validity...")
    coords_valid = test_coordinates_validity()
    
    print(f"\n2. Running full simulation test...")
    success = test_sumo_with_static_vehicle()
    
    print(f"\n{'='*60}")
    print("RESULTS:")
    print(f"Coordinates valid: {coords_valid}")
    print(f"Simulation test: {'PASSED' if success else 'FAILED'}")
    print(f"{'='*60}")
    
    if not success:
        sys.exit(1)