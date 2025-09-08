# pyOpenDRIVE Data Structure Guide

This document provides a detailed breakdown of the data structures available in `pyOpenDRIVE` after loading an `.xodr` file. It focuses on how to access and interpret the map data using the Python API, with a special emphasis on retrieving lane coordinates and junction connectivity.

## 1. Loading the Map: The Root Object

All data access begins by loading an OpenDRIVE file, which returns a `PyOpenDriveMap` object. This is the root of the data hierarchy.

```python
from pyOpenDRIVE.OpenDriveMap import PyOpenDriveMap

# Load the map file (path must be encoded)
xodr_file_path = "path/to/your_map.xodr".encode()
odr_map = PyOpenDriveMap(xodr_file_path)
```

---

## 2. Top-Level Object: `PyOpenDriveMap`

This object represents the entire map.

### Key Methods:
- `odr_map.get_roads() -> list[PyRoad]`: Returns a list of all road objects in the map.
- `odr_map.get_junctions() -> list[PyJunction]`: Returns a list of all junction objects in the map.
- `odr_map.get_road_network_mesh(eps: float) -> PyMesh`: Generates a 3D mesh of the entire road network, useful for visualization. `eps` is the sampling precision (smaller value = higher detail).

---

## 3. Road Object: `PyRoad`

Represents a single road as defined in the OpenDRIVE standard.

### How to Access:
```python
roads = odr_map.get_roads()
my_road = roads[0]  # Get the first road
```

### Key Attributes:
- `my_road.id -> str`: The unique identifier of the road.
- `my_road.length -> float`: The length of the road's reference line in meters.
- `my_road.junction -> str`: The ID of the junction this road belongs to. Returns "-1" if it's not part of a junction.

### Core Methods:
- `my_road.get_lanesections() -> list[PyLaneSection]`: Retrieves a list of lane sections within this road.
- `my_road.get_xyz(s: float, t: float, h: float) -> tuple[float, float, float]`: A crucial function to convert from the road's local coordinate system (s, t, h) to global Cartesian coordinates (x, y, z).
    - `s`: Longitudinal distance along the reference line.
    - `t`: Lateral offset perpendicular to the reference line.
    - `h`: Height offset perpendicular to the s-t surface.
- `my_road.get_lane_mesh(lane: PyLane, eps: float) -> PyMesh`: **The primary method for getting lane coordinates.** It generates a `PyMesh` object for a specified lane.

---

## 4. Lane Section Object: `PyLaneSection`

A segment of a road that has a consistent lane layout.

### How to Access:
```python
lane_sections = my_road.get_lanesections()
my_lane_section = lane_sections[0]
```

### Key Attributes:
- `my_lane_section.s0 -> float`: The starting `s` position of this lane section along the road's reference line.

### Core Methods:
- `my_lane_section.get_lanes() -> list[PyLane]`: Returns a list of all lanes within this section.

---

## 5. Lane Object: `PyLane`

Represents a single lane, the fundamental unit for driving.

### How to Access:
```python
lanes = my_lane_section.get_lanes()
my_lane = lanes[0]
```

### Key Attributes:
- `my_lane.id -> int`: The lane identifier (e.g., -1, 1, 2). `0` is the reference line, negative IDs are typically to the right, and positive IDs are to the left.
- `my_lane.type -> str`: The type of lane (e.g., 'driving', 'shoulder', 'border').

### How to Get Lane Coordinates

You cannot directly query a list of coordinates for a lane. Instead, you must generate a **3D mesh** for the lane, from which you can extract the vertex coordinates. The `PyMesh` object contains a `.vertices` attribute.

**Example:**
```python
# Assuming my_road and my_lane are already defined
# eps controls the precision; smaller values result in more vertices
eps = 0.5 

# Generate the mesh for the lane using the parent road object
lane_mesh = my_road.get_lane_mesh(my_lane, eps)

# The .vertices attribute is a list of (x, y, z) tuples
lane_coordinates = lane_mesh.vertices

print(f"Lane {my_lane.id} contains {len(lane_coordinates)} vertex coordinates.")
# print(lane_coordinates[:5]) # Print the first 5 coordinates
```

---

## 6. Junction Object: `PyJunction`

Represents an intersection or junction where multiple roads connect.

### How to Access:
```python
junctions = odr_map.get_junctions()
my_junction = junctions[0]
```

### Key Attributes:
- `my_junction.id -> str`: The unique identifier of the junction.
- `my_junction.name -> str`: The name of the junction.

### Core Methods:
- `my_junction.id_to_connection -> dict[str, PyJunctionConnection]`: **The key to accessing connection details.** This is a dictionary mapping a connection ID to its corresponding `PyJunctionConnection` object.

---

## 7. Junction Connection Object: `PyJunctionConnection`

Defines a specific path through a junction, connecting an incoming road to a connecting road.

### How to Access:
```python
# .id_to_connection is a dictionary
connections = my_junction.id_to_connection.values()
my_connection = list(connections)[0]
```

### Key Attributes:
- `my_connection.id -> str`: The ID of this specific connection.
- `my_connection.incoming_road -> str`: The ID of the road entering this connection.
- `my_connection.connecting_road -> str`: The ID of the internal road that represents the connection's path.
- `my_connection.lane_links -> set[PyJunctionLaneLink]`: **The critical lane-level connectivity information.** This is a set of `PyJunctionLaneLink` objects.

---

## 8. Lane Link Object: `PyJunctionLaneLink`

This object defines the direct link from a lane on an incoming road to a lane on the connecting road.

### How to Access:
```python
# .lane_links is a set
lane_links = my_connection.lane_links
my_lane_link = list(lane_links)[0]
```

### Key Attributes:
- `my_lane_link.from -> int`: The lane ID on the `incoming_road`.
- `my_lane_link.to -> int`: The lane ID on the `connecting_road`.

### How to Get Full Junction Connectivity

**Example:**
```python
# Assuming my_junction is already defined
print(f"Analyzing Junction: {my_junction.id}")

# Iterate through all connections in the junction
for conn_id, connection in my_junction.id_to_connection.items():
    print(f"  Connection ID: {conn_id}")
    print(f"    Path: From road '{connection.incoming_road}' via internal road '{connection.connecting_road}'.")
    
    # Iterate through all lane-to-lane links within that connection
    if not connection.lane_links:
        print("    This connection has no lane-level links.")
    else:
        for lane_link in connection.lane_links:
            print(f"      - Lane Link: From lane {lane_link.from} -> To lane {lane_link.to}")

```
