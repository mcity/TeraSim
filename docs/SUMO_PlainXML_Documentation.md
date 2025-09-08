# SUMO PlainXML Network Format Documentation

## Overview

The PlainXML format is SUMO's human-readable network description format that allows creating and editing road networks before converting them into the binary network format used by the simulation. This format consists of several XML files that describe different aspects of the network topology and properties.

## File Types

### 1. Node Files (`.nod.xml`)
Define intersections and junctions in the network.

#### Structure
```xml
<nodes>
    <node id="node_id" x="0.0" y="0.0" type="priority" .../>
    ...
</nodes>
```

#### Node Attributes
| Attribute | Type | Description | Default |
|-----------|------|-------------|---------|
| `id` | string | Unique identifier for the node | required |
| `x` | float | X-coordinate position (meters) | required |
| `y` | float | Y-coordinate position (meters) | required |
| `z` | float | Z-coordinate (elevation in meters) | 0.0 |
| `type` | enum | Junction type | - |
| `tlType` | string | Traffic light algorithm type | static |
| `tlLayout` | string | Traffic light layout algorithm | - |
| `tl` | string | Traffic light ID this node belongs to | - |
| `radius` | float | Turning radius for all connections (meters) | 1.5 |
| `shape` | position list | Custom shape for junction | - |
| `keepClear` | bool | Whether junction must be kept clear | true |
| `rightOfWay` | string | Algorithm for computing right of way | default |
| `fringe` | enum | Network fringe type indicator | - |
| `name` | string | Optional name for visualization | - |
| `controlledInner` | string list | Inner edges controlled by traffic light | - |

#### Junction Types
- `priority`: Right-before-left rule
- `traffic_light`: Controlled by traffic signals
- `right_before_left`: Right-before-left without priority
- `left_before_right`: Left-before-right rule
- `unregulated`: No right-of-way rules
- `priority_stop`: Priority with stop signs
- `allway_stop`: All-way stop intersection
- `rail_signal`: Railway signal
- `zipper`: Zipper merge
- `rail_crossing`: Railway crossing
- `traffic_light_unregulated`: Traffic light without rules when off

### 2. Edge Files (`.edg.xml`)
Define roads and streets connecting nodes.

#### Structure
```xml
<edges>
    <edge id="edge_id" from="node1" to="node2" priority="1" numLanes="2" speed="13.89">
        <lane index="0" speed="10.0" allow="pedestrian bicycle"/>
        <lane index="1" disallow="pedestrian bicycle"/>
    </edge>
    ...
</edges>
```

#### Edge Attributes
| Attribute | Type | Description | Default |
|-----------|------|-------------|---------|
| `id` | string | Unique identifier for the edge | required |
| `from` | string | ID of source node | required |
| `to` | string | ID of destination node | required |
| `type` | string | Reference to edge type definition | - |
| `numLanes` | int | Number of lanes | 1 |
| `speed` | float | Maximum speed (m/s) | 13.89 |
| `priority` | int | Priority for right-of-way computation | -1 |
| `length` | float | Edge length (meters) | computed |
| `shape` | position list | Edge geometry as polyline | straight line |
| `spreadType` | enum | How lanes are positioned | right |
| `allow` | string list | Allowed vehicle classes | all |
| `disallow` | string list | Disallowed vehicle classes | - |
| `width` | float | Lane width (meters) | -1 (default) |
| `endOffset` | float | Distance to stop line from junction | 0.0 |
| `sidewalkWidth` | float | Width of sidewalk (meters) | -1 (none) |
| `bikeLaneWidth` | float | Width of bike lane (meters) | -1 (none) |
| `distance` | float | Distance offset for kilometrage | 0.0 |
| `stopOffset` | float | Offset for stopping positions | 0.0 |
| `stopException` | string list | Vehicle classes exempt from stopOffset | - |

#### Lane-Specific Attributes
Lanes within an edge can have individual attributes:
- `index`: Lane number (0 = rightmost)
- `speed`: Lane-specific speed limit
- `allow`/`disallow`: Lane-specific permissions
- `width`: Lane-specific width
- `acceleration`: Whether lane is acceleration lane
- `shape`: Lane-specific geometry
- `type`: Lane type identifier
- `changeLeft`/`changeRight`: Lane change restrictions

#### Spread Types
- `right`: Lanes spread to the right of the edge shape
- `center`: Lanes centered on the edge shape
- `roadCenter`: Lanes spread to both sides

### 3. Connection Files (`.con.xml`)
Define how edges and lanes connect at junctions.

#### Structure
```xml
<connections>
    <connection from="edge1" to="edge2" fromLane="0" toLane="0" .../>
    <prohibition prohibitor="edge1->edge2" prohibited="edge3->edge4"/>
    <crossing node="node_id" edges="edge1 edge2" priority="true" width="4.0"/>
    <walkingArea node="node_id" edges="edge1 edge2" shape="..."/>
</connections>
```

#### Connection Attributes
| Attribute | Type | Description | Default |
|-----------|------|-------------|---------|
| `from` | string | Source edge ID | required |
| `to` | string | Destination edge ID | required |
| `fromLane` | int | Source lane index | required |
| `toLane` | int | Destination lane index | required |
| `pass` | bool | Whether vehicles may pass waiting vehicles | false |
| `keepClear` | bool | Whether to keep intersection clear | true |
| `contPos` | float | Position of internal junction | computed |
| `visibility` | float | Visibility distance for approaching | 4.5 |
| `speed` | float | Maximum speed through connection | computed |
| `length` | float | Connection length | computed |
| `shape` | position list | Custom connection shape | computed |
| `uncontrolled` | bool | Whether connection is always green at TL | false |
| `allow` | string list | Allowed vehicle classes | - |
| `disallow` | string list | Disallowed vehicle classes | - |
| `changeLeft` | string list | Vehicle classes allowed to change left | - |
| `changeRight` | string list | Vehicle classes allowed to change right | - |
| `indirect` | bool | Whether connection is an indirect left turn | false |
| `type` | string | Connection type identifier | - |

#### Crossing Attributes (Pedestrian)
| Attribute | Type | Description |
|-----------|------|-------------|
| `node` | string | Junction ID |
| `edges` | string list | Edges to cross |
| `priority` | bool | Whether crossing has priority |
| `width` | float | Crossing width (meters) |
| `shape` | position list | Custom crossing shape |
| `discard` | bool | Whether to remove crossing |

### 4. Type Files (`.typ.xml`)
Define reusable edge and lane type templates.

#### Structure
```xml
<types>
    <type id="highway.motorway" priority="1" numLanes="3" speed="44.44">
        <restriction vClass="pedestrian" speed="0"/>
    </type>
    <laneType id="sidewalk" width="2.0" allow="pedestrian"/>
</types>
```

#### Type Attributes
Edge types can have all edge attributes as defaults, plus:
- `id`: Type identifier
- `oneway`: Whether type creates one-way edges
- `discard`: Whether to discard edges of this type

### 5. Traffic Light Files (`.tll.xml`)
Define traffic light programs and signal groups.

#### Structure
```xml
<tlLogics>
    <tlLogic id="tl_id" type="static" programID="0" offset="0">
        <phase duration="31" state="GGgrrrGGgrrr"/>
        <phase duration="5" state="yygrrryygrrr"/>
        <phase duration="31" state="rrrGGGrrrGGG"/>
        <phase duration="5" state="rrryyyrrryyy"/>
    </tlLogic>
    <connection from="edge1" to="edge2" fromLane="0" toLane="0" tl="tl_id" linkIndex="0"/>
</tlLogics>
```

#### Traffic Light Attributes
| Attribute | Type | Description |
|-----------|------|-------------|
| `id` | string | Traffic light ID |
| `type` | string | Control type (static, actuated, etc.) |
| `programID` | string | Program identifier |
| `offset` | int | Time offset for coordination |

#### Phase Attributes
| Attribute | Type | Description |
|-----------|------|-------------|
| `duration` | float | Phase duration in seconds |
| `state` | string | Signal states (G=green, r=red, y=yellow, etc.) |
| `minDur` | float | Minimum duration for actuated control |
| `maxDur` | float | Maximum duration for actuated control |
| `name` | string | Optional phase name |
| `next` | int list | Next phases for actuated control |

## Conversion to Network

Convert PlainXML files to SUMO network:

```bash
netconvert --node-files=nodes.nod.xml \
           --edge-files=edges.edg.xml \
           --connection-files=connections.con.xml \
           --type-files=types.typ.xml \
           --tllogic-files=tllogic.tll.xml \
           --output-file=network.net.xml
```

Simplified with all files in one directory:
```bash
netconvert --plain-xml-prefix=mynet --output-file=mynet.net.xml
```

## Vehicle Classes

Common vehicle classes for permissions:
- `passenger`: Passenger cars
- `bus`: Buses
- `truck`: Trucks
- `bicycle`: Bicycles
- `pedestrian`: Pedestrians
- `tram`: Trams
- `rail`: Trains
- `motorcycle`: Motorcycles
- `emergency`: Emergency vehicles
- `taxi`: Taxis
- `delivery`: Delivery vehicles

## Special Considerations

### Roundabouts
Mark edges as roundabout parts:
```xml
<roundabout nodes="node1 node2 node3 node4" edges="edge1 edge2 edge3 edge4"/>
```

### Opposite Direction Driving
For edges with opposite direction lanes:
```xml
<edge id="edge1" from="A" to="B">
    <neigh lane="opposite_edge1_0"/>
</edge>
```

### District (TAZ) Definition
```xml
<TAZ id="district1" edges="edge1 edge2 edge3">
    <tazSource id="edge1" weight="0.5"/>
    <tazSink id="edge2" weight="0.3"/>
</TAZ>
```

## Validation

Validate PlainXML files:
```bash
netconvert --plain.validate-input \
           --node-files=nodes.nod.xml \
           --edge-files=edges.edg.xml
```

## Common Use Cases

### 1. Simple Intersection
```xml
<!-- nodes.nod.xml -->
<nodes>
    <node id="C" x="0" y="0" type="priority"/>
    <node id="N" x="0" y="100"/>
    <node id="E" x="100" y="0"/>
    <node id="S" x="0" y="-100"/>
    <node id="W" x="-100" y="0"/>
</nodes>

<!-- edges.edg.xml -->
<edges>
    <edge id="NC" from="N" to="C" numLanes="2"/>
    <edge id="EC" from="E" to="C" numLanes="2"/>
    <edge id="SC" from="S" to="C" numLanes="2"/>
    <edge id="WC" from="W" to="C" numLanes="2"/>
    <edge id="CN" from="C" to="N" numLanes="2"/>
    <edge id="CE" from="C" to="E" numLanes="2"/>
    <edge id="CS" from="C" to="S" numLanes="2"/>
    <edge id="CW" from="C" to="W" numLanes="2"/>
</edges>
```

### 2. Highway On-Ramp
```xml
<edges>
    <edge id="highway" from="A" to="B" numLanes="3" speed="33.33" priority="1"/>
    <edge id="ramp" from="R" to="M" numLanes="1" speed="16.67" priority="-1"/>
</edges>

<connections>
    <connection from="ramp" to="highway" fromLane="0" toLane="0" pass="true"/>
</connections>
```

## Important Notes

1. **Coordinate System**: Uses Cartesian coordinates in meters
2. **Edge Direction**: Edges are unidirectional; bidirectional roads need two edges
3. **Lane Indexing**: Lanes indexed from right to left (0 = rightmost)
4. **Default Values**: Many attributes have sensible defaults if omitted
5. **XML Structure**: All files must have proper XML root elements
6. **ID Uniqueness**: All IDs must be unique within their type
7. **Network Consistency**: Node references in edges must exist
8. **Connection Validity**: Lane indices must be within range

## References

- [Official SUMO PlainXML Documentation](https://sumo.dlr.de/docs/Networks/PlainXML.html)
- [SUMO Network Building](https://sumo.dlr.de/docs/Networks/Building_Networks_from_own_XML-descriptions.html)
- [Vehicle Class Permissions](https://sumo.dlr.de/docs/Vehicle_Type_Parameter_Defaults.html)