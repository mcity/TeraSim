# OpenDRIVE Format Documentation

## Overview

OpenDRIVE is an open file format for the logical description of road networks. It was developed to simplify the exchange of road network data between different driving simulators. The format uses XML and is designed to describe the geometry of roads, lanes, and features along the road.

- **File Extension**: `.xodr` (uncompressed), `.xodrz` (gzip compressed)
- **Format**: XML-based
- **Current Version**: 1.8.1 (as of November 2024)
- **Maintained by**: ASAM (Association for Standardization of Automation and Measuring Systems)

## Version History

| Version | Year | Key Features |
|---------|------|-------------|
| 1.4 | 2015 | Base standard, widely supported |
| 1.5 | 2018 | Enhanced junction descriptions |
| 1.6 | 2019 | First ASAM version, improved signals |
| 1.7 | 2021 | Enhanced surface descriptions |
| 1.8 | 2024 | Latest improvements |

**Note**: Backward compatibility is maintained from version 1.4 onwards.

## XML Structure Hierarchy

```xml
<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="" version="" date="" north="" south="" east="" west="">
        <geoReference/>
        <userData/>
    </header>
    
    <road id="" name="" length="" junction="">
        <link/>
        <type/>
        <planView/>
        <elevationProfile/>
        <lateralProfile/>
        <lanes/>
        <objects/>
        <signals/>
        <surface/>
        <railroad/>
    </road>
    
    <controller/>
    <junction/>
    <junctionGroup/>
    <station/>
</OpenDRIVE>
```

## Core Elements

### 1. Header Element

Defines metadata and coordinate system information.

```xml
<header revMajor="1" revMinor="4" name="Example" version="1.0" 
        date="2024-01-01T00:00:00" north="100" south="-100" 
        east="100" west="-100" vendor="Company">
    <geoReference><!CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    <userData>
        <userData code="myCode" value="myValue"/>
    </userData>
</header>
```

**Attributes**:
- `revMajor`, `revMinor`: OpenDRIVE format version
- `name`: Database name
- `version`: Database version
- `date`: Creation/modification date
- `north`, `south`, `east`, `west`: Database boundaries
- `vendor`: Vendor/tool that created the file

### 2. Road Element

Describes individual road segments.

```xml
<road name="Road 1" length="100.0" id="1" junction="-1">
    <link>
        <predecessor elementType="road" elementId="0" contactPoint="end"/>
        <successor elementType="junction" elementId="2" contactPoint="start"/>
    </link>
    <type s="0.0" type="town">
        <speed max="13.89" unit="m/s"/>
    </type>
    <planView>...</planView>
    <lanes>...</lanes>
</road>
```

**Attributes**:
- `id`: Unique identifier
- `name`: Optional road name
- `length`: Total road length in meters
- `junction`: Junction ID (-1 if not part of junction)

**Sub-elements**:
- `link`: Connectivity to other roads/junctions
- `type`: Road type (motorway, rural, town, etc.)
- `planView`: Horizontal geometry
- `elevationProfile`: Vertical geometry
- `lateralProfile`: Lateral geometry (superelevation, crossfall)
- `lanes`: Lane definitions

### 3. Junction Element

Defines intersections and their internal connections.

```xml
<junction id="1" name="Junction1" type="default">
    <connection id="0" incomingRoad="1" connectingRoad="10" contactPoint="start">
        <laneLink from="-1" to="-1"/>
        <laneLink from="-2" to="-2"/>
    </connection>
    <priority high="1" low="2"/>
    <controller id="1" type="trafficLight"/>
</junction>
```

**Attributes**:
- `id`: Unique junction identifier
- `name`: Optional junction name
- `type`: Junction type (default, roundabout, etc.)

**Connection Attributes**:
- `incomingRoad`: ID of incoming road
- `connectingRoad`: ID of connecting road within junction
- `contactPoint`: Connection point (start/end)

### 4. Controller Element

Defines traffic signal controllers.

```xml
<controller id="1" name="TrafficLight1" sequence="1">
    <control signalId="1" type="trafficLight"/>
    <control signalId="2" type="trafficLight"/>
</controller>
```

**Attributes**:
- `id`: Unique controller identifier
- `name`: Controller name
- `sequence`: Execution sequence

### 5. Signal Element

Defines traffic signals, signs, and road furniture.

```xml
<signal s="50.0" t="-3.5" id="1" name="Signal1" dynamic="yes" 
        orientation="+" zOffset="5.0" country="DEU" type="294" 
        subtype="10" value="50" unit="km/h">
    <validity fromLane="-2" toLane="-1"/>
    <dependency id="2" type="limitLine"/>
</signal>
```

**Attributes**:
- `s`, `t`: Position along/lateral to reference line
- `id`: Unique signal identifier
- `dynamic`: Whether signal can change state
- `type`, `subtype`: Signal type codes (country-specific)
- `value`: Signal value (e.g., speed limit)

## Lane System

### Lane Section Structure

```xml
<lanes>
    <laneOffset s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
    <laneSection s="0.0">
        <left>
            <lane id="2" type="driving" level="false">
                <link>
                    <predecessor id="2"/>
                    <successor id="2"/>
                </link>
                <width sOffset="0.0" a="3.5" b="0.0" c="0.0" d="0.0"/>
                <roadMark sOffset="0.0" type="solid" weight="standard" 
                         color="white" width="0.12" laneChange="none"/>
                <speed sOffset="0.0" max="50" unit="km/h"/>
                <access sOffset="0.0" restriction="simulator"/>
            </lane>
        </left>
        <center>
            <lane id="0" type="none" level="false">
                <roadMark sOffset="0.0" type="solid solid" color="yellow"/>
            </lane>
        </center>
        <right>
            <lane id="-1" type="driving" level="false">
                <!-- Similar structure as left lanes -->
            </lane>
        </right>
    </laneSection>
</lanes>
```

### Lane Types

| Type | Description | Usage |
|------|-------------|-------|
| `driving` | Normal driving lane | Regular traffic |
| `stop` | Hard shoulder/emergency lane | Emergency stops |
| `shoulder` | Soft shoulder | Not for regular traffic |
| `biking` | Bicycle lane | Bicycles only |
| `sidewalk` | Pedestrian walkway | Pedestrians |
| `border` | Border marking | Visual separation |
| `restricted` | Restricted usage | Special vehicles |
| `parking` | Parking lane | Parked vehicles |
| `bidirectional` | Lane for both directions | Shared lane |
| `median` | Central reservation | Separation |
| `special1-3` | User-defined | Custom usage |
| `roadWorks` | Construction area | Temporary |
| `curb` | Curb stone | Physical border |
| `entry` | Acceleration lane | Highway entry |
| `exit` | Deceleration lane | Highway exit |
| `onRamp` | On-ramp | Merging traffic |
| `offRamp` | Off-ramp | Exiting traffic |
| `connectingRamp` | Connection between ramps | Ramp connections |

### Lane Numbering Convention

- **Center lane**: ID = 0 (reference line)
- **Left lanes**: Positive IDs (1, 2, 3, ...) increasing to the left
- **Right lanes**: Negative IDs (-1, -2, -3, ...) decreasing to the right

## Geometry Definitions

### Plan View (Horizontal Geometry)

```xml
<planView>
    <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="100.0">
        <line/>  <!-- Straight line -->
    </geometry>
    <geometry s="100.0" x="100.0" y="0.0" hdg="0.0" length="50.0">
        <arc curvature="0.01"/>  <!-- Circular arc -->
    </geometry>
    <geometry s="150.0" x="150.0" y="0.5" hdg="0.5" length="30.0">
        <spiral curvStart="0.0" curvEnd="0.02"/>  <!-- Clothoid/Euler spiral -->
    </geometry>
    <geometry s="180.0" x="180.0" y="1.0" hdg="0.6" length="40.0">
        <poly3 a="0.0" b="0.0" c="0.001" d="0.0001"/>  <!-- Cubic polynomial -->
    </geometry>
    <geometry s="220.0" x="220.0" y="2.0" hdg="0.7" length="35.0">
        <paramPoly3 aU="0.0" bU="1.0" cU="0.0" dU="0.0" 
                   aV="0.0" bV="0.0" cV="0.5" dV="0.0" pRange="arcLength"/>
    </geometry>
</planView>
```

### Geometry Types

1. **Line**: Straight segment
   - No additional parameters needed

2. **Arc**: Circular arc
   - `curvature`: Constant curvature (1/radius)

3. **Spiral**: Clothoid/Euler spiral
   - `curvStart`: Curvature at start
   - `curvEnd`: Curvature at end

4. **Poly3**: Cubic polynomial
   - Coefficients: a, b, c, d
   - u(s) = a + b*s + c*s² + d*s³

5. **ParamPoly3**: Parametric cubic polynomial
   - Separate polynomials for u and v coordinates
   - `pRange`: "arcLength" or "normalized"

## Elevation and Lateral Profiles

### Elevation Profile (Vertical Geometry)

```xml
<elevationProfile>
    <elevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
    <elevation s="100.0" a="5.0" b="0.02" c="0.0" d="0.0"/>
</elevationProfile>
```

### Lateral Profile (Superelevation and Crossfall)

```xml
<lateralProfile>
    <superelevation s="0.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
    <crossfall s="0.0" a="-0.02" b="0.0" c="0.0" d="0.0" side="both"/>
    <shape s="0.0" t="-5.0" a="0.0" b="0.0" c="0.0" d="0.0"/>
</lateralProfile>
```

## Objects Along the Road

```xml
<objects>
    <object id="1" type="pole" name="LightPole" s="25.0" t="-3.0" 
            zOffset="0.0" validLength="0.0" orientation="none" 
            length="0.3" width="0.3" height="6.0" hdg="0.0" 
            pitch="0.0" roll="0.0">
        <repeat s="0.0" length="200.0" distance="50.0" 
                tStart="-3.0" tEnd="-3.0" widthStart="0.3" 
                widthEnd="0.3" heightStart="6.0" heightEnd="6.0" 
                zOffsetStart="0.0" zOffsetEnd="0.0"/>
        <validity fromLane="-4" toLane="-1"/>
    </object>
</objects>
```

## Surface Description

```xml
<surface>
    <CRG file="surface.crg" sStart="0.0" sEnd="100.0" 
         orientation="same" mode="attached" purpose="friction"
         sOffset="0.0" tOffset="0.0" zOffset="0.0" zScale="1.0"
         hOffset="0.0"/>
</surface>
```

## Traffic Control and Priority

### Junction Priority Rules

```xml
<junction id="1" name="Intersection">
    <connection id="0" incomingRoad="1" connectingRoad="10" contactPoint="start">
        <laneLink from="-1" to="-1"/>
    </connection>
    <priority high="1" low="2"/>  <!-- Road 1 has priority over Road 2 -->
</junction>
```

### Signal States and Types

Common signal types (country-specific):
- **Germany (DEU)**:
  - 206: Stop sign
  - 205: Give way
  - 274: Speed limit
  - 294: Traffic light
  - 301: Priority road

### Controller Logic

Controllers can manage multiple signals:
```xml
<controller id="1" name="Intersection_Controller">
    <control signalId="1" type="trafficLight"/>
    <control signalId="2" type="trafficLight"/>
    <control signalId="3" type="trafficLight"/>
    <control signalId="4" type="trafficLight"/>
</controller>
```

## Important Conversion Considerations

### For SUMO Conversion

1. **Junction Types**: 
   - OpenDRIVE junctions without controllers → SUMO "priority" type
   - Junctions with controllers → SUMO "traffic_light" type
   - Roundabouts → Special handling required

2. **Lane Mapping**:
   - OpenDRIVE center lane (0) is virtual
   - Left lanes (positive IDs) → Opposite direction in SUMO
   - Right lanes (negative IDs) → Forward direction in SUMO

3. **Signal Handling**:
   - Dynamic signals → Traffic lights in SUMO
   - Static signals → Road attributes or POIs

4. **Geometry Conversion**:
   - Spirals and paramPoly3 need approximation
   - SUMO uses polylines for complex curves

## User Data Extensions

OpenDRIVE allows custom data through userData elements:

```xml
<userData>
    <vectorScene program="RoadRunner" version="2019.0.0"/>
    <property name="customProperty" value="customValue"/>
</userData>
```

## Best Practices

1. **Unique IDs**: Ensure all elements have unique identifiers
2. **Connectivity**: Verify road links and lane connections
3. **Geometry Continuity**: Ensure smooth transitions between geometry elements
4. **Lane Consistency**: Maintain consistent lane numbering across sections
5. **Signal Placement**: Position signals accurately relative to stop lines
6. **Junction Design**: Define all possible connections explicitly

## Common Issues and Solutions

### Issue 1: Missing Junction Control Type
**Problem**: OpenDRIVE file lacks controller/signal definitions for junctions
**Solution**: Infer from junction complexity and road types

### Issue 2: Lane Connection Gaps
**Problem**: Discontinuous lane IDs between road sections
**Solution**: Use lane link elements to maintain connectivity

### Issue 3: Geometry Discontinuities
**Problem**: Gaps between geometry segments
**Solution**: Ensure end position of one segment matches start of next

## References

- [ASAM OpenDRIVE Official Site](https://www.asam.net/standards/detail/opendrive/)
- [OpenDRIVE 1.6 Specification](https://releases.asam.net/OpenDRIVE/1.6.0/)
- Format version 1.4 (2015) - Base standard maintained for backward compatibility
- Latest version 1.8.1 (2024) - Current ASAM standard

## Appendix: Sample OpenDRIVE File

```xml
<?xml version="1.0" encoding="UTF-8"?>
<OpenDRIVE>
    <header revMajor="1" revMinor="4" name="Sample" version="1.0" 
            date="2024-01-01T00:00:00" north="500" south="-500" 
            east="500" west="-500">
        <geoReference><![CDATA[+proj=utm +zone=32 +datum=WGS84]]></geoReference>
    </header>
    
    <!-- Simple straight road -->
    <road name="Main Street" length="200.0" id="1" junction="-1">
        <link>
            <predecessor elementType="junction" elementId="1"/>
            <successor elementType="junction" elementId="2"/>
        </link>
        <type s="0.0" type="town">
            <speed max="13.89" unit="m/s"/>
        </type>
        <planView>
            <geometry s="0.0" x="0.0" y="0.0" hdg="0.0" length="200.0">
                <line/>
            </geometry>
        </planView>
        <lanes>
            <laneSection s="0.0">
                <left>
                    <lane id="1" type="driving" level="false">
                        <width sOffset="0.0" a="3.5" b="0" c="0" d="0"/>
                    </lane>
                </left>
                <center>
                    <lane id="0" type="none" level="false"/>
                </center>
                <right>
                    <lane id="-1" type="driving" level="false">
                        <width sOffset="0.0" a="3.5" b="0" c="0" d="0"/>
                    </lane>
                </right>
            </laneSection>
        </lanes>
    </road>
    
    <!-- Simple T-junction -->
    <junction id="1" name="T-Junction">
        <connection id="0" incomingRoad="1" connectingRoad="10" contactPoint="start">
            <laneLink from="-1" to="-1"/>
        </connection>
        <connection id="1" incomingRoad="2" connectingRoad="11" contactPoint="start">
            <laneLink from="-1" to="-1"/>
        </connection>
    </junction>
</OpenDRIVE>
```