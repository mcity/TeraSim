"""
TeraSim Real-time Visualization Application (Dash Version)

This Dash application provides real-time visualization of TeraSim simulations.
It reads simulation state from Redis and displays vehicles, traffic lights, and road network.
Uses Dash's callback system for efficient updates without page refresh.
"""

import dash
from dash import dcc, html, Input, Output, State
import plotly.graph_objects as go
import json
import argparse
import redis
import numpy as np
from datetime import datetime
import logging

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Parse command line arguments
parser = argparse.ArgumentParser()
parser.add_argument("--simulation_uuid", required=True)
parser.add_argument("--redis_host", default="localhost")
parser.add_argument("--redis_port", type=int, default=6379)
parser.add_argument("--port", type=int, default=8050)
parser.add_argument("--update_interval", type=float, default=0.5, help="Update interval in seconds")
args = parser.parse_args()

# Connect to Redis with connection pooling and timeout
pool = redis.ConnectionPool(
    host=args.redis_host, 
    port=args.redis_port, 
    decode_responses=True,
    socket_connect_timeout=5,
    socket_timeout=5,
    max_connections=10
)
redis_client = redis.Redis(connection_pool=pool)

# Initialize Dash app
app = dash.Dash(__name__, update_title=None)
app.title = "TeraSim Real-time Visualization"

# Define CSS styles - Minimalist theme
styles = {
    'container': {
        'padding': '20px',
        'fontFamily': '-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif',
        'backgroundColor': '#ffffff',
        'minHeight': '100vh'
    },
    'header': {
        'backgroundColor': '#f8f8f8',
        'padding': '15px',
        'marginBottom': '20px',
        'borderRadius': '0',
        'borderBottom': '1px solid #e0e0e0'
    },
    'title': {
        'fontSize': '28px',
        'fontWeight': '300',
        'marginBottom': '10px',
        'letterSpacing': '2px',
        'color': '#333333'
    },
    'info': {
        'fontSize': '13px',
        'color': '#666666',
        'fontFamily': 'monospace'
    },
    'controlPanel': {
        'backgroundColor': '#f8f8f8',
        'padding': '15px',
        'borderRadius': '0',
        'marginBottom': '20px',
        'border': '1px solid #e0e0e0'
    },
    'metric': {
        'backgroundColor': '#ffffff',
        'padding': '12px',
        'borderRadius': '0',
        'border': '1px solid #e0e0e0',
        'marginBottom': '10px'
    },
    'metricTitle': {
        'fontSize': '11px',
        'color': '#666666',
        'marginBottom': '5px',
        'textTransform': 'uppercase',
        'letterSpacing': '1px'
    },
    'metricValue': {
        'fontSize': '24px',
        'fontWeight': '300',
        'color': '#333333',
        'fontFamily': 'monospace'
    }
}

# Layout
app.layout = html.Div([
    # Header
    html.Div([
        html.H1("TeraSim Real-time Visualization", style=styles['title']),
        html.Div(f"Simulation ID: {args.simulation_uuid}", style=styles['info'])
    ], style=styles['header']),
    
    # Main content
    html.Div([
        # Left column - Map
        html.Div([
            dcc.Graph(
                id='live-map',
                style={'height': '700px'},
                config={'displayModeBar': True, 'displaylogo': False, 'toImageButtonOptions': {'format': 'png', 'filename': 'terasim_map'}}
            )
        ], style={'width': '75%', 'display': 'inline-block', 'verticalAlign': 'top'}),
        
        # Right column - Statistics and Controls
        html.Div([
            # Control Panel
            html.Div([
                html.H3("DISPLAY OPTIONS", style={'marginBottom': '15px', 'fontSize': '14px', 'fontWeight': '300', 'letterSpacing': '1px', 'color': '#888'}),
                dcc.Checklist(
                    id='display-options',
                    options=[
                        {'label': 'Track AV', 'value': 'track_av'},
                        {'label': 'Vehicle Labels', 'value': 'labels'},
                        {'label': 'Traffic Lights', 'value': 'traffic_lights'},
                        {'label': 'Construction Zones', 'value': 'construction'}
                    ],
                    value=['track_av'],  # Default to track AV
                    style={'marginBottom': '10px', 'fontSize': '13px'}
                )
            ], style=styles['controlPanel']),
            
            # Statistics
            html.Div([
                html.H3("STATISTICS", style={'marginBottom': '15px', 'fontSize': '14px', 'fontWeight': '300', 'letterSpacing': '1px', 'color': '#888'}),
                html.Div(id='stats-container')
            ], style={'marginTop': '20px'}),
            
            # Status
            html.Div([
                html.H3("STATUS", style={'marginBottom': '15px', 'fontSize': '14px', 'fontWeight': '300', 'letterSpacing': '1px', 'color': '#888'}),
                html.Div(id='status-container')
            ], style={'marginTop': '20px'})
            
        ], style={'width': '23%', 'display': 'inline-block', 'verticalAlign': 'top', 'marginLeft': '2%'})
    ]),
    
    # Interval component for updates
    dcc.Interval(
        id='interval-component',
        interval=int(args.update_interval * 1000),  # Convert to milliseconds
        n_intervals=0
    ),
    
    # Store components for data
    dcc.Store(id='map-data-store'),
    dcc.Store(id='previous-state-store')
    
], style=styles['container'])


def get_simulation_state(simulation_uuid):
    """Get current simulation state from Redis."""
    try:
        # Set timeout for Redis operations
        redis_client.ping()  # Check connection
        state_str = redis_client.get(f"simulation:{simulation_uuid}:state")
        if state_str:
            return json.loads(state_str)
    except redis.ConnectionError as e:
        logger.error(f"Redis connection error: {e}")
    except redis.TimeoutError as e:
        logger.error(f"Redis timeout error: {e}")
    except json.JSONDecodeError as e:
        logger.error(f"JSON decode error: {e}")
    except Exception as e:
        logger.error(f"Error getting simulation state: {e}")
    return None


def get_map_data(simulation_uuid):
    """Get static map data from Redis."""
    try:
        map_data_str = redis_client.get(f"simulation:{simulation_uuid}:map_data")
        if map_data_str:
            return json.loads(map_data_str)
    except Exception as e:
        logger.error(f"Error getting map data: {e}")
    return None


def create_map_traces(map_data, sim_state, display_options):
    """Create all map traces for the visualization."""
    traces = []
    
    # Get AV position if tracking
    av_x, av_y = None, None
    if 'track_av' in display_options and sim_state:
        vehicles = sim_state.get("agent_details", {}).get("vehicle", {})
        for vid, vdata in vehicles.items():
            if "AV" in vid:
                av_x, av_y = vdata["x"], vdata["y"]
                break
    
    # Draw all lane boundaries (edges and dividers)
    if map_data and "edges" in map_data:
        for edge in map_data["edges"]:
            if "boundaries" in edge:
                for boundary in edge["boundaries"]:
                    x_coords = [pt[0] for pt in boundary["points"]]
                    y_coords = [pt[1] for pt in boundary["points"]]
                    
                    # Different styles for edge boundaries vs lane dividers
                    if boundary["type"] == "edge":
                        # Road edge - solid black line
                        line_style = dict(color='black', width=2)
                        name = f'Edge {edge["id"]} {boundary.get("side", "")}'
                    else:
                        # Lane divider - dashed black line
                        line_style = dict(color='black', width=2, dash='dash')
                        name = f'Divider {edge["id"]}'
                    
                    traces.append(go.Scatter(
                        x=x_coords,
                        y=y_coords,
                        mode='lines',
                        line=line_style,
                        name=name,
                        showlegend=False,
                        hoverinfo='skip'
                    ))
    
    # Draw lanes (as thin center lines)
    if map_data and "lanes" in map_data:
        for lane in map_data["lanes"]:
            x_coords = [pt[0] for pt in lane["shape"]]
            y_coords = [pt[1] for pt in lane["shape"]]
            
            traces.append(go.Scatter(
                x=x_coords,
                y=y_coords,
                mode='lines',
                line=dict(color='lightgray', width=1, dash='dot'),  # Gray dotted center line
                name=f'Lane {lane["id"]}',
                showlegend=False,
                hovertemplate=f'Lane: {lane["id"]}<br>Speed limit: {lane["speed_limit"]:.1f} m/s<extra></extra>'
            ))
    
    # Draw junctions
    if map_data and "junctions" in map_data:
        for junction in map_data["junctions"]:
            if junction["shape"]:
                x_coords = [pt[0] for pt in junction["shape"]] + [junction["shape"][0][0]]
                y_coords = [pt[1] for pt in junction["shape"]] + [junction["shape"][0][1]]
                
                traces.append(go.Scatter(
                    x=x_coords,
                    y=y_coords,
                    mode='lines',
                    fill='toself',
                    fillcolor='rgba(135, 206, 235, 0.2)',
                    line=dict(color='steelblue', width=1),
                    showlegend=False,
                    hovertemplate=f'Junction: {junction["id"]}<extra></extra>'
                ))
    
    # Draw vehicles with rotation
    if sim_state and "agent_details" in sim_state:
        vehicles = sim_state["agent_details"].get("vehicle", {})
        show_labels = 'labels' in display_options
        
        if vehicles:
            # Instead of grouping, draw each vehicle individually with rotation
            for vid, vdata in vehicles.items():
                x, y = vdata["x"], vdata["y"]
                text = vid if show_labels else ""
                hover = f'{vid}<br>Speed: {vdata["speed"]:.1f} m/s<br>Type: {vdata["type"]}'
                
                # Determine vehicle color - original colors
                if "AV" in vid:
                    color = 'red'
                elif "BV" in vid:
                    color = 'blue'
                else:
                    color = 'green'
                
                # Get vehicle angle (SUMO angle: 0=North, clockwise)
                sumo_angle = vdata.get("sumo_angle", 0)
                # Convert to mathematical angle (0=East, counter-clockwise)
                # SUMO: 0=North, 90=East, 180=South, 270=West (clockwise)
                # Math: 0=East, 90=North, 180=West, 270=South (counter-clockwise)
                math_angle = (90 - sumo_angle) % 360
                
                # Create arrow-shaped vehicle using path
                # Define vehicle shape (pointing right at 0 degrees)
                length = vdata.get("length", 4.5)
                width = vdata.get("width", 1.8)
                
                # Simple arrow shape
                arrow_length = length * 0.8
                arrow_width = width * 0.8
                
                # Calculate rotated arrow points
                angle_rad = np.radians(math_angle)
                cos_a = np.cos(angle_rad)
                sin_a = np.sin(angle_rad)
                
                # Arrow points (relative to center)
                points = [
                    [arrow_length/2, 0],  # tip
                    [-arrow_length/2, arrow_width/2],  # left back
                    [-arrow_length/2, -arrow_width/2],  # right back
                    [arrow_length/2, 0]  # close path
                ]
                
                # Rotate and translate points
                rotated_points = []
                for px, py in points:
                    rx = px * cos_a - py * sin_a + x
                    ry = px * sin_a + py * cos_a + y
                    rotated_points.append([rx, ry])
                
                # Draw vehicle as filled shape
                x_coords = [p[0] for p in rotated_points]
                y_coords = [p[1] for p in rotated_points]
                
                traces.append(go.Scatter(
                    x=x_coords,
                    y=y_coords,
                    mode='lines',
                    fill='toself',
                    fillcolor=color,
                    line=dict(color='darkgray', width=1),
                    name=vid,
                    showlegend=False,
                    hovertemplate=hover
                ))
                
                # Add label if needed
                if show_labels:
                    traces.append(go.Scatter(
                        x=[x],
                        y=[y],
                        mode='text',
                        text=[vid],
                        textposition="top center",
                        showlegend=False,
                        hoverinfo='skip'
                    ))
        
        # Draw VRUs (pedestrians)
        vrus = sim_state["agent_details"].get("vru", {})
        if vrus:
            vru_x = [v["x"] for v in vrus.values()]
            vru_y = [v["y"] for v in vrus.values()]
            vru_text = [vid if show_labels else "" for vid in vrus.keys()]
            
            traces.append(go.Scatter(
                x=vru_x, y=vru_y,
                mode='markers+text',
                marker=dict(size=8, color='orange', symbol='circle'),
                text=vru_text,
                textposition="top center",
                name='VRU',
                hovertemplate='VRU: %{text}<extra></extra>'
            ))
        
        # Draw construction objects
        construction_objects = sim_state.get("construction_objects", {})
        if construction_objects:
            for cid, cdata in construction_objects.items():
                # Determine icon based on type
                if cdata["type"] == "CONSTRUCTION_CONE":
                    color = 'orange'
                    symbol = 'triangle-up'
                    size = 6
                elif cdata["type"] == "CONSTRUCTION_BARRIER":
                    color = 'yellow'
                    symbol = 'square'
                    size = 8
                elif cdata["type"] == "CONSTRUCTION_SIGN":
                    color = 'red'
                    symbol = 'diamond'
                    size = 10
                else:
                    color = 'gray'
                    symbol = 'circle'
                    size = 6
                
                traces.append(go.Scatter(
                    x=[cdata["x"]], y=[cdata["y"]],
                    mode='markers',
                    marker=dict(size=size, color=color, symbol=symbol, line=dict(width=1, color='black')),
                    showlegend=False,
                    hovertemplate=f'Construction: {cid}<br>Type: {cdata["type"]}<br>Position: ({cdata["x"]:.1f}, {cdata["y"]:.1f})<extra></extra>'
                ))
    
    # Draw traffic lights
    if 'traffic_lights' in display_options and map_data and "traffic_lights" in map_data:
        tl_states = {}
        if sim_state and "traffic_light_details" in sim_state:
            tl_states = sim_state["traffic_light_details"]
        
        for tl in map_data["traffic_lights"]:
            tl_id = tl["id"]
            x, y = tl["position"]
            
            # Get current state
            color = 'gray'
            state = 'unknown'
            if tl_id in tl_states:
                state_str = tl_states[tl_id].get("tls", "")
                if 'r' in state_str.lower():
                    color = 'red'
                    state = 'red'
                elif 'y' in state_str.lower():
                    color = 'yellow'
                    state = 'yellow'
                elif 'g' in state_str.lower():
                    color = 'green'
                    state = 'green'
            
            traces.append(go.Scatter(
                x=[x], y=[y],
                mode='markers',
                marker=dict(size=12, color=color, symbol='circle', line=dict(width=2, color='black')),
                showlegend=False,
                hovertemplate=f'Traffic Light: {tl_id}<br>State: {state}<extra></extra>'
            ))
    
    # Draw construction zones
    if 'construction' in display_options and sim_state and "construction_zone_details" in sim_state:
        construction_zones = sim_state["construction_zone_details"]
        if construction_zones:
            for lane_id, shape in construction_zones.items():
                x_coords = [pt[0] for pt in shape]
                y_coords = [pt[1] for pt in shape]
                
                traces.append(go.Scatter(
                    x=x_coords, y=y_coords,
                    mode='lines',
                    line=dict(color='orange', width=3, dash='dash'),
                    showlegend=False,
                    hovertemplate=f'Construction Zone<br>Lane: {lane_id}<extra></extra>'
                ))
    
    return traces


# Callback to load initial map data
@app.callback(
    Output('map-data-store', 'data'),
    Input('interval-component', 'n_intervals'),
    prevent_initial_call=False
)
def load_map_data(n):
    """Load map data once and store it."""
    if n == 0:  # Only load on first interval
        return get_map_data(args.simulation_uuid)
    return dash.no_update


# Main update callback
@app.callback(
    [Output('live-map', 'figure'),
     Output('stats-container', 'children'),
     Output('status-container', 'children')],
    [Input('interval-component', 'n_intervals')],
    [State('map-data-store', 'data'),
     State('display-options', 'value')]
)
def update_visualization(n, map_data, display_options):
    """Update the visualization with current simulation state."""
    
    # Debug logging
    logger.info(f"Update callback triggered: n_intervals={n}")
    
    try:
        # Get current simulation state with timeout protection
        sim_state = get_simulation_state(args.simulation_uuid)
        
        if sim_state:
            logger.info(f"Got simulation state, time={sim_state.get('simulation_time', -1)}")
        else:
            logger.warning("No simulation state found!")
    except Exception as e:
        logger.error(f"Failed to get simulation state: {e}")
        sim_state = None
    
    # Create figure
    fig = go.Figure()
    
    if sim_state and map_data:
        # Add all traces
        traces = create_map_traces(map_data, sim_state, display_options)
        for trace in traces:
            fig.add_trace(trace)
        
        # Update layout
        fig.update_layout(
            title=dict(
                text="SIMULATION MAP",
                font=dict(size=16, color='#333', family='sans-serif')
            ),
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            hovermode='closest',
            height=700,
            showlegend=False,
            plot_bgcolor='#f8f8f8',
            paper_bgcolor='white',
            font=dict(color='#333'),
            xaxis=dict(gridcolor='#e0e0e0', zerolinecolor='#ccc'),
            yaxis=dict(gridcolor='#e0e0e0', zerolinecolor='#ccc'),
            uirevision='display-options' if 'track_av' not in display_options else None  # Keep zoom only when not tracking
        )
        
        # Set axis properties
        if 'track_av' in display_options and sim_state:
            # Find AV position
            vehicles = sim_state.get("agent_details", {}).get("vehicle", {})
            av_found = False
            for vid, vdata in vehicles.items():
                if "AV" in vid:
                    av_x, av_y = vdata["x"], vdata["y"]
                    # Set view window centered on AV (50m range)
                    view_range = 50
                    fig.update_xaxes(range=[av_x - view_range, av_x + view_range])
                    fig.update_yaxes(range=[av_y - view_range, av_y + view_range])
                    av_found = True
                    break
            
            if not av_found and "bounds" in map_data:
                # Fallback to full map view if no AV found
                bounds = map_data["bounds"]
                margin = 50
                fig.update_xaxes(range=[bounds["min_x"]-margin, bounds["max_x"]+margin])
                fig.update_yaxes(range=[bounds["min_y"]-margin, bounds["max_y"]+margin])
        else:
            # Normal full map view
            if "bounds" in map_data:
                bounds = map_data["bounds"]
                margin = 50
                fig.update_xaxes(range=[bounds["min_x"]-margin, bounds["max_x"]+margin])
                fig.update_yaxes(range=[bounds["min_y"]-margin, bounds["max_y"]+margin])
        
        # Equal aspect ratio
        fig.update_yaxes(scaleanchor="x", scaleratio=1)
        
        # Create statistics
        vehicles = sim_state.get("agent_details", {}).get("vehicle", {})
        vrus = sim_state.get("agent_details", {}).get("vru", {})
        construction_objects = sim_state.get("construction_objects", {})
        
        stats_children = [
            html.Div([
                html.Div("Simulation Time", style=styles['metricTitle']),
                html.Div(f"{sim_state.get('simulation_time', 0):.1f} s", style=styles['metricValue'])
            ], style=styles['metric']),
            html.Div([
                html.Div("Vehicles", style=styles['metricTitle']),
                html.Div(str(len(vehicles)), style=styles['metricValue'])
            ], style=styles['metric']),
            html.Div([
                html.Div("VRUs", style=styles['metricTitle']),
                html.Div(str(len(vrus)), style=styles['metricValue'])
            ], style=styles['metric']),
            html.Div([
                html.Div("Construction", style=styles['metricTitle']),
                html.Div(str(len(construction_objects)), style=styles['metricValue'])
            ], style=styles['metric'])
        ]
        
        if vehicles:
            avg_speed = np.mean([v["speed"] for v in vehicles.values()])
            stats_children.append(
                html.Div([
                    html.Div("Avg Speed", style=styles['metricTitle']),
                    html.Div(f"{avg_speed:.1f} m/s", style=styles['metricValue'])
                ], style=styles['metric'])
            )
        
        # Create status
        status = redis_client.get(f"simulation:{args.simulation_uuid}:status") or "Unknown"
        status_children = [
            html.Div([
                html.Div(f"STATUS: {status.upper()}", style={'color': 'green', 'fontSize': '14px', 'fontFamily': 'monospace'}),
                html.Div(f"UPDATED: {datetime.now().strftime('%H:%M:%S')}", 
                        style={'color': '#666', 'fontSize': '11px', 'marginTop': '5px', 'fontFamily': 'monospace'})
            ])
        ]
        
    else:
        # No data available
        fig.add_annotation(
            text="WAITING FOR SIMULATION DATA",
            xref="paper", yref="paper",
            x=0.5, y=0.5,
            showarrow=False,
            font=dict(size=14, color="#666", family='monospace')
        )
        fig.update_layout(
            xaxis=dict(visible=False),
            yaxis=dict(visible=False),
            plot_bgcolor='white',
            paper_bgcolor='white'
        )
        
        stats_children = [html.Div("NO DATA AVAILABLE", style={'color': '#666', 'fontSize': '12px', 'fontFamily': 'monospace'})]
        status_children = [html.Div("WAITING", style={'color': 'orange', 'fontSize': '14px', 'fontFamily': 'monospace'})]
    
    return fig, stats_children, status_children


if __name__ == '__main__':
    logger.info(f"Starting Dash visualization app on port {args.port}")
    logger.info(f"Simulation UUID: {args.simulation_uuid}")
    logger.info(f"Redis connection: {args.redis_host}:{args.redis_port}")
    
    app.run(
        host='0.0.0.0',
        port=args.port,
        debug=False,
        dev_tools_silence_routes_logging=True,
        dev_tools_ui=False
    )