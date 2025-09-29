#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from pathlib import Path
import sys
import yaml

from terasim_vis import Net, Trajectories


def load_config(config_path):
    """Load configuration from YAML file."""
    config_path = Path(config_path)
    
    if not config_path.exists():
        print(f"Error: Configuration file {config_path} not found.")
        sys.exit(1)
    
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Convert paths to Path objects
        if config.get('fcd'):
            config['fcd'] = Path(config['fcd'])
        if config.get('net'):
            config['net'] = Path(config['net'])
        if config.get('save'):
            config['save'] = Path(config['save'])
        
        # Handle output directory configuration
        if config.get('output'):
            output_config = config['output']
            if output_config.get('dir'):
                output_config['dir'] = Path(output_config['dir'])
        
        # Set defaults for missing values
        config.setdefault('start_time', 0)
        config.setdefault('end_time', None)
        config.setdefault('speed_colors', False)
        config.setdefault('lane_colors', False)
        config.setdefault('animate', False)
        config.setdefault('dpi', 150)
        config.setdefault('figsize', '12,8')
        
        return config
        
    except Exception as e:
        print(f"Error loading configuration file: {e}")
        sys.exit(1)


class TrafficVisualizer:
    """Class for visualizing SUMO traffic simulation data."""
    
    def __init__(self, net, trajectories, config):
        """Initialize the visualizer with network, trajectories, and configuration."""
        self.net = net
        self.trajectories = trajectories
        self.config = config
        self.fig = None
        self.ax = None
        self.target_trajectory = None
        
    def setup_plot(self):
        """Set up the matplotlib figure and plot the network."""
        figsize = tuple(map(float, self.config['figsize'].split(",")))
        self.fig, self.ax = plt.subplots(figsize=figsize)
        
        # Get map styling configuration
        map_style = self.config.get('map_style', {})
        colors = map_style.get('colors', {})
        
        # Then change the colors of road elements
        lane_color = colors.get('lane_color', 'white')
        lane_marking_color = colors.get('lane_marking_color', 'black')
        # Plot the network with custom road colors
        # First plot with default settings to get the network structure
        self.net.plot(
            self.ax,
            style=map_style.get('style', 'EUR'),
            zoom_to_extents=map_style.get('zoom_to_extents', True),
            plot_stop_lines=map_style.get('plot_stop_lines', False),
            lane_kwargs=dict(color=lane_color),
            junction_kwargs=dict(color=lane_color),
            tl_phases=map_style.get('tl_phases', None)
            # lane_marking_kwargs=dict(color=lane_marking_color)
        )

        
        self.ax.set_aspect('equal')
        
        # Apply custom view settings if not zooming to extents
        if not map_style.get('zoom_to_extents', True):
            self._set_custom_view(map_style.get('view', {}))
        
        # Optional title and labels based on config
        if map_style.get('show_title', False):
            title = map_style.get('title', 'SUMO Traffic Simulation Visualization')
            self.ax.set_title(title)
        
        if map_style.get('show_labels', False):
            self.ax.set_xlabel("X coordinate (m)")
            self.ax.set_ylabel("Y coordinate (m)")
        
        # Hide ticks if not showing labels
        if not map_style.get('show_labels', False):
            self.ax.set_xticks([])
            self.ax.set_yticks([])
        
        # Remove margins and padding to fill the entire display area
        self.ax.margins(0)  # Remove default margins around the plot
        self.ax.set_aspect('equal', adjustable='box')  # Maintain aspect ratio
        
        # Adjust the subplot to remove whitespace
        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        
        # Alternative: use tight_layout for automatic adjustment
        # plt.tight_layout(pad=0)
        
        return self.fig, self.ax
    
    def _set_custom_view(self, view_config):
        """Set custom view boundaries based on configuration."""
        width = view_config.get('width', 200)
        height = view_config.get('height', 200)
        
        if view_config.get('center_on_ego', True) and self.target_trajectory:
            # Center on ego vehicle's first position
            if hasattr(self.target_trajectory, 'x') and len(self.target_trajectory.x) > 0:
                center_x = self.target_trajectory.x[0]
                center_y = self.target_trajectory.y[0]
            else:
                # Fallback to fixed center if ego trajectory not available
                center_x = view_config.get('center_x', 0)
                center_y = view_config.get('center_y', 0)
        else:
            # Use fixed center coordinates
            center_x = view_config.get('center_x', 0)
            center_y = view_config.get('center_y', 0)
        
        # Set view limits
        half_width = width / 2
        half_height = height / 2
        
        self.ax.set_xlim(center_x - half_width, center_x + half_width)
        self.ax.set_ylim(center_y - half_height, center_y + half_height)
    
    def configure_trajectory_colors(self, ego_vehicle_id=None, adv_vehicle_id=None):
        """Configure colors and markers for all trajectories."""
        # Find the target vehicle for following
        if ego_vehicle_id:
            for trajectory in self.trajectories:
                if trajectory.id == ego_vehicle_id:
                    self.target_trajectory = trajectory
                    print(f"Found target vehicle (ego): {ego_vehicle_id}")
                    break
            if not self.target_trajectory:
                print(f"Warning: Ego vehicle {ego_vehicle_id} not found in trajectories")
        
        # Set up color scheme and marker properties for different vehicle types
        for trajectory in self.trajectories:
            # Highlight the ego vehicle (green)
            if ego_vehicle_id and trajectory.id == ego_vehicle_id:
                trajectory.assign_colors_constant("blue")  # Green for ego vehicle
                trajectory.point_plot_kwargs["color"] = "blue"
                trajectory.point_plot_kwargs["markeredgecolor"] = "black"
                trajectory.point_plot_kwargs["zorder"] = 300  # Higher z-order to show on top
            # Highlight the adversarial vehicle (red)
            elif adv_vehicle_id and trajectory.id == adv_vehicle_id:
                trajectory.assign_colors_constant("red")  # Red for adversarial vehicle
                trajectory.point_plot_kwargs["color"] = "red"
                trajectory.point_plot_kwargs["markeredgecolor"] = "black"
                trajectory.point_plot_kwargs["zorder"] = 290  # High z-order but below ego
            else:
                # All other vehicles use gray color
                trajectory.assign_colors_constant("gray")
                trajectory.point_plot_kwargs["color"] = "gray"
                trajectory.point_plot_kwargs["markeredgecolor"] = "black"
                trajectory.point_plot_kwargs["zorder"] = 200
            # Set size, color for different vehicle types
            vehicle_type = getattr(trajectory, 'type', '')
            if 'ped' in vehicle_type.lower():
                trajectory.length = 1
                trajectory.width = 1
                trajectory.point_plot_kwargs["ms"] = 1
            elif 'bike' in vehicle_type.lower() or 'bicycle' in vehicle_type.lower():
                trajectory.length = 2  # Bicycles
                trajectory.width = 1
                trajectory.point_plot_kwargs["ms"] = 1
            else:
                trajectory.length = 4.8  # Regular vehicles
                trajectory.width = 2
                trajectory.point_plot_kwargs["ms"] = 2
    
    def plot_static_trajectories(self, speed_colors=False, lane_colors=False, ego_vehicle_id=None, adv_vehicle_id=None):
        """Plot all trajectories as static lines."""
        if not speed_colors and not lane_colors and (ego_vehicle_id or adv_vehicle_id):
            # Use special vehicle coloring scheme
            self.configure_trajectory_colors(ego_vehicle_id, adv_vehicle_id)
        else:
            # Use original coloring scheme for static plots
            for trajectory in self.trajectories:
                if speed_colors:
                    trajectory.assign_colors_speed(cmap="viridis")
                elif lane_colors:
                    trajectory.assign_colors_lane(cmap="tab10")
                else:
                    # Assign colors based on vehicle type
                    vehicle_type = getattr(trajectory, 'type', '')
                    if 'bike' in vehicle_type.lower() or 'bicycle' in vehicle_type.lower():
                        trajectory.assign_colors_constant("yellow")  # Bicycles
                    elif trajectory.id.startswith("VRU_"):
                        trajectory.assign_colors_constant("blue")   # Pedestrians
                    else:
                        trajectory.assign_colors_constant("red")    # Regular vehicles
        
        for trajectory in self.trajectories:
            trajectory.plot(self.ax, linewidth=1.5, alpha=0.7)
        
        # Add colorbar if using speed colors
        if speed_colors and self.trajectories.mappables:
            # Use the first trajectory's mappable for the colorbar
            first_mappable = next(iter(self.trajectories.mappables.values()))
            cbar = plt.colorbar(first_mappable, ax=self.ax, shrink=0.6)
            cbar.set_label("Speed (m/s)")
    
    def create_animation(self, ego_vehicle_id=None, adv_vehicle_id=None):
        """Create animated visualization with optional camera following."""
        # Configure trajectory colors and find target vehicle
        self.configure_trajectory_colors(ego_vehicle_id, adv_vehicle_id)
        
        # Set trajectories timestep
        self.trajectories.timestep = 0.1
        print(f"Trajectories timestep: {self.trajectories.timestep}")
        print(f"Trajectories start: {self.trajectories.start}, end: {self.trajectories.end}")
        
        # Get timestep range
        timestep_range = self.trajectories.timestep_range()
        
        # Filter frames to desired time range
        start_time = self.config.get('start_time')
        end_time = self.config.get('end_time')
        
        if start_time is not None or end_time is not None:
            if start_time is None:
                start_time = self.trajectories.start
            if end_time is None:
                end_time = self.trajectories.end
            timestep_range = timestep_range[(timestep_range >= start_time) & (timestep_range <= end_time)]
        
        # Limit to max frames if desired
        max_frames = self.config.get('max_frames')
        if max_frames is not None:
            timestep_range = timestep_range[:max_frames]
        
        print(f"Creating animation with {len(timestep_range)} frames")
        print(f"Time range: {timestep_range[0]:.1f} to {timestep_range[-1]:.1f}")
        
        # Create animation function with camera following
        def animate_with_follow(frame):
            # Call original plot_points function
            self.trajectories.plot_points(frame, self.ax, True)
            
            # Follow target vehicle if specified
            if self.target_trajectory and frame in self.target_trajectory.time:
                # Get vehicle position at current frame
                frame_idx = self.target_trajectory.time.index(frame)
                vehicle_x = self.target_trajectory.x[frame_idx]
                vehicle_y = self.target_trajectory.y[frame_idx]
                
                # Set view to follow the vehicle with configured view size
                map_style = self.config.get('map_style', {})
                view_config = map_style.get('view', {})
                width = view_config.get('width', 200)
                height = view_config.get('height', 200)
                
                half_width = width / 2
                half_height = height / 2
                
                self.ax.set_xlim(vehicle_x - half_width, vehicle_x + half_width)
                self.ax.set_ylim(vehicle_y - half_height, vehicle_y + half_height)
            
            return self.ax.get_children()  # Return artists for blitting
        
        if self.target_trajectory:
            # Use custom animation function for following
            anim = animation.FuncAnimation(
                self.fig, animate_with_follow, 
                frames=timestep_range, 
                repeat=False,
                interval=int(1000 * self.trajectories.timestep) if self.trajectories.timestep else 100,
                blit=False  # Disable blitting when changing view limits
            )
        else:
            # Use original approach when not following
            anim = animation.FuncAnimation(
                self.fig, self.trajectories.plot_points, 
                frames=timestep_range, 
                repeat=False,
                interval=int(1000 * self.trajectories.timestep) if self.trajectories.timestep else 100,
                fargs=(self.ax, True),
                blit=True
            )
        
        # Store time_range for later use in saving
        anim.time_range = timestep_range
        return anim


def main(path_to_config):
    """Main function."""
    config = load_config(path_to_config)
    
    print(f"Loading network from: {config['net']}")
    net = Net(str(config['net']))

    print(f"Loading trajectories from: {config['fcd']}")
    trajectories = Trajectories(str(config['fcd']), 
                                  start_time=config['start_time'],
                                  end_time=config['end_time'],
                                  vehicle_only=config['vehicle_only'] if 'vehicle_only' in config else False)
    print(f"Loaded {len(trajectories.trajectories)} trajectories")
    
    # Create visualizer instance
    visualizer = TrafficVisualizer(net, trajectories, config)
    visualizer.setup_plot()
    
    output_dir = Path(config['output_dir'])
    output_dir.mkdir(parents=True, exist_ok=True)

    if config['animate']:
        print("Creating animation...")
        ego_vehicle_id = config.get('ego_vehicle_id')
        adv_vehicle_id = config.get('adv_vehicle_id')
        if ego_vehicle_id:
            print(f"Camera will follow ego vehicle: {ego_vehicle_id}")
        if adv_vehicle_id:
            print(f"Highlighting adversarial vehicle: {adv_vehicle_id}")
        anim = visualizer.create_animation(ego_vehicle_id, adv_vehicle_id)
        
        video_name = config.get('video_name', 'animation')
        max_frames = config.get('max_frames')
        if max_frames is not None:
            video_name = f"{video_name}_{max_frames}"
        save_path = output_dir/f"{video_name}.mp4"
        
        print(f"Saving animation to: {save_path}")
        Writer = animation.writers['ffmpeg']
        writer = Writer(fps=8, metadata=dict(artist='TeraSim-Vis'), bitrate=3600)
        print(f"Saving {len(anim.time_range)} frames at 8 fps...")
        anim.save(str(save_path), writer=writer, dpi=config['dpi'])
        print(f"Animation saved successfully!")
    
    else:
        print("Creating static visualization...")
        ego_vehicle_id = config.get('ego_vehicle_id')
        adv_vehicle_id = config.get('adv_vehicle_id')
        visualizer.plot_static_trajectories(
            config['speed_colors'], config['lane_colors'], ego_vehicle_id, adv_vehicle_id)
        save_path = output_dir/f"visualization.png"
        print(f"Saving plot to: {save_path}")
        plt.savefig(str(save_path), dpi=config['dpi'], bbox_inches='tight', pad_inches=0)
    
    print("Visualization complete!")


if __name__ == "__main__":
    # path_to_config = "vis_configs/France_Paris_PedestrianCrossing.yaml"
    # path_to_config = "vis_configs/Germany_Rossfeld_AggressiveMerge.yaml"
    # path_to_config = "vis_configs/US_Arizona_HighwayCutin.yaml"
    # path_to_config = "vis_configs/US_AnnArbor_RoundaboutFailToYield.yaml"
    path_to_config = "vis_configs/US_SanDiego_RunRedLight.yaml"
    # path_to_config = "vis_configs/US_Chicago_UncoordinatedLeftTurn.yaml"
    # path_to_config = "vis_configs/Mcity_CyclistCrossing.yaml"
    # path_to_config = "vis_configs/Mcity_PedestrianCrossing.yaml"
    # path_to_config = "vis_configs/Mcity_UnprotectedLeftTurn.yaml"
    # path_to_config = "vis_configs/US_AnnArbor_RoundaboutFailToYield.yaml"
    # path_to_config = "vis_configs/Demo_AnnArborRoundabout.yaml"
    main(path_to_config)