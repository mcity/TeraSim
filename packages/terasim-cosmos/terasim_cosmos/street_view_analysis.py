import sumolib
import dotenv
import os
import requests
from openai import OpenAI
from pathlib import Path
import base64
import xml.etree.ElementTree as ET

# Load environment variables
dotenv.load_dotenv()

class StreetViewRetrievalAndAnalysis:
    """
    Class for retrieving and analyzing street view images using Google Street View API and GPT-4 Vision
    """
    
    def __init__(self):
        """
        Initialize StreetViewRetrievalAndAnalysis
        
        Args:
            google_maps_api_key: Google Maps API key for street view retrieval
            openai_api_key: OpenAI API key for image analysis
        """
        # Load API keys from environment if not provided
        self.google_maps_api_key = os.getenv('GOOGLE_MAPS_API_KEY')
        self.openai_api_key = os.getenv('OPENAI_API_KEY')
        
        if not self.google_maps_api_key:
            raise ValueError("Google Maps API key is required")
        if not self.openai_api_key:
            raise ValueError("OpenAI API key is required")
            
        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.openai_api_key)
    
    def get_street_view_image(self, latitude: float, longitude: float, heading: int = 0, pitch: int = 0, fov: int = 90) -> bytes:
        """
        Get a street view image from Google Street View API
        
        Args:
            latitude: Latitude coordinate
            longitude: Longitude coordinate
            heading: Heading angle in degrees (0-360)
            pitch: Pitch angle in degrees (-90 to 90)
            fov: Field of view in degrees (10-120)
            
        Returns:
            Image data as bytes
        """
        url = f"https://maps.googleapis.com/maps/api/streetview"
        params = {
            'size': '600x400',  # Image size
            'location': f'{latitude},{longitude}',
            'heading': heading,
            'pitch': pitch,
            'fov': fov,
            'key': self.google_maps_api_key
        }
        
        response = requests.get(url, params=params)
        if response.status_code == 200:
            return response.content
        else:
            raise Exception(f"Failed to get street view image: {response.status_code}")
    
    def analyze_image_with_llm(self, image_data: list) -> str:
        """
        Analyze the image using GPT-4 Vision and generate environment description
        
        Args:
            image_data_list: List of image data as bytes
            
        Returns:
            Environment description as string
        """
        # Convert images to base64
        base64_image = base64.b64encode(image_data).decode('utf-8')
        
        # Construct message content with multiple images
        content = [
            {
                "type": "text", 
                "text": "Please describe the environment and setting of this street view image. Focus on static elements like buildings, roads, vegetation, weather conditions, and overall atmosphere. Ignore any moving objects or people. This description will be used as a prompt for video generation."
            },
        ]
        
        # Add each image to the content
        content.append({
            "type": "image_url",
            "image_url": {
                "url": f"data:image/jpeg;base64,{base64_image}"
            }
        })
        
        response = self.client.chat.completions.create(
            model="gpt-4o",
            messages=[{
                "role": "user",
                "content": content
            }],
            max_tokens=500
        )
        
        return response.choices[0].message.content
    
    def get_vehicle_position_at_time(self, path_to_fcd: Path,
                                     vehicle_id: str,
                                     target_time: float) -> tuple:
        """
        Get vehicle position and angle data from FCD file at a specific time

        Args:
            path_to_fcd: Path to FCD XML file
            vehicle_id: Vehicle ID to track
            target_time: Target time in seconds

        Returns:
            Tuple of (x, y, angle) for vehicle position, or None if not found
        """
        tree = ET.parse(path_to_fcd)
        root = tree.getroot()
        all_timesteps = list(root.findall("timestep"))
        all_timesteps.sort(key=lambda x: float(x.get("time")))

        # Find the closest timestep to target_time
        target_timestep = None
        min_diff = float('inf')
        for timestep in all_timesteps:
            time_diff = abs(float(timestep.get("time")) - target_time)
            if time_diff < min_diff:
                min_diff = time_diff
                target_timestep = timestep

        if target_timestep is None:
            return None

        # Find the vehicle in this timestep
        for vehicle in target_timestep.findall("vehicle"):
            if vehicle.get("id") == vehicle_id:
                return (
                    float(vehicle.get("x")),
                    float(vehicle.get("y")),
                    float(vehicle.get("angle"))
                )
        return None
    
    def get_streetview_image_and_description(self, path_to_output: Path,
                                             path_to_fcd: Path,
                                             path_to_map: Path,
                                             vehicle_id: str = None,
                                             target_time: float = 0.0) -> str:
        """
        Get street view image and generate environment description at a specific time

        Args:
            path_to_output: Output directory path
            path_to_fcd: Path to FCD XML file
            path_to_map: Path to SUMO network file
            vehicle_id: Vehicle ID to track
            target_time: Target time in seconds to capture street view

        Returns:
            Environment description as string
        """
        # Load SUMO network
        sumo_net = sumolib.net.readNet(path_to_map)

        # Get vehicle position at target time
        vehicle_position = self.get_vehicle_position_at_time(
            path_to_fcd=path_to_fcd,
            vehicle_id=vehicle_id,
            target_time=target_time
        )

        if vehicle_position is None:
            return "Vehicle not found at the specified time"

        x, y, angle = vehicle_position
        # Convert coordinates to lat/lon
        lon, lat = sumo_net.convertXY2LonLat(x, y)

        # Get street view image from front view
        print(f"Processing location at lon: {lon}, lat: {lat}, angle: {angle}")

        # Get street view images from 6 directions around the vehicle
        camera_angle_list = {'front': 0, "front_left": -66, "front_right": 66, "rear": 180, "rear_left": -152, "rear_right": 152}
        fov_list = {'front': 120, "front_left": 120, "front_right": 120, "rear": 30, "rear_left": 70, "rear_right": 70}
        default_prompt_list = {
            "front": "The video is captured from a camera mounted on a car. The camera is facing forward. ",
            "front_left": "The video is captured from a camera mounted on a car. The camera is facing to the left. ",
            "front_right": "The video is captured from a camera mounted on a car. The camera is facing to the right. ",
            "rear": "The video is captured from a camera mounted on a car. The camera is facing backwards. ",
            "rear_left": "The video is captured from a camera mounted on a car. The camera is facing the rear left side. ",
            "rear_right": "The video is captured from a camera mounted on a car. The camera is facing the rear right side. ",
        }

        descriptions = []
        for camera_name, camera_angle in camera_angle_list.items():
            try:
                heading = angle + camera_angle
                fov = fov_list[camera_name]
                image_data = self.get_street_view_image(lat, lon, heading=heading, fov=fov)

                # Save image to file
                image_filename = f"streetview_image_{camera_name}.jpg"
                with open(path_to_output / image_filename, 'wb') as f:
                    f.write(image_data)
                print(f"Street view image saved as {path_to_output / image_filename}")

                # Generate environment description
                description = self.analyze_image_with_llm(image_data)
                # Add default prompt at the beginning
                full_description = default_prompt_list[camera_name] + description
                desc_filename = f"prompt_{camera_name}.txt"
                with open(path_to_output / desc_filename, 'w') as f:
                    f.write(full_description)
                print(f"Environment description saved as {path_to_output / desc_filename}")

                descriptions.append(f"{camera_name}: {full_description}")

            except Exception as e:
                print(f"Error retrieving street view for {camera_name}: {e}")
                descriptions.append(f"{camera_name}: Failed to retrieve street view")

        # Combine all descriptions
        combined_description = "\n\n".join(descriptions)
        return combined_description
