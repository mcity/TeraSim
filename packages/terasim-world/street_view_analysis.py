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
    
    def analyze_image_with_llm(self, image_data_list: list) -> str:
        """
        Analyze the image using GPT-4 Vision and generate environment description
        
        Args:
            image_data_list: List of image data as bytes
            
        Returns:
            Environment description as string
        """
        # Convert images to base64
        base64_image_list = [base64.b64encode(image_data).decode('utf-8') for image_data in image_data_list]
        
        # Construct message content with multiple images
        content = [
            {
                "type": "text", 
                "text": "Please describe the environment and setting of this street view image. Focus on static elements like buildings, roads, vegetation, weather conditions, and overall atmosphere. Ignore any moving objects or people. This description will be used as a prompt for video generation."
            },
        ]
        
        # Add each image to the content
        for base64_image in base64_image_list:
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
    
    def get_vehicle_position_view(self, path_to_fcd: Path, 
                                  vehicle_id: str, 
                                  timestep_start: int, 
                                  timestep_end: int, 
                                  timestep_interval: int = 10) -> list:
        """
        Get vehicle position and angle data from FCD file
        
        Args:
            path_to_fcd: Path to FCD XML file
            vehicle_id: Vehicle ID to track
            timestep_start: Starting timestep
            timestep_end: Ending timestep
            timestep_interval: Interval between timesteps
            
        Returns:
            List of tuples (x, y, angle) for vehicle positions
        """
        tree = ET.parse(path_to_fcd)
        root = tree.getroot()
        all_timesteps = list(root.findall("timestep"))
        all_timesteps.sort(key=lambda x: float(x.get("time")))
        target_timestep_list = all_timesteps[timestep_start:timestep_end:timestep_interval]
        
        vehicle_position_angle_list = []
        for target_timestep in target_timestep_list:
            for vehicle in target_timestep.findall("vehicle"):
                if vehicle.get("id") == vehicle_id:
                    vehicle_position_angle_list.append((
                        float(vehicle.get("x")), 
                        float(vehicle.get("y")), 
                        float(vehicle.get("angle"))
                    ))
        return vehicle_position_angle_list
    
    def get_streetview_image_and_description(self, path_to_output: Path, 
                                             path_to_fcd: Path,
                                             path_to_map: Path,
                                             vehicle_id: str = None,
                                             timestep_start: int = -40,
                                             timestep_end: int = -1,
                                             fov: int = 120) -> str:
        """
        Get street view images and generate environment description
        
        Args:
            path_to_output: Output directory path
            path_to_fcd: Path to FCD XML file
            path_to_map: Path to SUMO network file
            vehicle_id: Vehicle ID to track
            timestep_start: Starting timestep
            timestep_end: Ending timestep
            fov: Field of view for street view images
            
        Returns:
            Environment description as string
        """
        # Load SUMO network
        sumo_net = sumolib.net.readNet(path_to_map)

        # Get vehicle positions
        vehicle_position_angle_list = self.get_vehicle_position_view(
            path_to_fcd=path_to_fcd, 
            vehicle_id=vehicle_id, 
            timestep_start=timestep_start, 
            timestep_end=timestep_end, 
            timestep_interval=timestep_end - timestep_start + 1
        )

        # Convert coordinates to lat/lon
        vehicle_lon_lat_angle_list = []
        for x, y, angle in vehicle_position_angle_list:
            lon, lat = sumo_net.convertXY2LonLat(x, y)
            vehicle_lon_lat_angle_list.append((lon, lat, angle))

        # Get street view images
        image_data_list = []
        for i, (lon, lat, angle) in enumerate(vehicle_lon_lat_angle_list):
            image_data = self.get_street_view_image(lat, lon, heading=angle, fov=fov)
            image_data_list.append(image_data)

            # Save image to file
            with open(path_to_output / f"streetview_image_{i}.jpg", 'wb') as f:
                f.write(image_data)
            print(f"Street view image saved as {path_to_output} / streetview_image_{i}.jpg")

        # Generate environment description
        description = self.analyze_image_with_llm(image_data_list)
        with open(path_to_output / f"streetview_description.txt", 'w') as f:
            f.write(description)
            print(f"Environment description saved as {path_to_output} / streetview_description.txt")
        
        return description
