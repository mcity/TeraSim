from pyproj import Transformer

def sumo_to_latlon(x, y, net_offset_x=-15432887.11, net_offset_y=-4183426.91):
    # Add netOffset to the coordinates
    x_with_offset = x + net_offset_x
    y_with_offset = y + net_offset_y
    
    # Define the source projection (SUMO's projection)
    src_proj = "+proj=merc +a=6378137 +b=6378137 +lat_ts=0.0 +lon_0=0.0 +x_0=0.0 +y_0=0 +k=1.0 +units=m +nadgrids=@null +wktext +no_defs"
    
    # Define the target projection (WGS84 - standard lat/lon)
    dst_proj = "EPSG:4326"
    
    # Create transformer
    transformer = Transformer.from_crs(src_proj, dst_proj, always_xy=True)
    
    # Transform coordinates
    lon, lat = transformer.transform(x_with_offset, y_with_offset)
    
    return lat, lon

# Example usage
if __name__ == "__main__":
    # Your SUMO coordinates
    x = 19197.652417078865
    y = 1293.6773011692458
    
    # Convert to lat/lon
    lat, lon = sumo_to_latlon(x, y)
    
    print(f"SUMO coordinates: x={x}, y={y}")
    print(f"Coordinates with offset: x={x + -15432887.11}, y={y + -4183426.91}")
    print(f"Lat/Lon coordinates: lat={lat:.6f}, lon={lon:.6f}") 