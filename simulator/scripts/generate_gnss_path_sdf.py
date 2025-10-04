#!/usr/bin/env python3

import csv
import math
import sys

def lat_lon_to_xy(lat, lon, origin_lat, origin_lon):
    EARTH_RADIUS = 6371000.0
    
    lat_diff = math.radians(lat - origin_lat)
    lon_diff = math.radians(lon - origin_lon)
    
    x = EARTH_RADIUS * lon_diff * math.cos(math.radians(origin_lat))
    y = EARTH_RADIUS * lat_diff
    
    return x, y

def generate_gnss_path_sdf(csv_file):
    # Read first GNSS point to use as origin
    first_lat = None
    first_lon = None
    points = []
    
    with open(csv_file, 'r') as file:
        csv_reader = csv.DictReader(file)
        
        for i, row in enumerate(csv_reader):
            lat = float(row['Latitude'])
            lon = float(row['Longitude'])
            
            if i == 0:
                first_lat = lat
                first_lon = lon
            
            x, y = lat_lon_to_xy(lat, lon, first_lat, first_lon)
            points.append((x, y))
    
    # Generate SDF model for GNSS path
    sdf_content = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="gnss_path">
    <static>true</static>
"""
    
    # Create spheres at each GNSS point
    for i, (x, y) in enumerate(points):
        sdf_content += f"""
    <link name="waypoint_{i}">
      <pose>{x} {y} 0.5 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.1</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
    </link>
"""
    
    # Create lines between consecutive points
    for i in range(len(points) - 1):
        x1, y1 = points[i]
        x2, y2 = points[i + 1]
        
        # Calculate midpoint
        mid_x = (x1 + x2) / 2
        mid_y = (y1 + y2) / 2
        
        # Calculate length
        length = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
        # Calculate rotation
        angle = math.atan2(y2 - y1, x2 - x1)
        
        sdf_content += f"""
    <link name="path_segment_{i}">
      <pose>{mid_x} {mid_y} 0.5 0 0 {angle}</pose>
      <visual name="visual">
        <geometry>
          <box>
            <size>{length} 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>1 0 0 1</specular>
        </material>
      </visual>
    </link>
"""
    
    sdf_content += """
  </model>
</sdf>
"""
    
    return sdf_content

if __name__ == "__main__":
    csv_file = sys.argv[1] if len(sys.argv) > 1 else "/home/ubuntu/formula_ws/shihou_gnssnav.csv"
    
    # Count points for the message
    point_count = 0
    with open(csv_file, 'r') as file:
        csv_reader = csv.DictReader(file)
        point_count = sum(1 for _ in csv_reader)
    
    sdf_content = generate_gnss_path_sdf(csv_file)
    
    output_file = "/home/ubuntu/formula_ws/src/simulator/models/gnss_path/model.sdf"
    
    import os
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    with open(output_file, 'w') as f:
        f.write(sdf_content)
    
    # Create model.config
    config_content = """<?xml version="1.0"?>
<model>
  <name>gnss_path</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>GNSS Path Generator</name>
  </author>
  <description>
    GNSS path visualization model
  </description>
</model>
"""
    
    config_file = "/home/ubuntu/formula_ws/src/simulator/models/gnss_path/model.config"
    with open(config_file, 'w') as f:
        f.write(config_content)
    
    print(f"Generated GNSS path model with {point_count} points")