#!/usr/bin/env python3
import csv
import math
import argparse

def generate_path(output_file, dt=0.05):
    # Parameters
    total_time = 10.0
    
    # State
    x = 0.0
    y = 0.0
    yaw = 0.0
    
    path_points = []
    
    t = 0.0
    while t <= total_time + 1e-6:
        # Determine inputs
        v = 1.0
        omega = 0.0
        
        if t <= 5.0:
            # 0-5s: Straight
            omega = 0.0
        else:
            # 5-10s: Turn
            omega = 0.2
            
        # Store current state
        path_points.append([x, y, yaw, v])
        
        # Update state for next step (Euler integration)
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw += omega * dt
        
        t += dt
        
    # Save to CSV
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'yaw', 'v'])
        writer.writerows(path_points)
        
    print(f"Generated {len(path_points)} points to {output_file}")
    print("Trajectory Description:")
    print("  0-5s: Straight at 1 m/s")
    print("  5-10s: Turn at 1 m/s, 0.2 rad/s")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate synthetic test path')
    parser.add_argument('output', default='test_path.csv', nargs='?', help='Output CSV file')
    args = parser.parse_args()
    
    generate_path(args.output)
