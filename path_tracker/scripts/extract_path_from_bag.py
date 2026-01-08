#!/usr/bin/env python3
import sys
import argparse
import csv
import math

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)
    return storage_options, converter_options

def quaternion_to_yaw(q):
    # q is object with x, y, z, w
    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def main():
    parser = argparse.ArgumentParser(description='Extract path from rosbag')
    parser.add_argument('bag_path', help='Path to rosbag')
    parser.add_argument('output', help='Output CSV file path')
    parser.add_argument('--topic', default='/gnss_path', help='Topic name to extract')
    parser.add_argument('--start', type=float, default=50.0, help='Start time offset from bag start (seconds)')
    parser.add_argument('--duration', type=float, default=20.0, help='Duration to extract (seconds)')
    args = parser.parse_args()

    # 1. Open Bag and Get Metadata
    try:
        reader = rosbag2_py.SequentialReader()
        storage_options, converter_options = get_rosbag_options(args.bag_path)
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    # Get bag start time
    metadata = reader.get_metadata()
    t_obj = metadata.starting_time
    
    bag_start_time_ns = 0
    if hasattr(t_obj, 'nanoseconds'):
        bag_start_time_ns = t_obj.nanoseconds
    elif hasattr(t_obj, 'nanoseconds_since_epoch'):
        bag_start_time_ns = t_obj.nanoseconds_since_epoch
    elif hasattr(t_obj, 'timestamp'):
        bag_start_time_ns = int(t_obj.timestamp() * 1e9)
    else:
        # Fallback or debug
        print(f"Unknown time object type: {type(t_obj)}")
        print(f"Attributes: {dir(t_obj)}")
        # Try casting to int if it behaves like one
        try:
             bag_start_time_ns = int(t_obj)
        except:
             print("Could not convert starting_time to nanoseconds.")
             return
    
    start_window_ns = bag_start_time_ns + int(args.start * 1e9)
    end_window_ns = start_window_ns + int(args.duration * 1e9)
    
    print(f"Bag Start: {bag_start_time_ns}")
    print(f"Extract Window: {start_window_ns} - {end_window_ns}")
    print(f"Topic: {args.topic}")

    # 2. Setup Type Support
    topic_type = None
    for type_info in metadata.topics_with_message_count:
        if type_info.topic_metadata.name == args.topic:
            topic_type = type_info.topic_metadata.type
            break
            
    if topic_type is None:
        print(f"Topic {args.topic} not found in bag.")
        return

    try:
        msg_type_class = get_message(topic_type)
    except ImportError:
        print(f"Could not import message type: {topic_type}")
        return

    # 3. Read and Filter
    storage_filter = rosbag2_py.StorageFilter(topics=[args.topic])
    reader.set_filter(storage_filter)

    extracted_path = []

    last_x, last_y = None, None

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        if t < start_window_ns:
            continue
        if t > end_window_ns:
            break
            
        msg = deserialize_message(data, msg_type_class)
        
        # Handle different message types
        x, y, yaw = 0.0, 0.0, 0.0
        
        # Case A: PoseStamped
        if hasattr(msg, 'pose') and hasattr(msg.pose, 'position'): # PoseStamped
            x = msg.pose.position.x
            y = msg.pose.position.y
            q = msg.pose.orientation
            yaw = quaternion_to_yaw(q)
            extracted_path.append([x, y, yaw, 2.0])
            
        # Case B: Odometry
        elif hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'): # Odometry
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = quaternion_to_yaw(q)
            extracted_path.append([x, y, yaw, 2.0])

        # Case C: Path (nav_msgs/Path)
        elif hasattr(msg, 'poses'):
            for p in msg.poses:
                # Optional: Filter by pose timestamp if needed. 
                # For now, we take all poses in the message if the message itself is in the window.
                # Or better, if poses have timestamps, we can filter them specifically.
                p_ns = 0
                if hasattr(p.header.stamp, 'nanoseconds'): # Check if proper Time object
                     p_ns = p.header.stamp.nanoseconds
                elif hasattr(p.header.stamp, 'sec'): # builtin_interfaces/Time
                     p_ns = p.header.stamp.sec * 1000000000 + p.header.stamp.nanosec

                # If timestamp is valid and out of range, skip?
                # However, usually Path messages are snapshots. 
                # Let's just assume we want the points in this Path message.
                
                px = p.pose.position.x
                py = p.pose.position.y
                pq = p.pose.orientation
                pyaw = quaternion_to_yaw(pq)
                
                # Deduplicate logic inside loop
                if last_x is not None and abs(px - last_x) < 1e-3 and abs(py - last_y) < 1e-3:
                    continue
                extracted_path.append([px, py, pyaw, 2.0])
                last_x, last_y = px, py
            continue # specific handling for Path which adds multiple points

        # Case D: Point or simple Vector3
        elif hasattr(msg, 'x') and hasattr(msg, 'y'):
            x = msg.x
            y = msg.y
            yaw = 0.0
            extracted_path.append([x, y, yaw, 2.0])
            
        else:
            print(f"Unsupported message structure for {topic}: {type(msg)}")
            return

        # Simple Duplicate Filter (for single point messages)
        # Note: 'Path' case continues above, so this is only for single msg types
        x, y, yaw, _ = extracted_path[-1]
        if last_x is not None and abs(x - last_x) < 1e-3 and abs(y - last_y) < 1e-3:
            extracted_path.pop() # Remove the duplicate we just added
            continue
        last_x, last_y = x, y

    # 4. Save to CSV
    if not extracted_path:
        print("No messages found in time window.")
        return
        
    print(f"Extracted {len(extracted_path)} points.")
    with open(args.output, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'yaw', 'v'])
        writer.writerows(extracted_path)
    print(f"Saved to {args.output}")

if __name__ == '__main__':
    main()
