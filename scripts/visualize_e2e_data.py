#!/usr/bin/env python3

import cv2
import numpy as np
import csv
import argparse
from pathlib import Path
import matplotlib.pyplot as plt
from typing import List, Tuple

class E2EDataVisualizer:
    def __init__(self, dataset_path: str):
        self.dataset_path = Path(dataset_path)
        self.images_dir = self.dataset_path / 'images'
        self.waypoints_dir = self.dataset_path / 'path'

        if not self.images_dir.exists() or not self.waypoints_dir.exists():
            raise ValueError(f"Dataset directory not found or incomplete: {dataset_path}")

        # Get list of sample files
        self.image_files = sorted(list(self.images_dir.glob('*.png')))
        self.waypoint_files = sorted(list(self.waypoints_dir.glob('*.csv')))

        if len(self.image_files) != len(self.waypoint_files):
            print(f"Warning: Number of images ({len(self.image_files)}) != waypoints ({len(self.waypoint_files)})")

        self.num_samples = min(len(self.image_files), len(self.waypoint_files))
        print(f"Found {self.num_samples} samples in {dataset_path}")

        self.current_index = 0

    def load_waypoints(self, waypoint_file: Path) -> List[Tuple[float, float]]:
        """Load waypoints from CSV file"""
        waypoints = []
        with open(waypoint_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                x = float(row['x'])
                y = float(row['y'])
                waypoints.append((x, y))
        return waypoints

    def create_waypoint_visualization(self, waypoints: List[Tuple[float, float]], size=(400, 400)) -> np.ndarray:

        vis = np.ones((size[1], size[0], 3), dtype=np.uint8) * 255

        if len(waypoints) == 0:
            return vis

        xs = [wp[0] for wp in waypoints]
        ys = [wp[1] for wp in waypoints]

        if len(xs) > 0:
            x_min, x_max = -1.0, 5.0
            y_min, y_max = -3.0, 3.0

            scale_x = (size[0] - 40) / (x_max - x_min) if x_max != x_min else 1.0
            scale_y = (size[1] - 40) / (y_max - y_min) if y_max != y_min else 1.0
            scale = min(scale_x, scale_y)

            def world_to_image(x, y):
                img_x = int((y - y_min) * scale + 20)
                img_y = int(size[1] - ((x - x_min) * scale + 20))
                return img_x, img_y

            origin = world_to_image(0, 0)
            cv2.circle(vis, origin, 8, (0, 0, 255), -1)
            cv2.putText(vis, 'Robot', (origin[0] + 10, origin[1]),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            self._draw_grid(vis, x_min, x_max, y_min, y_max, scale, size, world_to_image)

            points = []
            for i, (x, y) in enumerate(waypoints):
                pt = world_to_image(x, y)
                points.append(pt)

                color = (0, 200, 0)
                cv2.circle(vis, pt, 5, color, -1)

                cv2.putText(vis, str(i+1), (pt[0] - 10, pt[1] - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)

            for i in range(len(points) - 1):
                cv2.line(vis, points[i], points[i+1], (100, 100, 100), 1)

        return vis

    def _draw_grid(self, vis, x_min, x_max, y_min, y_max, scale, size, world_to_image):
        grid_interval = 1.0

        y = y_min + (grid_interval - (y_min % grid_interval))
        while y < y_max:
            pt1 = world_to_image(x_min, y)
            pt2 = world_to_image(x_max, y)
            cv2.line(vis, pt1, pt2, (220, 220, 220), 1)
            y += grid_interval

        x = x_min + (grid_interval - (x_min % grid_interval))
        while x < x_max:
            pt1 = world_to_image(x, y_min)
            pt2 = world_to_image(x, y_max)
            cv2.line(vis, pt1, pt2, (220, 220, 220), 1)
            x += grid_interval

        # Draw axis labels
        cv2.putText(vis, 'Y (left) ->', (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
        cv2.putText(vis, 'X (forward)', (size[0]//2 - 50, size[1] - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

    def visualize_sample(self, index: int):
        """Visualize a single sample"""
        if index < 0 or index >= self.num_samples:
            print(f"Index {index} out of range [0, {self.num_samples-1}]")
            return

        # Load image
        image_path = self.image_files[index]
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)

        if image is None:
            print(f"Failed to load image: {image_path}")
            return

        # Convert BGRA to BGR if needed
        if image.shape[2] == 4:
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        # Load waypoints
        waypoint_path = self.waypoint_files[index]
        waypoints = self.load_waypoints(waypoint_path)

        # Create waypoint visualization
        waypoint_vis = self.create_waypoint_visualization(waypoints)

        # Resize if needed to match heights
        if image.shape[0] != waypoint_vis.shape[0]:
            scale = image.shape[0] / waypoint_vis.shape[0]
            new_width = int(waypoint_vis.shape[1] * scale)
            waypoint_vis = cv2.resize(waypoint_vis, (new_width, image.shape[0]))

        # Concatenate images horizontally
        combined = np.hstack([image, waypoint_vis])

        # Add text info
        info_text = f"Sample {index+1}/{self.num_samples} - {image_path.stem}"
        cv2.putText(combined, info_text, (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(combined, info_text, (10, 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        # Add control instructions
        instructions = "Controls: [N]ext, [P]rev, [Q]uit, [0-9] Jump to sample"
        cv2.putText(combined, instructions, (10, combined.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        cv2.putText(combined, instructions, (10, combined.shape[0] - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        return combined

    def run(self):
        """Main visualization loop"""
        cv2.namedWindow('E2E Data Visualization', cv2.WINDOW_NORMAL)

        while True:
            vis = self.visualize_sample(self.current_index)
            if vis is None:
                break

            cv2.imshow('E2E Data Visualization', vis)

            # Wait for key press
            key = cv2.waitKey(0) & 0xFF

            if key == ord('q') or key == 27:  # q or ESC
                break
            elif key == ord('n') or key == 83:  # n or right arrow
                self.current_index = min(self.current_index + 1, self.num_samples - 1)
            elif key == ord('p') or key == 81:  # p or left arrow
                self.current_index = max(self.current_index - 1, 0)
            elif key == ord('j'):  # j for jump
                print(f"Enter sample number (1-{self.num_samples}): ", end='')
                try:
                    num = int(input())
                    self.current_index = max(0, min(num - 1, self.num_samples - 1))
                except ValueError:
                    print("Invalid input")

        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='Visualize E2E planner dataset')
    parser.add_argument('dataset_path', type=str, help='Path to dataset directory')
    parser.add_argument('--sample', type=int, default=0, help='Starting sample index (default: 0)')

    args = parser.parse_args()

    try:
        visualizer = E2EDataVisualizer(args.dataset_path)
        visualizer.current_index = args.sample
        visualizer.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
