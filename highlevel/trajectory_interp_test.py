#!/usr/bin/env python3
import numpy as np
from scipy.interpolate import splprep, splev
import json
import sys

def calculate_num_samples(control_points, resolution=0.01):
    """Adaptive sampling based on path length"""
    points = np.array(control_points).reshape(-1, 3)
    total_length = 0.0
    for i in range(len(points) - 1):
        total_length += np.linalg.norm(points[i+1] - points[i])
    
    num_samples = max(10, int(total_length / resolution))
    print(f"Number of samples: {num_samples}")
    return num_samples

def interpolate_trajectory(control_points, resolution=0.01):
    points = np.array(control_points).reshape(-1, 3)
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    
    # Adaptive sampling
    num_samples = calculate_num_samples(control_points, resolution)
    
    # Need at least k+1 points for spline of degree k
    k = min(3, len(points) - 1)
    print(f"Using spline degree: {k}")
    
    tck, u = splprep([x, y, z], s=0, k=k)
    u_fine = np.linspace(0, 1, num_samples)
    x_fine, y_fine, z_fine = splev(u_fine, tck)
    
    return x_fine, y_fine, z_fine

def main():
    if len(sys.argv) < 2:
        print('Usage: python3 test_interpolator.py <json_file>')
        print('Example: python3 test_interpolator.py trajectories/circle.json')
        return
    
    json_file = sys.argv[1]
    
    # Load trajectory from JSON
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
            control_points = data['waypoints']
            resolution = data.get('resolution', 0.01)
            name = data.get('name', 'unknown')
        
        print(f"\n{'='*60}")
        print(f"Testing Trajectory: {name}")
        print(f"Resolution: {resolution}m")
        print(f"Control Points: {len(control_points)//3}")
        print(f"{'='*60}\n")
        
        # Interpolate
        x_fine, y_fine, z_fine = interpolate_trajectory(control_points, resolution)
        
        # Print first 10 points
        print(f"\nFirst 10 interpolated points:")
        for i in range(min(10, len(x_fine))):
            print(f"  [{i:3d}] x={x_fine[i]:7.4f}, y={y_fine[i]:7.4f}, z={z_fine[i]:7.4f}")
        
        # Print last 10 points
        print(f"\nLast 10 interpolated points:")
        for i in range(max(0, len(x_fine)-10), len(x_fine)):
            print(f"  [{i:3d}] x={x_fine[i]:7.4f}, y={y_fine[i]:7.4f}, z={z_fine[i]:7.4f}")
        
        # Print what would be published to ROS
        interpolated = np.column_stack([x_fine, y_fine, z_fine]).flatten()
        print(f"\n{'='*60}")
        print(f"ROS Message Info:")
        print(f"  Total array size: {len(interpolated)} values")
        print(f"  Total waypoints: {len(x_fine)} points")
        print(f"  Format: [x1, y1, z1, x2, y2, z2, ...]")
        print(f"{'='*60}\n")
        
    except FileNotFoundError:
        print(f'Error: File not found: {json_file}')
    except json.JSONDecodeError:
        print(f'Error: Invalid JSON file: {json_file}')
    except KeyError as e:
        print(f'Error: Missing required field in JSON: {e}')
    except Exception as e:
        print(f'Error: {e}')

if __name__ == '__main__':
    main()