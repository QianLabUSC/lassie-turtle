#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy.interpolate import splprep, splev
import json
import sys

class TrajectoryInterpolator(Node):
    def __init__(self):
        super().__init__('trajectory_interpolator')
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/trajectory_points',
            10
        )
    
    def calculate_num_samples(self, control_points, resolution=0.01):
        """Adaptive sampling based on path length"""
        points = np.array(control_points).reshape(-1, 3)
        total_length = 0.0
        for i in range(len(points) - 1):
            total_length += np.linalg.norm(points[i+1] - points[i])
        
        num_samples = max(10, int(total_length / resolution))
        return num_samples
        
    def interpolate_and_publish(self, control_points, resolution=0.01):
        points = np.array(control_points).reshape(-1, 3)
        x, y, z = points[:, 0], points[:, 1], points[:, 2]
        
        # Adaptive sampling
        num_samples = self.calculate_num_samples(control_points, resolution)
        self.get_logger().info(
            f'Generating {num_samples} points (resolution={resolution}m)'
        )
        
        # Need at least k+1 points for spline of degree k
        k = min(3, len(points) - 1)
        tck, u = splprep([x, y, z], s=0, k=k)
        u_fine = np.linspace(0, 1, num_samples)
        x_fine, y_fine, z_fine = splev(u_fine, tck)
        
        interpolated = np.column_stack([x_fine, y_fine, z_fine]).flatten()
        
        msg = Float64MultiArray()
        msg.data = interpolated.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f'Published {num_samples} interpolated points')

def main():
    rclpy.init()
    node = TrajectoryInterpolator()
    
    # Check for JSON file argument
    if len(sys.argv) < 2:
        node.get_logger().error('Usage: python3 trajectory_interpolator.py <json_file>')
        node.get_logger().info('Example: python3 trajectory_interpolator.py trajectories/circle.json')
        return
    
    json_file = sys.argv[1]
    
    # Load trajectory from JSON
    try:
        with open(json_file, 'r') as f:
            data = json.load(f)
            control_points = data['waypoints']
            resolution = data.get('resolution', 0.01)
            name = data.get('name', 'unknown')
            
        node.get_logger().info(f'Loaded trajectory: {name}')
        node.interpolate_and_publish(control_points, resolution)
        
    except FileNotFoundError:
        node.get_logger().error(f'File not found: {json_file}')
        return
    except json.JSONDecodeError:
        node.get_logger().error(f'Invalid JSON file: {json_file}')
        return
    except KeyError as e:
        node.get_logger().error(f'Missing required field in JSON: {e}')
        return
    
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()