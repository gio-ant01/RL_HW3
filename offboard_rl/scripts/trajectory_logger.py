#!/usr/bin/env python3
"""
Trajectory Data Logger for PX4 Multi-Waypoint Planner

This node subscribes to PX4 topics and logs:
- Position (x, y, z)
- Velocity (vx, vy, vz)
- Acceleration (ax, ay, az)
- Attitude (yaw angle)

Data is saved to CSV files for later plotting.
"""

import rclpy
from rclpy.node import Node
import rclpy.qos
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
import numpy as np
import csv
from datetime import datetime
import math

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        
        # Configure QoS to match PX4's settings (BEST_EFFORT reliability)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Create subscriptions with correct QoS
        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.position_callback,
            qos_profile)
        
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)
        
        # Data storage
        self.data = {
            'time': [],
            'x': [],
            'y': [],
            'z': [],
            'vx': [],
            'vy': [],
            'vz': [],
            'ax': [],
            'ay': [],
            'az': [],
            'yaw': []
        }
        
        self.start_time = None
        self.current_yaw = 0.0
        self.message_count = 0
        
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"trajectory_data_{timestamp}.csv"
        
        self.get_logger().info(f"Trajectory Logger started. Logging to: {self.filename}")
        self.get_logger().info("Waiting for PX4 messages...")
        self.get_logger().info("Make sure MicroXRCEAgent is running and PX4 SITL is active!")
        self.get_logger().info("Press Ctrl+C to stop logging and save data.")
    
    def quat_to_euler(self, q):
        """
        Convert quaternion to Euler angles (roll, pitch, yaw)
        q = [w, x, y, z]
        Returns [roll, pitch, yaw] in radians
        """
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def position_callback(self, msg):
        """Callback for vehicle position messages"""
        # Initialize start time on first message
        if self.start_time is None:
            self.start_time = msg.timestamp / 1e6  # Convert to seconds
            self.get_logger().info("✓ Started receiving data! Logging in progress...")
        
        # Calculate elapsed time
        current_time = msg.timestamp / 1e6
        elapsed_time = current_time - self.start_time
        
        # Store data
        self.data['time'].append(elapsed_time)
        self.data['x'].append(msg.x)
        self.data['y'].append(msg.y)
        self.data['z'].append(msg.z)
        self.data['vx'].append(msg.vx)
        self.data['vy'].append(msg.vy)
        self.data['vz'].append(msg.vz)
        self.data['ax'].append(msg.ax)
        self.data['ay'].append(msg.ay)
        self.data['az'].append(msg.az)
        self.data['yaw'].append(self.current_yaw)
        
        self.message_count += 1
        
        # Log progress every 100 samples
        if self.message_count % 100 == 0:
            self.get_logger().info(f"Logged {self.message_count} samples ({elapsed_time:.1f}s)...")
    
    def attitude_callback(self, msg):
        """Callback for vehicle attitude messages"""
        # Extract yaw from quaternion
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        self.current_yaw = self.quat_to_euler(q)
    
    def save_data(self):
        """Save logged data to CSV file"""
        if len(self.data['time']) == 0:
            self.get_logger().warn("No data to save!")
            self.get_logger().warn("Possible issues:")
            self.get_logger().warn("  1. MicroXRCEAgent not running")
            self.get_logger().warn("  2. PX4 SITL not started")
            self.get_logger().warn("  3. QoS mismatch (should be fixed now)")
            self.get_logger().warn("Check: ros2 topic list | grep fmu")
            return
        
        self.get_logger().info(f"Saving {len(self.data['time'])} samples to {self.filename}...")
        
        with open(self.filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            
            # Write header
            writer.writerow(['time', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'yaw'])
            
            # Write data
            for i in range(len(self.data['time'])):
                writer.writerow([
                    self.data['time'][i],
                    self.data['x'][i],
                    self.data['y'][i],
                    self.data['z'][i],
                    self.data['vx'][i],
                    self.data['vy'][i],
                    self.data['vz'][i],
                    self.data['ax'][i],
                    self.data['ay'][i],
                    self.data['az'][i],
                    self.data['yaw'][i]
                ])
        
        self.get_logger().info(f"Data saved successfully to {self.filename}")
        
        # Print summary statistics
        self.print_summary()
    
    def print_summary(self):
        """Print summary statistics of logged data"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("DATA LOGGING SUMMARY")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Total samples: {len(self.data['time'])}")
        self.get_logger().info(f"Duration: {self.data['time'][-1]:.2f} seconds")
        self.get_logger().info(f"Sample rate: {len(self.data['time'])/self.data['time'][-1]:.1f} Hz")
        self.get_logger().info("\nPosition range:")
        self.get_logger().info(f"  X: [{min(self.data['x']):.2f}, {max(self.data['x']):.2f}] m")
        self.get_logger().info(f"  Y: [{min(self.data['y']):.2f}, {max(self.data['y']):.2f}] m")
        self.get_logger().info(f"  Z: [{min(self.data['z']):.2f}, {max(self.data['z']):.2f}] m")
        self.get_logger().info("\nVelocity range:")
        self.get_logger().info(f"  VX: [{min(self.data['vx']):.2f}, {max(self.data['vx']):.2f}] m/s")
        self.get_logger().info(f"  VY: [{min(self.data['vy']):.2f}, {max(self.data['vy']):.2f}] m/s")
        self.get_logger().info(f"  VZ: [{min(self.data['vz']):.2f}, {max(self.data['vz']):.2f}] m/s")
        self.get_logger().info("\nMax acceleration:")
        self.get_logger().info(f"  AX: {max(abs(min(self.data['ax'])), abs(max(self.data['ax']))):.2f} m/s²")
        self.get_logger().info(f"  AY: {max(abs(min(self.data['ay'])), abs(max(self.data['ay']))):.2f} m/s²")
        self.get_logger().info(f"  AZ: {max(abs(min(self.data['az'])), abs(max(self.data['az']))):.2f} m/s²")
        self.get_logger().info("="*60 + "\n")

def main(args=None):
    rclpy.init(args=args)
    
    logger = TrajectoryLogger()
    
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        logger.get_logger().info("\nStopping logger...")
        logger.save_data()
    finally:
        logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()