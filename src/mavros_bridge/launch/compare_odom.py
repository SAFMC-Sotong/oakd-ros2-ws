#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import time
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import sys

class OdometryRecorder(Node):
    def __init__(self, duration=60):
        super().__init__('odometry_recorder')
        
        # Recording duration in seconds
        self.duration = duration
        self.start_time = time.time()
        
        # Create subscriptions
        self.sub1 = self.create_subscription(Odometry, '/zed_node/odom', self.odom1_callback, 10)
        self.sub2 = self.create_subscription(Odometry, '/mavros/odometry/in', self.odom2_callback, 10)
        
        # Data storage
        self.data1 = []
        self.data2 = []
        
        # Create timer for checking recording completion
        self.timer = self.create_timer(1.0, self.check_recording_complete)
        
        self.get_logger().info(f'Recording odometry data for {duration} seconds...')
    
    def odom1_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        self.data1.append({
            'ros_time': timestamp,
            'wall_time': time.time(),
            'pos_x': msg.pose.pose.position.x,
            'pos_y': msg.pose.pose.position.y,
            'pos_z': msg.pose.pose.position.z,
            'quat_x': msg.pose.pose.orientation.x,
            'quat_y': msg.pose.pose.orientation.y,
            'quat_z': msg.pose.pose.orientation.z,
            'quat_w': msg.pose.pose.orientation.w,
            'lin_vel_x': msg.twist.twist.linear.x,
            'lin_vel_y': msg.twist.twist.linear.y,
            'lin_vel_z': msg.twist.twist.linear.z,
            'ang_vel_x': msg.twist.twist.angular.x,
            'ang_vel_y': msg.twist.twist.angular.y,
            'ang_vel_z': msg.twist.twist.angular.z
        })
    
    def odom2_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        self.data2.append({
            'ros_time': timestamp,
            'wall_time': time.time(),
            'pos_x': msg.pose.pose.position.x,
            'pos_y': msg.pose.pose.position.y,
            'pos_z': msg.pose.pose.position.z,
            'quat_x': msg.pose.pose.orientation.x,
            'quat_y': msg.pose.pose.orientation.y,
            'quat_z': msg.pose.pose.orientation.z,
            'quat_w': msg.pose.pose.orientation.w,
            'lin_vel_x': msg.twist.twist.linear.x,
            'lin_vel_y': msg.twist.twist.linear.y,
            'lin_vel_z': msg.twist.twist.linear.z,
            'ang_vel_x': msg.twist.twist.angular.x,
            'ang_vel_y': msg.twist.twist.angular.y,
            'ang_vel_z': msg.twist.twist.angular.z
        })
    
    def check_recording_complete(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration:
            self.get_logger().info(f'Recording complete. Collected {len(self.data1)} and {len(self.data2)} messages.')
            self.analyze_data()
            rclpy.shutdown()
    
    def analyze_data(self):
        if not self.data1 or not self.data2:
            self.get_logger().error("Insufficient data to analyze")
            return
        
        # Convert to pandas DataFrames
        df1 = pd.DataFrame(self.data1)
        df2 = pd.DataFrame(self.data2)
        
        # Calculate message rates
        time_diffs1 = np.diff(df1['wall_time'])
        time_diffs2 = np.diff(df2['wall_time'])
        
        rate1 = 1.0 / np.mean(time_diffs1) if len(time_diffs1) > 0 else 0
        rate2 = 1.0 / np.mean(time_diffs2) if len(time_diffs2) > 0 else 0
        
        rate_std1 = np.std(1.0 / time_diffs1) if len(time_diffs1) > 0 else 0
        rate_std2 = np.std(1.0 / time_diffs2) if len(time_diffs2) > 0 else 0
        
        # Calculate ROS timestamp differences
        df1_sorted = df1.sort_values('ros_time')
        df2_sorted = df2.sort_values('ros_time')
        
        # Set up plotting style
        sns.set(style="whitegrid")
        
        # Create comparison plots
        fig, axs = plt.subplots(3, 2, figsize=(12, 12))
        
        # 1. Plot message rates
        sns.histplot(1.0 / time_diffs1, ax=axs[0, 0], kde=True, color='blue', label=f'/zed_node/odom: {rate1:.2f}±{rate_std1:.2f} Hz')
        sns.histplot(1.0 / time_diffs2, ax=axs[0, 0], kde=True, color='red', alpha=0.6, label=f'/mavros/odometry/in: {rate2:.2f}±{rate_std2:.2f} Hz')
        axs[0, 0].set_title('Message Rate Distribution')
        axs[0, 0].set_xlabel('Rate (Hz)')
        axs[0, 0].legend()
        
        # 2. Plot position trajectories
        axs[0, 1].plot(df1['pos_x'], df1['pos_y'], 'b-', label='/zed_node/odom')
        axs[0, 1].plot(df2['pos_x'], df2['pos_y'], 'r-', label='/mavros/odometry/in')
        axs[0, 1].set_title('Position Trajectories (X-Y)')
        axs[0, 1].set_xlabel('X Position')
        axs[0, 1].set_ylabel('Y Position')
        axs[0, 1].legend()
        axs[0, 1].axis('equal')
        
        # 3. Plot position differences over time
        # Create synchronized data for comparison
        common_times = []
        pos_diffs = []
        angle_diffs = []
        
        for i, row1 in df1.iterrows():
            # Find closest timestamp in df2
            idx = (df2['ros_time'] - row1['ros_time']).abs().idxmin()
            row2 = df2.loc[idx]
            
            # Only compare if timestamps are close enough
            if abs(row1['ros_time'] - row2['ros_time']) < 0.1:  # Within 100ms
                common_times.append(row1['ros_time'] - df1['ros_time'].min())
                
                # Calculate position difference
                pos_diff = np.sqrt(
                    (row1['pos_x'] - row2['pos_x'])**2 +
                    (row1['pos_y'] - row2['pos_y'])**2 +
                    (row1['pos_z'] - row2['pos_z'])**2
                )
                pos_diffs.append(pos_diff)
                
                # Calculate orientation difference
                q1 = [row1['quat_x'], row1['quat_y'], row1['quat_z'], row1['quat_w']]
                q2 = [row2['quat_x'], row2['quat_y'], row2['quat_z'], row2['quat_w']]
                
                dot_product = sum(x*y for x, y in zip(q1, q2))
                dot_product = max(min(dot_product, 1.0), -1.0)
                angle_diff = 2 * np.arccos(abs(dot_product)) * 180/np.pi
                angle_diffs.append(angle_diff)
        
        axs[1, 0].plot(common_times, pos_diffs, 'g-')
        axs[1, 0].set_title('Position Difference')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Distance (m)')
        
        axs[1, 1].plot(common_times, angle_diffs, 'm-')
        axs[1, 1].set_title('Orientation Difference')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Angle (degrees)')
        
        # 4. Plot timestamp differences
        time_offsets = []
        for i, row1 in df1.iterrows():
            idx = (df2['wall_time'] - row1['wall_time']).abs().idxmin()
            row2 = df2.loc[idx]
            if abs(row1['wall_time'] - row2['wall_time']) < 0.1:
                time_diff = abs(row1['ros_time'] - row2['ros_time'])
                time_offsets.append(time_diff * 1000)  # Convert to ms
        
        if time_offsets:
            sns.histplot(time_offsets, ax=axs[2, 0], kde=True)
            axs[2, 0].set_title('Timestamp Difference Distribution')
            axs[2, 0].set_xlabel('Difference (ms)')
            axs[2, 0].axvline(np.mean(time_offsets), color='r', linestyle='--', 
                             label=f'Mean: {np.mean(time_offsets):.2f}ms')
            axs[2, 0].legend()
        
        # 5. Plot velocity comparison
        vel_diffs = []
        for i, row1 in df1.iterrows():
            idx = (df2['ros_time'] - row1['ros_time']).abs().idxmin()
            row2 = df2.loc[idx]
            if abs(row1['ros_time'] - row2['ros_time']) < 0.1:
                vel_diff = np.sqrt(
                    (row1['lin_vel_x'] - row2['lin_vel_x'])**2 +
                    (row1['lin_vel_y'] - row2['lin_vel_y'])**2 +
                    (row1['lin_vel_z'] - row2['lin_vel_z'])**2
                )
                vel_diffs.append(vel_diff)
        
        if vel_diffs:
            axs[2, 1].plot(common_times, vel_diffs, 'c-')
            axs[2, 1].set_title('Linear Velocity Difference')
            axs[2, 1].set_xlabel('Time (s)')
            axs[2, 1].set_ylabel('Difference (m/s)')
        
        plt.tight_layout()
        plt.savefig('odometry_comparison.png')
        
        # Print summary statistics
        print("\n===== Odometry Comparison Summary =====")
        print(f"/zed_node/odom Publishing Rate: {rate1:.2f} ± {rate_std1:.2f} Hz")
        print(f"/mavros/odometry/in Publishing Rate: {rate2:.2f} ± {rate_std2:.2f} Hz")
        
        if pos_diffs:
            print(f"Average Position Difference: {np.mean(pos_diffs):.4f} ± {np.std(pos_diffs):.4f} m")
        
        if angle_diffs:
            print(f"Average Orientation Difference: {np.mean(angle_diffs):.2f} ± {np.std(angle_diffs):.2f} degrees")
        
        if time_offsets:
            print(f"Average Timestamp Difference: {np.mean(time_offsets):.2f} ± {np.std(time_offsets):.2f} ms")
        
        if vel_diffs:
            print(f"Average Velocity Difference: {np.mean(vel_diffs):.4f} ± {np.std(vel_diffs):.4f} m/s")
        
        print("=====================================")
        print(f"Results saved to odometry_comparison.png")

def main():
    rclpy.init()
    
    # Get recording duration from command line argument
    duration = 60  # Default 60 seconds
    if len(sys.argv) > 1:
        try:
            duration = int(sys.argv[1])
        except ValueError:
            print(f"Invalid duration: {sys.argv[1]}. Using default of 60 seconds.")
    
    recorder = OdometryRecorder(duration)
    rclpy.spin(recorder)

if __name__ == '__main__':
    main()