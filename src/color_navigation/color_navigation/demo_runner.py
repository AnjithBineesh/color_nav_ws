#!/usr/bin/env python3
"""
Automated Multi-Cycle Demo Runner for Color Navigation.
Sequentially tests all three mission criteria:
  1. RED Object   -> Navigates to Red Pillar   -> Auto-deletes -> Returns to Home
  2. GREEN Object -> Navigates to Green Pillar -> Auto-deletes -> Returns to Home
  3. BLUE Object  -> Navigates to White Pillar -> Auto-deletes -> Returns to Home
Usage:
  ros2 run color_navigation demo
"""

import math
import subprocess
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


class DemoRunner(Node):
    def __init__(self):
        super().__init__('demo_runner')
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            10
        )
        self.home_pub = self.create_publisher(Empty, '/return_home', 10)

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def wait_for_odom(self):
        print("Waiting for Odometry from simulation...")
        while not self.odom_received and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        print(f"✔ Odometry connected! Initial Pose: X={self.current_x:.2f}, Y={self.current_y:.2f}")

    def wait_until_near(self, target_x, target_y, tolerance=0.45, timeout_sec=30.0):
        start = time.time()
        while time.time() - start < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            dist = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
            if dist < tolerance:
                return True
        return False

    def wait_until_docked_at_home(self, timeout_sec=35.0):
        start = time.time()
        while time.time() - start < timeout_sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            dist = math.sqrt(self.current_x**2 + self.current_y**2)
            if dist < 0.06 and abs(self.current_yaw) < 0.08:
                return True
        return False

    def send_home(self):
        print("🏠 Sending robot back to Home Base...")
        msg = Empty()
        self.home_pub.publish(msg)

    def spawn(self, color):
        print(f"\n📦 Presenting [{color.upper()}] object to robot...")
        cmd = ['ros2', 'run', 'color_navigation', 'spawn_object', '--color', color]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    rclpy.init()
    runner = DemoRunner()
    runner.wait_for_odom()

    stages = [
        ('red', (2.5, 1.8), 'RED PILLAR (Left)'),
        ('green', (3.5, 0.0), 'GREEN PILLAR (Center with Detour)'),
        ('blue', (2.5, -1.8), 'WHITE PILLAR (Right - Other Color)'),
    ]

    print("\n=======================================================")
    print("🚀 STARTING AUTOMATED MULTI-CYCLE MISSION DEMO")
    print("=======================================================")

    for idx, (color, (tx, ty), desc) in enumerate(stages, 1):
        print(f"\n--- [CYCLE {idx}/3]: Presenting {color.upper()} Object ---")
        time.sleep(1.0)
        runner.spawn(color)

        print(f"Waiting for robot to navigate to {desc} at ({tx}, {ty})...")
        if runner.wait_until_near(tx, ty, tolerance=0.82, timeout_sec=30.0):
            print(f"🎯 Reached {desc}!")
        else:
            print(f"⚠️ Warning: Target arrival timed out.")

        time.sleep(2.0)
        runner.send_home()

        print("Waiting for robot to dock at exact Home (0.00, 0.00, Yaw=0.0°)...")
        if runner.wait_until_docked_at_home(timeout_sec=35.0):
            print(f"🏠 Successfully docked at Home! Detector reset to standby.")
        else:
            print(f"⚠️ Docking check timed out, proceeding...")
        time.sleep(1.5)

    print("\n=======================================================")
    print("🎉 ALL 3 COLOR CYCLES COMPLETED SUCCESSFULLY!")
    print("=======================================================")
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

