#!/usr/bin/env python3
"""
CLI Command to send the robot back to Home position (0.0, 0.0, yaw=0.0).
Usage:
  ros2 run color_navigation return_home
  ros2 run color_navigation home
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from color_navigation.gazebo_utils import remove_existing_models


def main():
    # Remove any remaining test object only if it currently exists in Gazebo
    removed, _ = remove_existing_models()
    for obj, world in removed:
        print(f"✔ Removed test object [{obj}] from Gazebo world '{world}'.")

    rclpy.init()
    node = Node('return_home_cli')
    pub = node.create_publisher(Empty, '/return_home', 10)

    # Allow publisher connection to establish
    time.sleep(0.3)

    msg = Empty()
    pub.publish(msg)

    print("=====================================================")
    print("🏠 [HOME COMMAND DELIVERED]")
    print("Commanding robot to return to Home: (X=0.0, Y=0.0, Yaw=0.0)")
    print("The robot will:")
    print("  1. Rotate and drive back to (0.0, 0.0)")
    print("  2. Re-align its heading to 0.0 rad (facing +X forward)")
    print("  3. Reset the color detector to standby")
    print("  4. Wait for the next object to be presented!")
    print("=====================================================")

    # Spin briefly to guarantee delivery
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

