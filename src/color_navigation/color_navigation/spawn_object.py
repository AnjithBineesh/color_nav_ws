#!/usr/bin/env python3
"""
Dynamic Object Spawner CLI for Gazebo Simulation
Allows manually presenting and removing colored objects in front of the robot.
Usage:
  ros2 run color_navigation spawn_object --color red
  ros2 run color_navigation spawn_object --color green
  ros2 run color_navigation spawn_object --color blue
  ros2 run color_navigation spawn_object --delete
"""

import argparse
import os
import subprocess
import sys
from ament_index_python.packages import get_package_share_directory
from color_navigation.gazebo_utils import remove_existing_models


def main():
    parser = argparse.ArgumentParser(
        description='Manually present or remove colored objects in front of the robot in Gazebo'
    )
    parser.add_argument(
        '--color',
        choices=['red', 'green', 'blue', 'yellow'],
        default='red',
        help='Color of object to present: red, green, blue, yellow'
    )
    parser.add_argument(
        '--x',
        type=float,
        default=0.9,
        help='X position in front of robot (default: 0.9 m)'
    )
    parser.add_argument(
        '--y',
        type=float,
        default=0.0,
        help='Y position in front of robot (default: 0.0 m)'
    )
    parser.add_argument(
        '--z',
        type=float,
        default=0.175,
        help='Z elevation (default: 0.175 m)'
    )
    parser.add_argument(
        '--name',
        default='presented_object',
        help='Entity name in Gazebo'
    )
    parser.add_argument(
        '--delete',
        action='store_true',
        help='Remove the presented object from the world'
    )
    parser.add_argument(
        '--home',
        action='store_true',
        help='Command robot to return to Home position (0, 0)'
    )

    args = parser.parse_args()

    if args.home:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Empty
        import time

        # Remove existing object only if present in Gazebo
        removed, _ = remove_existing_models()
        for obj, world in removed:
            print(f"✔ Removed test object [{obj}] from Gazebo world '{world}'.")

        rclpy.init()
        node = Node('spawn_home_trigger')
        pub = node.create_publisher(Empty, '/return_home', 10)
        time.sleep(0.3)
        pub.publish(Empty())
        print("🏠 Sent command to return robot to Home position (0.0, 0.0, yaw=0.0).")
        rclpy.spin_once(node, timeout_sec=0.5)
        node.destroy_node()
        rclpy.shutdown()
        return

    if args.delete:
        print(f"Checking for object [{args.name}] in Gazebo...")
        removed, warning = remove_existing_models(target_names=[args.name])
        if warning:
            print(f"Warning: {warning}", file=sys.stderr)
        if removed:
            for obj, world in removed:
                print(f"✔ Successfully deleted [{obj}] from world '{world}'.")
        else:
            print(f"Notice: Object [{args.name}] was not active or already deleted.")
        return

    # Only delete existing object if it actually exists (avoids UserCommands error spam)
    remove_existing_models(target_names=[args.name])

    pkg_dir = get_package_share_directory('color_navigation')
    model_filename = f'{args.color}_box.sdf'
    model_path = os.path.join(pkg_dir, 'models', model_filename)

    if not os.path.exists(model_path):
        print(f"Error: Model file '{model_path}' not found.", file=sys.stderr)
        sys.exit(1)

    print("=====================================================")
    print(f"Presenting [{args.color.upper()}] object to the robot...")
    print(f"Placement Position: (X={args.x}, Y={args.y}, Z={args.z})")
    print("=====================================================")

    spawn_cmd = [
        'ros2', 'run', 'ros_gz_sim', 'create',
        '-name', args.name,
        '-file', model_path,
        '-x', str(args.x),
        '-y', str(args.y),
        '-z', str(args.z)
    ]

    result = subprocess.run(spawn_cmd)
    if result.returncode == 0:
        print(f"✔ [{args.color.upper()}] object successfully placed in front of robot camera!")
    else:
        print(f"Error spawning object (exit code {result.returncode})", file=sys.stderr)


if __name__ == '__main__':
    main()

