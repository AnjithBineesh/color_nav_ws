#!/usr/bin/env python3
"""
Autonomous Navigation Node for Differential-Drive Robot
Subscribes to /detected_color and navigates to the target pillar:
- RED   -> Red Pillar   at (2.5, 1.8)
- GREEN -> Green Pillar at (3.5, 0.0) [with detour around inspection object]
- OTHER -> White Pillar at (2.5, -1.8)
Uses closed-loop Odometry feedback and publishes velocity commands to /cmd_vel.
"""

import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, String
from color_navigation.gazebo_utils import remove_existing_models


class PillarNavigator(Node):
    # State Machine Definitions
    STATE_WAIT_COLOR = 'WAIT_FOR_COLOR_DETECTION'
    STATE_ROTATE = 'ROTATING_TO_WAYPOINT'
    STATE_DRIVE = 'DRIVING_TO_WAYPOINT'
    STATE_GOAL_REACHED = 'GOAL_REACHED'
    STATE_ALIGN_HOME = 'ALIGNING_AT_HOME'

    def __init__(self):
        super().__init__('pillar_navigator')

        # World parameter for Gazebo service calls
        self.declare_parameter('world_name', 'empty_arena')
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value

        # Current Robot Pose from Odometry
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        # Current State
        self.state = self.STATE_WAIT_COLOR
        self.detected_color = None
        self.active_target_color = None
        self.target_pillar_name = None
        self.is_returning_home = False
        self.object_cleanup_pending = False

        # Waypoint Queue
        self.waypoints = []
        self.current_waypoint_index = 0
        self.loop_count = 0

        # Controller Gains and Precision Tolerances
        self.kp_linear = 0.5
        self.kp_angular = 1.8
        self.max_linear_speed = 0.35
        self.max_angular_speed = 1.0
        self.angle_tolerance = 0.10             # ~5.7 degrees (outward navigation)
        self.robot_half_length = 0.20           # Robot chassis length 0.40 m / 2
        self.pillar_radius = 0.20               # Pillar cylinder radius in SDF
        self.safe_pillar_clearance = 0.35       # 35 cm clear air gap between bumper and pillar
        self.pillar_stop_distance = (
            self.robot_half_length + self.pillar_radius + self.safe_pillar_clearance
        )                                       # 0.75 m center-to-center stop threshold
        self.goal_stop_hold_ticks = 0
        self.intermediate_tolerance = 0.25      # meters (smooth detour corners)
        self.home_distance_tolerance = 0.04     # 4 cm precision for exact Home origin (0.00, 0.00)
        self.home_angle_tolerance = 0.03        # ~1.7 degrees precision for Home orientation (0.0 rad)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.color_sub = self.create_subscription(
            String,
            '/detected_color',
            self.color_callback,
            10
        )

        # Subscriber for Home command
        self.home_sub = self.create_subscription(
            Empty,
            '/return_home',
            self.home_callback,
            10
        )

        # Publisher to notify color detector to reset
        self.reset_detector_pub = self.create_publisher(
            Empty,
            '/reset_detector',
            10
        )

        # Path Publisher for RViz visualization
        self.path_pub = self.create_publisher(
            Path,
            '/planned_path',
            10
        )

        # Velocity Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control Loop Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Autonomous Pillar Navigator initialized.')
        self.get_logger().info('Home position set to: (X=0.0, Y=0.0, Yaw=0.0).')
        self.get_logger().info('Standing by for color detection from front camera...')

    def odom_callback(self, msg: Odometry):
        """Extract 2D position (x, y) and yaw angle from Odometry message."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Quaternion to Yaw calculation
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def color_callback(self, msg: String):
        """Handle detected color from anywhere and plan waypoints to the matching pillar."""
        color = msg.data.upper().strip()
        if color not in ('RED', 'GREEN', 'OTHER'):
            return

        webcam_active = self.count_publishers('/webcam_active') > 0
        if self.is_returning_home and not webcam_active:
            return  # Protect Gazebo onboard camera from seeing behind-pillar during return-home spin

        # Ignore duplicate confirmations for the pillar we are already navigating to or stopped at
        if (
            color == self.active_target_color
            and not self.is_returning_home
            and self.state in (self.STATE_ROTATE, self.STATE_DRIVE, self.STATE_GOAL_REACHED)
        ):
            return

        self.is_returning_home = False
        self.detected_color = color
        self.active_target_color = color
        self.goal_stop_hold_ticks = 0

        # Only expect a physical Gazebo test box in front of Home when not in webcam mode
        near_home_box = (not webcam_active and self.current_x < 1.1 and abs(self.current_y) < 0.6)

        # ========================================================
        # NAVIGATION DECISION LOGIC (Works from anywhere in arena):
        # 1. RED   -> Red Pillar   at (2.5, 1.8)
        # 2. GREEN -> Green Pillar at (3.5, 0.0) [bypass (1.0, 0.65) only if Gazebo box ahead]
        # 3. OTHER -> White Pillar at (2.5, -1.8)
        # ========================================================
        if color == 'RED':
            self.target_pillar_name = 'RED PILLAR'
            self.waypoints = [(2.5, 1.8)]
        elif color == 'GREEN':
            self.target_pillar_name = 'GREEN PILLAR'
            if near_home_box:
                self.waypoints = [(1.0, 0.65), (3.5, 0.0)]
            else:
                self.waypoints = [(3.5, 0.0)]
        else:
            self.target_pillar_name = 'WHITE PILLAR'
            self.waypoints = [(2.5, -1.8)]

        self.current_waypoint_index = 0
        self.object_cleanup_pending = near_home_box
        self.state = self.STATE_ROTATE
        self.publish_path()

        self.get_logger().info('=====================================================')
        self.get_logger().info(f'>> DETECTED COLOR: [{color}] (from X={self.current_x:.2f}, Y={self.current_y:.2f})')
        self.get_logger().info(f'>> TARGET DESTINATION: [{self.target_pillar_name}]')
        self.get_logger().info(f'>> PLANNED WAYPOINTS: {self.waypoints}')
        self.get_logger().info('=====================================================')

    def publish_path(self):
        """Publish active planned path to /planned_path for RViz visualization."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        if self.waypoints and self.odom_received:
            # Start path at current robot pose
            p0 = PoseStamped()
            p0.header = path_msg.header
            p0.pose.position.x = self.current_x
            p0.pose.position.y = self.current_y
            p0.pose.position.z = 0.05
            path_msg.poses.append(p0)

            # Append remaining waypoints
            for wx, wy in self.waypoints[self.current_waypoint_index:]:
                p = PoseStamped()
                p.header = path_msg.header
                p.pose.position.x = float(wx)
                p.pose.position.y = float(wy)
                p.pose.position.z = 0.05
                path_msg.poses.append(p)

        self.path_pub.publish(path_msg)

    def delete_spawned_objects(self, allow_fallback: bool = True):
        """Asynchronously remove existing presented test objects from Gazebo."""
        self.object_cleanup_pending = False

        def _delete_worker():
            removed, warning = remove_existing_models(
                preferred_world=self.world_name,
                fallback_remove=allow_fallback,
            )
            if warning:
                self.get_logger().warn(f'Gazebo cleanup note: {warning}')
            for obj, world in removed:
                self.get_logger().info(
                    f'✔ Automatically removed presented object [{obj}] from Gazebo world [{world}].'
                )

        thread = threading.Thread(target=_delete_worker, daemon=True)
        thread.start()

    def home_callback(self, msg: Empty):
        """Command the robot to return to exact Home position (0.00, 0.00, yaw=0.0)."""
        dist_to_home = math.sqrt(self.current_x ** 2 + self.current_y ** 2)
        if dist_to_home < self.home_distance_tolerance and abs(self.current_yaw) < self.home_angle_tolerance and not self.is_returning_home:
            self.get_logger().info('Robot is already at exact Home position.')
            self.reset_to_home_state()
            return

        self.get_logger().info('=====================================================')
        self.get_logger().info('🏠 COMMAND RECEIVED: RETURNING TO EXACT HOME (X=0.00, Y=0.00, Yaw=0.0°)...')
        if self.object_cleanup_pending:
            self.get_logger().info('🗑️ Removing test object from arena so Home path is clear...')
            self.delete_spawned_objects(allow_fallback=True)
        self.get_logger().info(f'Current Pose: X={self.current_x:.3f} m, Y={self.current_y:.3f} m, Yaw={math.degrees(self.current_yaw):.1f}°')
        self.get_logger().info('=====================================================')

        self.is_returning_home = True
        self.active_target_color = None
        self.target_pillar_name = 'EXACT HOME'

        # Plan return path:
        # If robot is in the Green corridor (x > 1.5 and abs(y) < 0.8), route around (1.2, 0.65)
        # to clear any test object sitting at (0.9, 0.0)
        if self.current_x > 1.5 and abs(self.current_y) < 0.8:
            self.waypoints = [(1.2, 0.65), (0.0, 0.0)]
        else:
            self.waypoints = [(0.0, 0.0)]

        self.current_waypoint_index = 0
        self.state = self.STATE_ROTATE
        self.publish_path()

    def reset_to_home_state(self):
        """Reset state machine and detector when docked at exact Home."""
        self.stop_robot()
        self.is_returning_home = False
        self.detected_color = None
        self.active_target_color = None
        self.target_pillar_name = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.loop_count = 0
        self.state = self.STATE_WAIT_COLOR

        if self.object_cleanup_pending:
            self.delete_spawned_objects(allow_fallback=False)

        # Trigger reset of color detector and clear visual path
        self.reset_detector_pub.publish(Empty())
        self.publish_path()

        self.get_logger().info('=====================================================')
        self.get_logger().info('🏠 ROBOT DOCKED AT EXACT HOME POSITION:')
        self.get_logger().info(f'   X   = {self.current_x:+.4f} m  (Target: 0.0000 m)')
        self.get_logger().info(f'   Y   = {self.current_y:+.4f} m  (Target: 0.0000 m)')
        self.get_logger().info(f'   Yaw = {math.degrees(self.current_yaw):+.2f}° (Target: 0.00°)')
        self.get_logger().info('Color detector reset to standby.')
        self.get_logger().info('Ready for a new object to be presented!')
        self.get_logger().info('=====================================================')

    def normalize_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def stop_robot(self):
        """Command zero velocity."""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def control_loop(self):
        """Closed-loop proportional heading and distance controller with precision Home docking."""
        self.loop_count += 1

        if not self.odom_received:
            if self.loop_count % 40 == 0:  # Every 2 seconds
                self.get_logger().info('⏳ Waiting for /odom topic from Gazebo...')
            return

        if self.state == self.STATE_WAIT_COLOR:
            if self.loop_count % 60 == 0:  # Every 3 seconds
                self.get_logger().info('📡 [STANDBY] Waiting for colored object. Present one with: ros2 run color_navigation spawn_object --color red|green|blue')
            return

        if self.state == self.STATE_GOAL_REACHED:
            if self.goal_stop_hold_ticks < 5:
                self.stop_robot()
                self.goal_stop_hold_ticks += 1
            return

        # State: Align heading to 0.0 rad (facing +X forward) at Home with anti-stall torque
        if self.state == self.STATE_ALIGN_HOME:
            home_yaw_target = 0.0
            heading_error = self.normalize_angle(home_yaw_target - self.current_yaw)

            cmd = Twist()
            if abs(heading_error) > 0.04:  # ~2.3 degrees precision
                sign = 1.0 if heading_error > 0 else -1.0
                angular_z = max(-0.6, min(0.6, self.kp_angular * heading_error))
                # Ensure minimum rotation speed so motors decisively overcome friction
                if abs(angular_z) < 0.20:
                    angular_z = sign * 0.20
                cmd.linear.x = 0.0
                cmd.angular.z = angular_z
                self.cmd_vel_pub.publish(cmd)
            else:
                self.stop_robot()
                self.reset_to_home_state()
            return

        if self.current_waypoint_index >= len(self.waypoints):
            if self.is_returning_home:
                self.state = self.STATE_ALIGN_HOME
            else:
                self.stop_robot()
                self.goal_stop_hold_ticks = 0
                self.state = self.STATE_GOAL_REACHED
                self.get_logger().info(f'🎉 SUCCESS: Reached destination [{self.target_pillar_name}]!')
                if self.object_cleanup_pending:
                    self.get_logger().info('🗑️ Disappearing test object from arena before returning home...')
                    self.delete_spawned_objects(allow_fallback=True)
            return

        # Automatically disappear the detected test object once robot clears the detection zone
        if (
            not self.is_returning_home
            and self.object_cleanup_pending
            and math.hypot(self.current_x, self.current_y) > 1.35
        ):
            self.get_logger().info('🗑️ Robot cleared presented object after detection -> removing object from Gazebo...')
            self.delete_spawned_objects(allow_fallback=True)

        target_x, target_y = self.waypoints[self.current_waypoint_index]

        # Calculate distance and desired angle to active waypoint
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        distance = math.sqrt(dx * dx + dy * dy)
        desired_yaw = math.atan2(dy, dx)
        heading_error = self.normalize_angle(desired_yaw - self.current_yaw)

        # Adaptive tolerance:
        # - 5 cm precision at exact Home origin (0.00, 0.00)
        # - 25 cm corner radius for intermediate detour waypoints
        # - 0.75 m safe stop distance in front of pillar (35 cm bumper clearance)
        is_final_waypoint = (self.current_waypoint_index == len(self.waypoints) - 1)
        if self.is_returning_home and is_final_waypoint:
            active_tolerance = 0.05
        elif not is_final_waypoint:
            active_tolerance = self.intermediate_tolerance
        else:
            active_tolerance = self.pillar_stop_distance

        cmd = Twist()

        # Check if current waypoint (or safe pillar perimeter) is reached
        if distance <= active_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(
                    f'Waypoint passed. Proceeding to waypoint {self.current_waypoint_index + 1} '
                    f'at {self.waypoints[self.current_waypoint_index]}'
                )
                self.state = self.STATE_ROTATE
                self.publish_path()
            else:
                if self.is_returning_home:
                    self.get_logger().info(
                        f'Reached Home origin: X={self.current_x:+.3f}m, Y={self.current_y:+.3f}m. '
                        f'Aligning heading to 0.00°...'
                    )
                    self.state = self.STATE_ALIGN_HOME
                else:
                    self.stop_robot()
                    self.goal_stop_hold_ticks = 0
                    self.state = self.STATE_GOAL_REACHED
                    bumper_clearance = max(
                        0.0,
                        distance - (self.robot_half_length + self.pillar_radius)
                    )
                    self.get_logger().info('=====================================================')
                    self.get_logger().info(
                        f'🎯 GOAL REACHED! Stopped safely at {self.target_pillar_name}.'
                    )
                    self.get_logger().info(
                        f'   Center Dist: {distance:.2f} m | Safe Bumper Clearance: {bumper_clearance:.2f} m'
                    )
                    if self.object_cleanup_pending:
                        self.get_logger().info('🗑️ Disappearing test object from arena before returning home...')
                        self.delete_spawned_objects(allow_fallback=True)
                    self.get_logger().info('Run [ros2 run color_navigation home] to return robot to starting position.')
                    self.get_logger().info('=====================================================')
            return

        # State 1: Rotate in-place towards waypoint until roughly aligned, with anti-stall torque
        if self.state == self.STATE_ROTATE:
            if abs(heading_error) > self.angle_tolerance:
                sign = 1.0 if heading_error > 0 else -1.0
                angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, self.kp_angular * heading_error))
                # Minimum anti-stall angular speed so wheels never freeze against friction
                if abs(angular_z) < 0.22:
                    angular_z = sign * 0.22
                cmd.linear.x = 0.0
                cmd.angular.z = angular_z
            else:
                self.state = self.STATE_DRIVE

        # State 2: Drive towards waypoint while maintaining heading
        elif self.state == self.STATE_DRIVE:
            if abs(heading_error) > 0.65:  # ~37 degrees
                self.state = self.STATE_ROTATE
                return

            if self.is_returning_home and is_final_waypoint:
                # Direct drive into exact origin (0.00, 0.00)
                linear_x = max(0.08, min(0.25, self.kp_linear * distance))
            elif not self.is_returning_home and is_final_waypoint:
                # Smoothly decelerate relative to the safe stopping perimeter so robot doesn't overshoot
                remaining_to_stop = max(0.0, distance - active_tolerance)
                linear_x = max(0.08, min(self.max_linear_speed, 0.8 * remaining_to_stop))
            else:
                linear_x = max(0.08, min(self.max_linear_speed, self.kp_linear * distance))

            angular_z = self.kp_angular * heading_error
            angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))

            cmd.linear.x = linear_x
            cmd.angular.z = angular_z

        self.cmd_vel_pub.publish(cmd)

        if self.loop_count % 20 == 0:  # Every 1 second
            self.get_logger().info(
                f'🚗 [{self.state}] -> Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} '
                f'| Dist: {distance:.2f}m | YawErr: {math.degrees(heading_error):+.1f}° '
                f'| Cmd: v={cmd.linear.x:.2f}m/s, w={cmd.angular.z:+.2f}rad/s'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PillarNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

