import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'differential_robot.urdf.xacro')
    default_world = os.path.join(pkg_robot_description, 'worlds', 'color_world.sdf')
    default_rviz_config = os.path.join(pkg_robot_description, 'config', 'view_robot.rviz')

    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')

    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 2. Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # 3. Spawn Robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'colour_detection_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ]
    )

    # 4. ROS-GZ Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            # cmd_vel (ROS -> GZ)
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            # Odometry (GZ -> ROS)
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            # Joint States (GZ -> ROS)
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            # TF (GZ -> ROS)
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            # Camera Image & Info (GZ -> ROS)
            '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 5. Optional RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Path to the SDF world file'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 alongside Gazebo if true'
        ),
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        bridge,
        rviz_node
    ])

