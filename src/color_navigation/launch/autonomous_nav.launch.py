import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_robot_description = get_package_share_directory('robot_description')
    pkg_color_nav = get_package_share_directory('color_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    color = context.perform_substitution(LaunchConfiguration('color')).lower().strip()
    manual = context.perform_substitution(LaunchConfiguration('manual')).lower().strip()
    webcam = context.perform_substitution(LaunchConfiguration('webcam')).lower().strip()
    webcam_device = int(context.perform_substitution(LaunchConfiguration('webcam_device')))
    rviz = LaunchConfiguration('rviz')
    headless = context.perform_substitution(LaunchConfiguration('headless')).lower().strip()

    use_webcam = webcam in ['true', '1']

    # If manual or webcam mode is active, start in an empty arena without pre-spawned objects
    if manual in ['true', '1'] or use_webcam:
        world_filename = 'empty_arena.sdf'
        world_name = 'empty_arena'
    elif 'green' in color:
        world_filename = 'world_green.sdf'
        world_name = 'three_pillars_world_green'
    elif 'other' in color or 'blue' in color or 'yellow' in color or 'white' in color:
        world_filename = 'world_other.sdf'
        world_name = 'three_pillars_world_other'
    else:
        world_filename = 'world_red.sdf'
        world_name = 'three_pillars_world_red'

    world_path = os.path.join(pkg_color_nav, 'worlds', world_filename)
    xacro_file = os.path.join(pkg_robot_description, 'urdf', 'differential_robot.urdf.xacro')
    default_rviz_config = os.path.join(pkg_robot_description, 'config', 'view_robot.rviz')

    gz_flags = '-r -s ' if headless in ['true', '1'] else '-r '

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
        launch_arguments={'gz_args': [gz_flags, world_path]}.items()
    )

    # 3. Spawn Robot
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

    # 4. ROS-Gazebo Parameter Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    # 5. OpenCV Color Detector Node (Laptop Webcam or Gazebo Camera)
    if use_webcam:
        detector_node = Node(
            package='color_navigation',
            executable='webcam_detector',
            name='webcam_color_detector',
            output='screen',
            parameters=[{
                'device_id': webcam_device,
                'debug_topic': '/camera/debug_image',
                'show_window': True
            }]
        )
    else:
        detector_node = Node(
            package='color_navigation',
            executable='color_detector',
            name='color_detector',
            output='screen'
        )

    # 6. Autonomous Pillar Navigator Node
    navigator_node = Node(
        package='color_navigation',
        executable='navigator',
        name='navigator',
        output='screen',
        parameters=[{
            'world_name': world_name
        }]
    )

    # 7. RViz2 (Optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    return [
        robot_state_publisher_node,
        gz_sim,
        spawn_robot,
        bridge,
        detector_node,
        navigator_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'color',
            default_value='red',
            description='Auto-spawned object color: red | green | other'
        ),
        DeclareLaunchArgument(
            'manual',
            default_value='false',
            description='If true, start empty world so you can present objects manually'
        ),
        DeclareLaunchArgument(
            'webcam',
            default_value='false',
            description='If true, detect real-world objects using your laptop webcam'
        ),
        DeclareLaunchArgument(
            'webcam_device',
            default_value='0',
            description='Camera device index for laptop webcam (e.g. 0, 1)'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz2 alongside simulation if true'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in server-only (headless) mode if true'
        ),
        OpaqueFunction(function=launch_setup)
    ])
