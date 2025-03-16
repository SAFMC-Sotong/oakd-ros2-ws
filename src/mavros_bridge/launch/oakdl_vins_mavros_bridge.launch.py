import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)

from launch.substitutions import (
    Command,
)
from launch_ros.actions import (
    Node,
    ComposableNodeContainer,
)
from launch_ros.descriptions import ComposableNode

vins_package = get_package_share_directory('vins')
mavros_package = get_package_share_directory('mavros_bridge')
robot_viewer_package = get_package_share_directory('uosm_robot_viewer')
xacro_path = os.path.join(robot_viewer_package, 'urdf', 'dogpa_oak.urdf.xacro')

vins_config = os.path.join(
    vins_package,
    'config',
    'oakdlite_px4',
    'oakdlite_px4.yaml'
)

px4_config_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_config.yaml'
)
px4_pluginlists_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_pluginlists.yaml'
)

def generate_launch_description():
    # Declare arguments
    launch_foxglove_arg = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='false',
        description='Whether to launch Foxglove Bridge',
        choices=['true', 'false']
    )

    # Create the composable node container
    container = ComposableNodeContainer(
        name='mavros_bridge_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container', # component_container_isolated
        arguments=['--ros-args', '--log-level', 'info'],
        composable_node_descriptions=[
            ComposableNode(
                package='oakd_ros2_minimal',
                namespace='',
                plugin='uosm::depthai::OakCamera',
                name='oak_node',
                parameters=[{
                        'useRaw': True,
                        'monoResolution': '400p',
                        'colorResolution': '1080p',
                        'previewWidth': 300,
                        'previewHeight': 300,
                        'colorFPS': 15,
                        'lrFPS': 30,
                        'syncThreshold': 10,
                        'convInterleaved': False,
                        'convGetBaseDeviceTimestamp': False,
                        'convUpdateROSBaseTimeOnToRosMsg': True,
                        'convReverseSocketOrder': True
                    }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='mavros_bridge',
                plugin='uosm::mavros::MavrosBridgeComponent',
                name='mavros_bridge',
                parameters=[
                    {"publish_rate_hz" : 15.0},
                    {"buffer_max_size_" : 20},
                    {"odom_topic" : "/odometry"},
                    {"map_frame" : "odom"},
                    {"base_frame" : "base_link"}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Robot State Publisher Component
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command(['xacro ', xacro_path]),
                    'package_path': mavros_package
                }]
            ),
        ],
        output='screen',
    )

    # Mavros Node
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        parameters=[
            px4_pluginlists_path,
            px4_config_path,
        {    
            # 'fcu_url': '/dev/ttyTHS1:921600',
            'fcu_url': '/dev/ttyACM0:2000000',
            'gcs_url': 'udp://@10.42.0.1',
            #'gcs_url': 'udp://@10.100.18.240',
            'tgt_system': 1,
            'tgt_component': 1,
            'fcu_protocol': "v2.0",
            'respawn_mavros': "false",
            'namespace': "mavros",
        }],
        output="screen"
    )

    if not os.path.exists(vins_config):
        raise FileNotFoundError(f"Config file not found: {vins_config}")
    
    vins = Node(
        package="vins",
        executable="vins_node",
        name="vins",
        namespace="",
        parameters=[
            {"config_file": vins_config}
        ],
        output="screen"
    )

    # Joint State Publisher
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen"
    )

    map_odom_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_pub_map_odom_aligned",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen"
    )

    return LaunchDescription([
        # Arguments
        launch_foxglove_arg,

        # Nodes and containers
        container,
        vins,
        mavros,
        map_odom_tf_publisher,
        jsp,
    ])