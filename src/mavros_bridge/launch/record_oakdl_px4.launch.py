import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

px4_config_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_config.yaml'
)
px4_pluginlists_path = os.path.join(
    get_package_share_directory('mavros_bridge'),
    'config', 'px4_pluginlists.yaml'
)

def generate_launch_description():

    # Mavros Node
    mavros_node = Node(
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
        # remappings=[
        #     ("/mavros/imu/data_raw", "/imu0"),
        # ]
    )

    oak_component = ComposableNode(
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
    )

    # Define the container to run the node
    oak_container = ComposableNodeContainer(
        name='oak_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container', # component_container_isolated
        arguments=['--ros-args', '--log-level', 'info'],
        output='screen',
        composable_node_descriptions=[oak_component]  # Fix: Add node to container
    )

    bag_record = ExecuteProcess(
    cmd=['ros2', 'bag', 'record', '--output', '/home/nvidia/ros2-ws/dataset/oakd_px4',
            '/left/image_raw',
            '/right/image_raw',
            '/mavros/imu/data'],
    output='screen')

    ld = LaunchDescription()
    ld.add_action(mavros_node)
    ld.add_action(oak_container)
    ld.add_action(bag_record)

    return ld