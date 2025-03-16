import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    vins_package = get_package_share_directory('vins')

    # Define VINS config file path
    vins_config = os.path.join(vins_package, 'config', 'oakdlite_px4', 'oakdlite_px4.yaml')

    # Define RViz config file path
    rviz_config = os.path.join(vins_package, 'config', 'vins_rviz_config.rviz')

    return LaunchDescription([
        # Launch VINS node
        Node(
            package="vins",
            executable="vins_node",
            name="vins",
            namespace="",
            parameters=[
                {"config_file": vins_config},
                {"stereo_sync_tolerance": 0.01}
            ],
            output="screen"
        ),
        
        # Launch RViz2 with the specified config
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen"
        )
    ])
