import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

DEBUG = os.environ.get('DEBUG', 'false').lower() == 'true' or os.environ.get('DEBUG', 'false').lower() == '1'


def generate_launch_description():
    # Define the package name
    pkg_name = 'apriltag_ros' # <--- CHANGE THIS to the name of your ROS package

    # Construct the path to the config file
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'cfg',
        'tags.yaml'
    )
    log_level = 'debug' if DEBUG else 'info'

    # Define the node
    apriltag_node = Node(
        package=pkg_name,
        executable='apriltag_ros_node', # This should match the executable name in your CMakeLists.txt
        name='apriltag_node',
        output='screen',
        parameters=[config_file, {'size': 0.12}],
        remappings=[
            ('image', '/zed/zed_node/left/color/raw/image'),
            ('camera_info', '/zed/zed_node/left/color/raw/image/camera_info'),
             ('detections', '/apriltag_node/detections'),
            # ('debug_image', '/my_debug_image')
        ],
        arguments=['--ros-args', '--log-level', 'debug' if DEBUG else 'info']
    )

    return LaunchDescription([
        apriltag_node
    ])
