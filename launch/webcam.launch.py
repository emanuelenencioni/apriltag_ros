import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the package name
    pkg_name = 'apriltag_ros' # <--- CHANGE THIS to the name of your ROS package

    # Construct the path to the config file
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'tags_36h11.yaml'
    )

    # Define the node
    apriltag_node = Node(
        package=pkg_name,
        executable='apriltag_ros_node', # This should match the executable name in your CMakeLists.txt
        name='apriltag_node',           # You can change the node name here
        output='screen',
        parameters=[config_file],
        remappings=[
            # Remap the generic topics to your specific ZED camera topics
            ('image', '/image_raw'),
            ('camera_info', '/camera_info'),

            # You can also remap the output topics if needed
            # ('detections', '/my_detections'),
            # ('debug_image', '/my_debug_image')
        ]
    )

    return LaunchDescription([
        apriltag_node
    ])
