from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('hand_tracker'),
                'launch', 'object_detection_node.launch.py'
            )
        )
    )

    hand_tracker_node = Node(
        package='hand_tracker',
        executable='hand_tracker_node',
        name='hand_tracker',
        output='screen',
    )
    
    return LaunchDescription([
        object_detection_launch,
        hand_tracker_node
    ])
