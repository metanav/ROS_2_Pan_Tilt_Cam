from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    resource_path = Path(get_package_share_directory('hand_tracker'), 'resource')

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
    )

    camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [320, 240],
                'frame_rate': 30.0,
                'auto_exposure': 3,            # 1 = Manual mode (NOT 0!)
                #'exposure_dynamic_framerate': True,  # Allow FPS to stabilize
                'brightness': 15,                 #50  0–255, 50 = neutral
                'contrast': 15,                   #32 0–64, 32 = default, natural
                'saturation': 70,                 # 0–128, 70 = vivid but not oversaturated
                'hue': 0,
                'gain': 0,
                'gamma': 100,                     # 100–500, 100 = linear (natural)
                'sharpness': 0,                   # 0 = soft, natural (no edge artifacts)
                'backlight_compensation': 1,      # 0 = off (prevents over-bright highlights)
                'auto_white_balance': True,       # Let camera adapt
                'white_balance_temperature': 3500,#4600, # Lock fallback (daylight)
                'power_line_frequency': 1,        # 1 = 50 Hz (EU/JP), 2 = 60 Hz (US)

            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ]
        )

    object_detection_node = Node(
            package = 'hand_tracker',
            namespace = '',
            executable = 'object_detection_node',
            output = 'screen',
            parameters = [ {'resource_path' : str(resource_path)} ]
    )

    img_bbox_node = Node(
            package = 'hand_tracker',
            namespace = '',
            executable = 'image_bbox_node',
            output = 'screen',
    )

    return LaunchDescription([
        camera_name_arg,
        camera_node,
        object_detection_node,
        img_bbox_node
    ])
