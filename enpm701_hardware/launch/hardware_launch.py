from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'foxglove_bridge_launch.xml'
            )
        ),
        launch_arguments={'port': '8765'}.items(),
    )

    v4l2_camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        parameters=[{'image_size': [640, 480]}],
    )

    return LaunchDescription([
        foxglove_launch,
        v4l2_camera,
    ])
