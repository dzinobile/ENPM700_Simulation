from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Resolve foxglove_bridge share and prefer the launch/ subfolder where
    # foxglove_bridge_launch.xml is usually installed.
    foxglove_share = get_package_share_directory('foxglove_bridge')
    # common install layout: <share>/foxglove_bridge/launch/...
    candidate_paths = [
        os.path.join(foxglove_share, 'launch', 'foxglove_bridge_launch.xml'),
        os.path.join(foxglove_share, 'foxglove_bridge_launch.xml'),
    ]

    foxglove_launch_path = None
    for p in candidate_paths:
        if os.path.exists(p):
            foxglove_launch_path = p
            break

    if foxglove_launch_path is None:
        raise FileNotFoundError(f"foxglove_bridge launch file not found in: {candidate_paths}")

    foxglove_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(foxglove_launch_path),
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
