from os.path import join
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    launcher_description_list = []

    launch_file_infos = [
        ('cone_detection', 'launch/container.launch.py'),
        ('cone_detection', 'launch/log_img_pub.launch.py'),
        ('cone_detection', 'launch/log_pcd_pub.launch.py'),
        ('cone_detection', 'launch/main_cone_detection.launch.py'),

    ]

    # 各 launch ファイルを IncludeLaunchDescription で追加
    for launch_file_info in launch_file_infos:
        pkg_prefix = get_package_share_directory(launch_file_info[0])
        path = join(pkg_prefix, launch_file_info[1])
        launcher = IncludeLaunchDescription(PythonLaunchDescriptionSource(path))
        launcher_description_list.append(launcher)
    return LaunchDescription(launcher_description_list)