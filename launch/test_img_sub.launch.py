from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    pkg_prefix = get_package_share_directory('cone_detection')
    test_img_sub = LoadComposableNodes(
        target_container='rs_container',  # コンテナ名はcontainer.launch.pyで指定
        composable_node_descriptions=[
            ComposableNode(
                package='cone_detection',
                plugin='cone_detector::ImageSubscriber',
                name='test_img_sub',
                remappings=[
                    # ('/signal_image', '/signal_image'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    return LaunchDescription([test_img_sub])