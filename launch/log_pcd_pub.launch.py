from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    # パッケージディレクトリのパスを取得
    pkg_prefix = get_package_share_directory('cone_detection')
    
    # log_img_pubのコンポーネントを定義
    log_img_pub = LoadComposableNodes(
        target_container='rs_container',
        composable_node_descriptions=[
            ComposableNode(
                package='cone_detection',
                plugin='cone_detector::LogPcdPublisher',
                name='log_pcd_pub',  # ノード名
                parameters=[join(pkg_prefix, 'cfg/log_pcd_pub.yaml')],
                remappings=[
                    # ('/camera1/image', '/camera1/image'),
                    ('/lidar/points', '/pandar40/points'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    return LaunchDescription([log_img_pub])