from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

def generate_launch_description():
    pkg_prefix = get_package_share_directory('cone_detection')
    crosswalk_signal = LoadComposableNodes(
        target_container='rs_container',
        composable_node_descriptions=[
            ComposableNode(
                package='cone_detection',
                plugin='cone_detector::Recognition',
                name='main_cone_detection',
                remappings=[
                    ('/camera1/image', '/camera1/image'),
                    # ('/camera1/image', '/camera1/image_test'),
                    ('/lidar/points', '/pandar40/points'), 
                    # ('/lidar/points', '/pandar40/points_test'), 
                    ('/light_msg', '/light_msg'),
                    ('/signal_image', '/signal_image')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )
    return LaunchDescription([crosswalk_signal])