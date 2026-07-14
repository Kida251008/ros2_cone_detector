from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer


def generate_launch_description():

    container = ComposableNodeContainer(
        name='rs_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen'
    )

    return LaunchDescription([
        container
    ])