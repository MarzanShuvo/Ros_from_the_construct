"""Launch square_service_client"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='bb8_cpp_unit_3_services', node_executable='square_service_client_node', output='screen'),
    ])