"""Launch cpp_simple_service_client_node_ex7_1"""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='cpp_unit_7_services', executable='cpp_simple_service_client_ex7_1_node', output='screen'),
    ])