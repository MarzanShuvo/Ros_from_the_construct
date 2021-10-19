from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='topic_mini_project_pkg', executable='mini_project_node', output='screen'
        ),
    ])