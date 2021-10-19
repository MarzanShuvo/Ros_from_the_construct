from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='topic_subscriber_pkg', executable='simple_topic_subscriber_node', output='screen'
        ),
    ])