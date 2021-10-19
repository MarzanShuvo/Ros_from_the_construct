from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='odom_subscriber_pkg', executable='odom_subscriber_node', output='screen'
        ),
    ])