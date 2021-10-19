from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='exercise_4_2_robot_move', executable='move_robot_node', output='screen'
        ),
    ])