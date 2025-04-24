import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='simple_robot_pkg',
            executable='simple_robot_mover',
            name='simple_robot_mover',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='simple_robot_pkg',
            executable='simple_robot_recorder',
            name='simple_robot_recorder',
            output='screen'
        )
    ])
