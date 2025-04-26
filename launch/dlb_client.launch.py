import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='deploadblade',
            executable='teleop_client',
            name='teleop_client',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='deploadblade',
            executable='depth_publisher_e',
            name='depth_publisher_e',
            output='screen',
            parameters=[
                {'depth_module.profile': '640x480x30'}
            ]
        )
    ])