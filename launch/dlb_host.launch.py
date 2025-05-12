from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # C++ Node: teleop_host
        Node(
            package='deploadblade',
            executable='teleop_host',
            name='teleop_host',
            output='screen'
        ),

        # C++ Node: depthtoimg
        Node(
            package='deploadblade',
            executable='depth_to_img',
            name='depth_to_img',
            output='screen'
        ),

        # Python Node: robotpose_pub.py
        Node(
            package='deploadblade',
            executable='robotpose_pub.py',
            name='robot_pose_publisher',
            output='screen'
        ),

        # Launch rqt for image viewing
        ExecuteProcess(
            cmd=['rqt'],
            output='screen'
        ),
    ])
