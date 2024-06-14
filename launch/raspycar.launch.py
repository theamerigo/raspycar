from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raspycar_pkg',
            executable='encoderL',
        ),
        Node(
            package='raspycar_pkg',
            executable='encoderR',
        ),
        Node(
            package='raspycar_pkg',
            executable='pidL',
        ),
        Node(
            package='raspycar_pkg',
            executable='pidR',
        ),
        Node(
            package='raspycar_pkg',
            executable='motorL',
        ),
        Node(
            package='raspycar_pkg',
            executable='motorR',
        ),
        Node(
            package='raspycar_pkg',
            executable='odometry',
            parameters=[
                {'theta0': -3.14159}
            ]
        ),
        Node(
            package='raspycar_pkg',
            executable='path_planning',
            parameters=[
                {'p0': [0, 0, -3.14159],
                 'pF': [1.5, 0, 0]}
            ]
        )
    ])

