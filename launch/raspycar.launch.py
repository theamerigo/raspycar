from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='raspycar_pkg',
            executable='reference',
        ),
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
        )
    ])

