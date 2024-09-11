from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ads1x15_ros",
                executable="ads1x15_node",
                name="ads1x15_node",
                parameters=[
                    {"pub_rate": 100},
                ],
            )
        ]
    )
