from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="two_dof_robot",
            executable="position_publisher",
            name="two_dof_position_publisher",
            parameters=[PathJoinSubstitution([
                FindPackageShare("two_dof_robot"),
                "config",
                "position_controller.yaml"
            ])],
            output="screen"
        )
    ])
