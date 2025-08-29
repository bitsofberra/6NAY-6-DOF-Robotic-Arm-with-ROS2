from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Arm commander node
        Node(
            package='panda_yolo_pickplace',
            executable='panda_arm_commander',
            name='panda_arm_commander',
            output='screen',
            parameters=[{'use_sim_time': True},
                        # paket içi yaml:
                        '$(find-pkg-share panda_yolo_pickplace)/config/pick_place_positions.yaml']
        ),
    ])
