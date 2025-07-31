from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        FindPackageShare('two_dof_robot'),
        'urdf',
        'robot.urdf.xacro'
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),
       # Node(
        #    package='joint_state_publisher_gui',
         #   executable='joint_state_publisher_gui',
          #  name='joint_state_publisher_gui',
           # output='screen',
        #),

         #Node(
          #  package='two_dof_robot',
           # executable='trajectory_publisher',  # Python dosyanızın adı
            #name='trajectory_publisher',
            #output='screen',
        #),

        Node(
            package='two_dof_robot',
            executable='trajectory_to_target',  # Python dosyanızın adı
            name='trajectory_to_target',
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])
