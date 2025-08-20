from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name='panda',
                             package_name='moveit_resources_panda_moveit_config')
        .to_moveit_configs()
    )
    params = moveit_config.to_dict()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': params['robot_description']}],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','world','panda_link0'],
        output='screen',
    )

    controllers_yaml = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'),
        'config', 'ros2_controllers.yaml'
    )
    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml, {'robot_description': params['robot_description']}],
        output='screen',
    )
    spawn_jsb  = Node(package='controller_manager', executable='spawner',
                      arguments=['joint_state_broadcaster'], output='screen')
    spawn_arm  = Node(package='controller_manager', executable='spawner',
                      arguments=['panda_arm_controller'], output='screen')
    spawn_hand = Node(package='controller_manager', executable='spawner',
                      arguments=['panda_hand_controller'], output='screen')

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[params],
        output='screen',
    )

    # planning_scene_relay (isteğe bağlı)
    relay = Node(
        package='moveit',
        executable='planning_scene_relay',
        output='screen',
    )

    server = Node(
        package='moveit',
        executable='custom_server',
        parameters=[params],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[params],
    )

    # YENİ: client ve GUI
    client = Node(
        package='moveit',
        executable='custom_client',
        output='screen',
    )

    gui = Node(
        package='moveit',
        executable='vector_input_gui',
        output='screen',
    )

    

    return LaunchDescription([
        static_tf,
        rsp,
        ros2_control, spawn_jsb, spawn_arm, spawn_hand,
        move_group,
        relay,
        server,
        client,   # <— client eklendi
        gui,      # <— GUI eklendi
        rviz,
    ])