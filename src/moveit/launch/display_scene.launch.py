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

    relay = Node(
        package='moveit',
        executable='planning_scene_relay',
        output='screen',
    )

    server = Node(
        package='moveit',
        executable='custom_server',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            { 'world_frame': 'panda_link0',
              'spawn_table': True,
              'spawn_box':   True,
              'acm_allow_box_and_table': False }
        ],
    )

    # (Opsiyonel) eski client kalabilir
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

    free_gui = Node(
    package='moveit',              # kendi paket adın
    executable='free_move_gui',
    output='screen',
)

    # Yeni pick_place_node – GUI'den /pick_place_cmd dinler
    pick_place = Node(
        package='moveit',                # paket adınızı yazın
        executable='pick_place_node',    # bu dosyadan üretilen ikili
        name='pick_place_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'world_frame':    'panda_link0',
                'planning_group': 'panda_arm',
                'eef_link':       'panda_link8',

                # hız/ivme (OMPL ölçekleri)
                'vel_scale': 0.35,
                'acc_scale': 0.35,

                # Kartezyen iniş/çıkış
                'eef_step':      0.004,
                'cart_dt':       0.02,   # ~ uç-efektör hızı ≈ eef_step/cart_dt
                'cart_min_frac': 0.45,
                'cart_segments': 6,

                # yaklaşma/kaçış
                'approach_z':     0.24,
                'retreat_z':      0.20,
                'pick_clearance': 0.08,

                # sahne/temas
                'ensure_box':        True,
                'allow_touch':       True,
                'attach_after_pick': True
            }
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[params],
    )

    return LaunchDescription([
        static_tf,
        rsp,
        ros2_control, spawn_jsb, spawn_arm, spawn_hand,
        move_group,
        relay,
        server,
        client,
        gui,
        free_gui,
        pick_place,   # move_group’tan sonra
        rviz,
    ])
