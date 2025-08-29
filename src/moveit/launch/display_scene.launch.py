# display_scene.launch.py

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # --- MoveIt configlerini yükle (URDF, SRDF, OMPL, controller mapping, vb.)
    moveit_config = (
        MoveItConfigsBuilder(robot_name='panda',
                             package_name='moveit_resources_panda_moveit_config')
        .to_moveit_configs()
    )
    params = moveit_config.to_dict()

    # --- Dosyalar
    controllers_yaml = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'),
        'config', 'ros2_controllers.yaml'
    )
    rviz_cfg = os.path.join(
        get_package_share_directory('moveit_resources_panda_moveit_config'),
        'config', 'moveit.rviz'
    )

    # --- TF ve robot_state_publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0'],
        output='screen',
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': params['robot_description']}],
        output='screen',
    )

    # --- ros2_control + spawner'lar
    ros2_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml, params],   # robot_description burada da mevcut
        output='screen',
    )

    spawn_jsb = Node(package='controller_manager', executable='spawner',
                     arguments=['joint_state_broadcaster'], output='screen')

    spawn_arm = Node(package='controller_manager', executable='spawner',
                     arguments=['panda_arm_controller'], output='screen')

    spawn_hand = Node(package='controller_manager', executable='spawner',
                      arguments=['panda_hand_controller'], output='screen')

    # --- move_group
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[params],
        output='screen',
    )

    # --- (opsiyonel) planning scene relay (senin paketinde ise)
    relay = Node(
        package='moveit',
        executable='planning_scene_relay',
        output='screen',
    )

    # --- Senin düğümler
    server = Node(
        package='moveit',
        executable='custom_server',
        output='screen',
        parameters=[
            params,  # MoveIt paramları
            {
                'world_frame': 'panda_link0',
                'spawn_table': True,
                'spawn_box': True,
                'acm_allow_box_and_table': False,
            },
        ],
    )

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
        package='moveit',
        executable='free_move_gui',
        output='screen',
    )

    pick_place = Node(
        package='moveit',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[
            params,
            {
                'world_frame': 'panda_link0',
                'planning_group': 'panda_arm',
                'eef_link': 'panda_link8',
                'vel_scale': 0.35,
                'acc_scale': 0.35,
                'eef_step': 0.004,
                'cart_dt': 0.02,
                'cart_min_frac': 0.45,
                'cart_segments': 6,
                'approach_z': 0.24,
                'retreat_z': 0.20,
                'pick_clearance': 0.08,
                'ensure_box': True,
                'allow_touch': True,
                'attach_after_pick': True,
            },
        ],
    )

    # --- RViz: MoveIt paramları + topic remap (TL'den oku)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[params],
        # RViz içindeki pluginler /monitored_planning_scene’i dinler;
        # bunu TL versiyonuna remap ediyoruz ki anında son sahneyi alsın.
        remappings=[('/monitored_planning_scene', '/monitored_planning_scene_tl')],
    )

    # --- Başlatma sırası (yarış durumlarını engelle)
    ld = LaunchDescription([
        static_tf,
        rsp,
        ros2_control,
        spawn_jsb,
        spawn_arm,
        spawn_hand,

        # Spawner'lar bittiğinde move_group'u başlat
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_arm,
                on_exit=[move_group],
            )
        ),

        # move_group başladıktan KISA BİR SÜRE sonra diğer düğümleri başlat
        # (controller'lar ve monitored scene hazır olsun diye)
        RegisterEventHandler(
            OnProcessExit(
                target_action=move_group,
                on_exit=[
                    relay,
                    TimerAction(period=0.8, actions=[
                        server, client, gui, free_gui, pick_place, rviz
                    ]),
                ],
            )
        ),
    ])

    return ld
