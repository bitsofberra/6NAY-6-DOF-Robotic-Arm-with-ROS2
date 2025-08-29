# bringup_gz.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix
from moveit_configs_utils import MoveItConfigsBuilder
import os

def generate_launch_description():
    # ---- Args ----
    use_sim_time  = LaunchConfiguration('use_sim_time')
    gz_spawn_z    = LaunchConfiguration('gz_spawn_z')
    start_moveit  = LaunchConfiguration('start_moveit')  # MoveIt'i opsiyonel başlat

    # ---- Paths ----
    pkg_share      = FindPackageShare('panda_yolo_pickplace')
    panda_xacro    = PathJoinSubstitution([pkg_share, 'urdf', 'panda_gz.urdf.xacro'])
    bridge_yaml    = PathJoinSubstitution([pkg_share, 'config', 'bridge.yaml'])
    controllers_yaml = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    world          = PathJoinSubstitution([pkg_share, 'world', 'dual_table.world.sdf'])
    rviz_cfg       = PathJoinSubstitution([pkg_share, 'rviz', 'pickplace.rviz'])

    # ---- robot_description (xacro -> string) ----
    xacro_cmd = Command([FindExecutable(name='xacro'), ' ', panda_xacro])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    # ---- gz-ros2-control system plugin yolu ----
    gz_prefix = get_package_prefix('gz_ros2_control')
    set_gz_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(gz_prefix, 'lib')
    )

    # ---- Gazebo başlat (plugin ile) ----
    start_gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', '-s', 'gz-ros2-control-system', world],
        output='screen'
    )


    # ---- Robot State Publisher ----
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # world -> panda_link0 sabit TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0.74', '0', '0', '0', 'world', 'panda_link0']
    )

    # ---- Bridge ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_yaml}],
        output='screen'
    )

    # ---- Model spawn (xacro çıktısını -string ile veriyoruz) ----
    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'dual_table',
            '-name',  'panda',
            '-x', '0', '-y', '0', '-z', gz_spawn_z,
            '-string', xacro_cmd
        ]
    )

    # ---- Controllers (CM yolu: /panda/controller_manager) ----
    spawner_js = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/panda/controller_manager',
        ]
    )

    spawner_arm = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'panda_arm_controller',
            '--controller-manager', '/panda/controller_manager',
            '--param-file', controllers_yaml
        ]
    )

    # Spawn başladıktan sonra controller spawner'ları başlat
    start_spawners = RegisterEventHandler(
        OnProcessStart(target_action=spawn_model, on_start=[spawner_js, spawner_arm])
    )

    # ---- MoveIt (opsiyonel) ----
    moveit_config = MoveItConfigsBuilder(
        robot_name='panda',
        package_name='moveit_resources_panda_moveit_config'
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict()
    moveit_params['robot_description'] = robot_description
    moveit_params['use_sim_time'] = use_sim_time

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[moveit_params],
        output='screen',
        condition=IfCondition(start_moveit)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=IfCondition(start_moveit)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gz_spawn_z',   default_value='0.74'),
        DeclareLaunchArgument('start_moveit', default_value='false'),
        set_gz_path,
        start_gz,
        rsp,
        static_tf,
        bridge,
        spawn_model,
        start_spawners,
        move_group,
        rviz
    ])
