from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, OpaqueFunction, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os
import subprocess

def _prepare_and_spawn(context, *args, **kwargs):
    pkg_share = get_package_share_directory('panda_yolo_pickplace')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'panda_gz.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # /tmp'ye URDF üret
    out_urdf = '/tmp/panda_gz.urdf'
    subprocess.run(['xacro', urdf_xacro, f'controllers_yaml:={controllers_yaml}', '-o', out_urdf], check=True)

    # Model spawn
    spawn = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'dual_table', '-name', 'panda',
            '-x', '0', '-y', '0', '-z', '0.74',
            '-file', out_urdf
        ],
        output='screen'
    )

    # Kontrolcüler
    sp_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/panda/controller_manager'],
        output='screen'
    )
    sp_arm = TimerAction(
        period=1.5,  # jsb'den biraz sonra
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['panda_arm_controller', '--controller-manager', '/panda/controller_manager'],
            output='screen'
        )]
    )
    return [spawn, sp_jsb, sp_arm]

def generate_launch_description():
    pkg_share = get_package_share_directory('panda_yolo_pickplace')
    moveit_share = get_package_share_directory('moveit_resources_panda_description')
    gz_ctrl_prefix = get_package_prefix('gz_ros2_control')

    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'panda_gz.urdf.xacro')
    world_sdf = os.path.join(pkg_share, 'world', 'dual_table.world.sdf')

    # ENV
    env_plugin = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(gz_ctrl_prefix, 'lib') + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    )
    env_resources = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        moveit_share + ':' + pkg_share + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    # Gazebo
    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world_sdf],
        output='screen'
    )

    # robot_description'u latched yayınlayan C++ node
    pub_desc = Node(
        package='panda_yolo_pickplace',
        executable='robot_description_publisher',
        name='robot_description_publisher',
        parameters=[{
            'topic': '/panda/robot_description',
            'urdf_xacro': urdf_xacro,
            'controllers_yaml': controllers_yaml,
        }],
        output='screen'
    )

    # spawn ve controller spawner'ları biraz gecikmeli çalıştır
    spawn_block = TimerAction(
        period=2.0,
        actions=[OpaqueFunction(function=_prepare_and_spawn)]
    )

    return LaunchDescription([
        env_plugin, env_resources,
        gz,
        pub_desc,
        spawn_block,
    ])
