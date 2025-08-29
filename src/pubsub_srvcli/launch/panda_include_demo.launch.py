from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Args
    json_path_arg  = DeclareLaunchArgument(
        "json_path",
        default_value="/home/revengeofthesob/ros3_ws/src/pubsub_srvcli/src/veri.json",
        description="Targets JSON path"
    )
    use_rviz_arg   = DeclareLaunchArgument("use_rviz",   default_value="true")
    launch_gui_arg = DeclareLaunchArgument("launch_gui", default_value="false")
    domain_arg     = DeclareLaunchArgument("domain_id",  default_value="71")

    json_path  = LaunchConfiguration("json_path")
    use_rviz   = LaunchConfiguration("use_rviz")
    launch_gui = LaunchConfiguration("launch_gui")
    domain_id  = LaunchConfiguration("domain_id")

    # Tüm alt proseslere aynı domain
    set_domain = SetEnvironmentVariable(name="ROS_DOMAIN_ID", value=domain_id)

    # 1) Resmi demo’yu include et (URDF+SRDF+move_group(+opsiyonel RViz))
    panda_cfg_pkg = get_package_share_directory("moveit_resources_panda_moveit_config")
    demo_launch   = os.path.join(panda_cfg_pkg, "launch", "demo.launch.py")
    include_demo  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(demo_launch),
        launch_arguments={"launch_rviz": use_rviz}.items()
    )

    # 2) Server – MoveIt paramlarını da veriyoruz (semantic garanti)
    mcfg   = MoveItConfigsBuilder("panda", package_name="moveit_resources_panda_moveit_config").to_moveit_configs()
    params = mcfg.to_dict()

    server = TimerAction(
        period=3.0,  # demo kurulsun
        actions=[
            Node(
                package="pubsub_srvcli",
                executable="server_moveit_panda",
                name="panda_move_server",
                output="screen",
                parameters=[params, {"json_path": json_path}],
            )
        ],
    )

    # 3) (opsiyonel) GUI – ROS’suz script; 5 sn gecikmeyle
    gui_script = os.path.join(
        get_package_share_directory("pubsub_srvcli"), "scripts", "gui_target_to_json_noros.py"
    )
    gui = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(cmd=["python3", "-u", gui_script, "--json", json_path], output="screen")],
        condition=IfCondition(launch_gui),
    )

    return LaunchDescription([
        json_path_arg, use_rviz_arg, launch_gui_arg, domain_arg,
        set_domain,
        include_demo,
        server,
        gui,
    ])
