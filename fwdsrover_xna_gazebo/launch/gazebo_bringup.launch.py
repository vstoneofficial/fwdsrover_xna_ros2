import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnShutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _check_no_running_gazebo(context, *args, **kwargs):
    has_gzserver = subprocess.run(
        ["pgrep", "gzserver"], stdout=subprocess.DEVNULL
    ).returncode == 0
    has_gzclient = subprocess.run(
        ["pgrep", "gzclient"], stdout=subprocess.DEVNULL
    ).returncode == 0
    port_in_use = subprocess.run(
        ["bash", "-lc", "ss -lnt | grep -q ':11345'"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0

    # 既存Gazeboやマスター用ポート占有がある場合は即中断する。
    # （起動継続すると /spawn_entity 不達で誤動作するため）
    if has_gzserver or has_gzclient or port_in_use:
        raise RuntimeError(
            "[gazebo_bringup] 既存Gazeboまたは11345ポート占有を検出。"
            "重複起動を解消してから再実行してください。"
        )

    return [LogInfo(msg="[gazebo_bringup] No existing Gazebo process detected.")]


def _cleanup_stale_gazebo(context, *args, **kwargs):
    cleanup_enabled = LaunchConfiguration("cleanup_stale_gazebo").perform(context).strip().lower()
    if cleanup_enabled not in ("1", "true", "yes", "on"):
        return []

    def _gazebo_in_use():
        has_gzserver = subprocess.run(
            ["pgrep", "-x", "gzserver"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).returncode == 0
        has_gzclient = subprocess.run(
            ["pgrep", "-x", "gzclient"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).returncode == 0
        port_in_use = subprocess.run(
            ["bash", "-lc", "ss -lnt | grep -q ':11345'"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode == 0
        return has_gzserver or has_gzclient or port_in_use

    for _ in range(2):
        subprocess.run(
            ["pkill", "-TERM", "-x", "gzserver"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        subprocess.run(
            ["pkill", "-TERM", "-x", "gzclient"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        for _ in range(20):
            if not _gazebo_in_use():
                return [LogInfo(msg="[gazebo_bringup] Cleaned up stale Gazebo processes")]
            subprocess.run(["bash", "-lc", "sleep 0.25"])

    subprocess.run(
        ["pkill", "-KILL", "-x", "gzserver"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    subprocess.run(
        ["pkill", "-KILL", "-x", "gzclient"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    for _ in range(20):
        if not _gazebo_in_use():
            return [LogInfo(msg="[gazebo_bringup] Cleaned up stale Gazebo processes")]
        subprocess.run(["bash", "-lc", "sleep 0.25"])

    return [LogInfo(msg="[gazebo_bringup] Cleaned up stale Gazebo processes")]


def _make_world_with_state(context, *args, **kwargs):
    physics_type = LaunchConfiguration("physics").perform(context)
    if physics_type not in ("ode", "bullet"):
        physics_type = "ode"

    world_xml = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <physics type="{physics_type}">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <include><uri>model://ground_plane</uri></include>
    <include><uri>model://sun</uri></include>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <publish_model_state>true</publish_model_state>
      <publish_link_state>true</publish_link_state>
      <update_rate>30</update_rate>
      <model_states_topic>/model_states</model_states_topic>
      <link_states_topic>/link_states</link_states_topic>
    </plugin>
  </world>
</sdf>
"""

    fd, path = tempfile.mkstemp(prefix="fwdsrover_world_", suffix=".world")
    with os.fdopen(fd, "w") as f:
        f.write(world_xml)

    return [
        LogInfo(msg=f"[gazebo_bringup] generated temp world: {path}"),
        SetLaunchConfiguration("world_path", path),
    ]


def _create_urdf_and_rsp(context, *args, **kwargs):
    rover = LaunchConfiguration("rover").perform(context)

    urdf_dir = os.path.join(
        get_package_share_directory("fwdsrover_description"),
        "urdf",
    )

    xacro_map = {
        "x40a": "x40a.xacro",
        "x120a": "x120a.xacro",
        "x120a_lb": "x120a_lb.xacro",
    }
    if rover not in xacro_map:
        raise RuntimeError(
            f"[gazebo_bringup] Unsupported rover '{rover}'. "
            "Use one of: x40a, x120a, x120a_lb"
        )

    xacro_file = os.path.join(urdf_dir, xacro_map[rover])

    fd, urdf_path = tempfile.mkstemp(prefix="robot_", suffix=".urdf")
    os.close(fd)

    gen = ExecuteProcess(
        cmd=[FindExecutable(name="xacro"), xacro_file, "-o", urdf_path],
        output="screen",
    )

    # gazebo_ros2_control からも安定して参照できるように
    # robot_description パラメータを明示的に渡す。
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(
                    [FindExecutable(name="xacro"), " ", xacro_file]
                ),
                "use_sim_time": True,
            }
        ],
    )

    return [
        SetLaunchConfiguration("urdf_path", urdf_path),
        LogInfo(msg=f"[gazebo_bringup] generated temp urdf: {urdf_path}"),
        gen,
        rsp,
    ]


def _resolve_spawn_z(context, *args, **kwargs):
    rover = LaunchConfiguration("rover").perform(context)
    spawn_z_arg = LaunchConfiguration("spawn_z").perform(context).strip()

    # 引数が未指定の場合のみ車種別既定値を適用する。
    if spawn_z_arg:
        spawn_z = spawn_z_arg
    elif rover in ("x120a", "x120a_lb"):
        spawn_z = "0.12"
    else:
        spawn_z = "0.03"

    return [
        SetLaunchConfiguration("spawn_z", spawn_z),
        LogInfo(msg=f"[gazebo_bringup] spawn z for {rover}: {spawn_z}"),
    ]


def _cleanup_temp_files(context, *args, **kwargs):
    world_path = LaunchConfiguration("world_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)

    for p in (world_path, urdf_path):
        try:
            if p and os.path.exists(p):
                os.remove(p)
        except Exception:
            pass

    return [LogInfo(msg="[gazebo_bringup] Cleaned up temp files")]


def generate_launch_description():
    rover = LaunchConfiguration("rover")
    gui = LaunchConfiguration("gui")
    world_path = LaunchConfiguration("world_path")
    urdf_path = LaunchConfiguration("urdf_path")
    spawn_z = LaunchConfiguration("spawn_z")

    check_no_running_gazebo = OpaqueFunction(function=_check_no_running_gazebo)
    make_world = OpaqueFunction(function=_make_world_with_state)
    create_urdf_and_rsp = OpaqueFunction(function=_create_urdf_and_rsp)
    resolve_spawn_z = OpaqueFunction(function=_resolve_spawn_z)
    cleanup_stale_gazebo = OpaqueFunction(function=_cleanup_stale_gazebo)
    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "gazebo_ros", "gazebo.launch.py",
            ["gui:=", gui],
            ["world:=", world_path],
        ],
        output="screen",
    )

    spawn = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "gazebo_ros",
            "spawn_entity.py",
            "-entity",
            rover,
            "-file",
            urdf_path,
            "-z",
            spawn_z,
        ],
        output="screen",
    )

    # 既にactiveなcontrollerは再設定せず、未起動なものだけ順次有効化する。
    # group spawner は joint_state_broadcaster を active 後に再設定して
    # exit 1 になることがあるため使用しない。
    spawners_group = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "sleep 12.0; "
            "until ros2 control list_controllers >/dev/null 2>&1; do "
            "echo '[gazebo_bringup] waiting for /controller_manager/list_controllers'; "
            "sleep 1.0; "
            "done; "
            "if ros2 control list_controllers 2>/dev/null | grep -Eq 'joint_state_broadcaster\\s+active'; then "
            "echo '[gazebo_bringup] joint_state_broadcaster already active'; "
            "else "
            "ros2 run controller_manager spawner joint_state_broadcaster "
            "--controller-manager-timeout 120 "
            "--service-call-timeout 120 || exit 1; "
            "fi; "
            "if ros2 control list_controllers 2>/dev/null | grep -Eq 'steer_position_controller\\s+active'; then "
            "echo '[gazebo_bringup] steer_position_controller already active'; "
            "else "
            "ros2 run controller_manager spawner steer_position_controller "
            "--controller-manager-timeout 120 "
            "--service-call-timeout 120 || exit 1; "
            "fi; "
            "if ros2 control list_controllers 2>/dev/null | grep -Eq 'wheel_velocity_controller\\s+active'; then "
            "echo '[gazebo_bringup] wheel_velocity_controller already active'; "
            "else "
            "ros2 run controller_manager spawner wheel_velocity_controller "
            "--controller-manager-timeout 120 "
            "--service-call-timeout 120 || exit 1; "
            "fi",
        ],
        output="screen",
    )

    rover_twist_relay = Node(
        package="fwdsrover_description",
        executable="rover_twist_relay.py",
        name="rover_twist_relay",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("fwdsrover_description"),
                "params",
                "rover_twist_relay.yaml",
            ),
            {
                "rover": rover,
                "use_sim_time": True,
            },
        ],
    )

    gazebo_odom_bridge = Node(
        package="fwdsrover_xna_bringup",
        executable="gazebo_odom_bridge",
        name="gazebo_odom_bridge",
        parameters=[
            {"model_name": rover},
            {"base_link_name": "base_footprint"},
            {"use_sim_time": True},
        ],
        output="screen",
    )

    on_shutdown_cleanup = RegisterEventHandler(
        OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup_temp_files)])
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("cleanup_stale_gazebo", default_value="true"),
            DeclareLaunchArgument(
                "rover",
                default_value="x40a",
                description="Rover type: x40a, x120a, x120a_lb",
            ),
            DeclareLaunchArgument("physics", default_value="ode"),
            DeclareLaunchArgument("spawn_z", default_value=""),
            cleanup_stale_gazebo,
            check_no_running_gazebo,
            make_world,
            create_urdf_and_rsp,
            resolve_spawn_z,
            gazebo,
            spawn,
            spawners_group,
            rover_twist_relay,
            gazebo_odom_bridge,
            on_shutdown_cleanup,
        ]
    )
