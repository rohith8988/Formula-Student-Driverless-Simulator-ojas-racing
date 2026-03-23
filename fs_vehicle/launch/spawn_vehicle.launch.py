"""
spawn_vehicle.launch.py
───────────────────────
Launches the FS car in an empty Gazebo Harmonic world.

What this does:
  1. Starts Gazebo Sim with empty_track.sdf
  2. Runs xacro to build the URDF and starts robot_state_publisher
  3. Spawns the fs_car model into the running simulation
  4. Starts ros_gz_bridge for all sensor/command topics
  5. Launches RViz with the fs_car config (unless launch_rviz:=false)

Usage:
  ros2 launch fs_vehicle spawn_vehicle.launch.py
  ros2 launch fs_vehicle spawn_vehicle.launch.py gz_headless:=true
  ros2 launch fs_vehicle spawn_vehicle.launch.py launch_rviz:=false
"""

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg = get_package_share_directory('fs_vehicle')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ── Launch arguments ───────────────────────────────────────────
    gz_headless_arg = DeclareLaunchArgument(
        'gz_headless',
        default_value='false',
        description='Run Gazebo without GUI (for CI / batch testing)',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg, 'worlds', 'empty_track.sdf'),
        description='Absolute path to the Gazebo world file to load',
    )
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='Vehicle spawn x position',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Vehicle spawn y position',
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.5',
        description='Vehicle spawn z position',
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw',
        default_value='0.0',
        description='Vehicle spawn yaw angle in radians',
    )
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='fs_car',
        description='Gazebo model name for the spawned vehicle',
    )
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='fs_world',
        description='Gazebo world name to receive the spawned vehicle',
    )
    bridge_config_arg = DeclareLaunchArgument(
        'bridge_config',
        default_value=os.path.join(pkg, 'config', 'ros_gz_bridge.yaml'),
        description='Path to the ros_gz_bridge control topics YAML config file',
    )
    sensors_bridge_config_arg = DeclareLaunchArgument(
        'sensors_bridge_config',
        default_value=os.path.join(pkg, 'config', 'ros_gz_bridge_sensors.yaml'),
        description='Path to the ros_gz_bridge sensor topics YAML config file',
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz with the fs_car config',
    )

    gz_headless    = LaunchConfiguration('gz_headless')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    model_name = LaunchConfiguration('model_name')
    world_name = LaunchConfiguration('world_name')
    bridge_cfg = LaunchConfiguration('bridge_config')
    sensors_bridge_cfg = LaunchConfiguration('sensors_bridge_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    urdf_file  = os.path.join(pkg, 'urdf', 'fs_car.urdf.xacro')
    rviz_config = os.path.join(pkg, 'config', 'fs_car.rviz')

    # GZ_SIM_RESOURCE_PATH lets Gazebo resolve model://fs_vehicle/meshes/...
    # It must point to the parent directory that contains the 'fs_vehicle' folder
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(pkg),  # .../install/fs_vehicle/share
    )

    # GZ_SIM_SYSTEM_PLUGIN_PATH lets Gazebo find our custom FsAckermannDrive.so
    gz_plugin_path = AppendEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(get_package_prefix('fs_vehicle_plugins'), 'lib'),
    )

    # ── Performance environment variables ──────────────────────────
    # Preload libgamemodeauto so Gazebo requests max CPU frequency from
    # the gamemode daemon automatically (equivalent to gamemoderun prefix).
    gamemode_preload = AppendEnvironmentVariable(
        'LD_PRELOAD',
        '/usr/lib/x86_64-linux-gnu/libgamemodeauto.so.0',
    )

    # Allow Eigen/OpenMP (used by DART and the bridge serialisers) to use
    # up to 4 threads for matrix operations without starving the physics core.
    omp_threads = SetEnvironmentVariable('OMP_NUM_THREADS', '4')

    # ── Robot description via xacro ────────────────────────────────
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str,
    )

    # ── 1. Gazebo Sim ──────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r = run immediately, -s = server only (headless)
            'gz_args': [
                PythonExpression([
                    '"-r -s " if "', gz_headless, '".lower() == "true" else "-r "'
                ]),
                world_file,
            ],
        }.items(),
    )

    # ── 2. Robot state publisher ───────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 3. Spawn entity ────────────────────────────────────────────
    # Wait for Gazebo to fully initialise before spawning.
    # 3 s covers Gazebo startup + ogre2 render engine init on most machines.
    # If the car occasionally fails to spawn, increase this value.
    spawn_car = TimerAction(
        period=0.1,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_fs_car',
            output='screen',
            arguments=[
                '-world', world_name,
                '-name',  model_name,
                '-topic', '/robot_description',
                '-x', spawn_x,
                '-y', spawn_y,
                '-z', spawn_z,
                '-Y', spawn_yaw,
            ],
        )],
    )

    # ── 4. ROS ↔ Gazebo bridge (split into two parallel nodes) ────
    # control_bridge: low-bandwidth cmd_vel / odom / tf / joints / imu / clock
    # sensors_bridge: high-bandwidth lidar scan + point cloud + camera
    # Each runs as a separate OS process → separate thread, so sensor
    # serialisation (23 k-point clouds @ 10 Hz) can never block control data.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_control',
        output='screen',
        parameters=[{
            'config_file': bridge_cfg,
            'use_sim_time': use_sim_time,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    sensors_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_sensors',
        output='screen',
        parameters=[{
            'config_file': sensors_bridge_cfg,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── 5. RViz ────────────────────────────────────────────────────
    # Delay RViz until the TF tree is populated:
    #   - spawn at 3 s → AckermannSteering starts publishing odom→base_link
    #   - robot_state_publisher needs joint_states from the bridge
    # 6 s gives all of that time to settle before RViz starts looking up frames.
    rviz = TimerAction(
        period=6.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_rviz),
        )],
    )

    return LaunchDescription([
        gz_headless_arg,
        use_sim_time_arg,
        world_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        model_name_arg,
        world_name_arg,
        bridge_config_arg,
        sensors_bridge_config_arg,
        launch_rviz_arg,
        # env vars applied before any process starts
        gamemode_preload,
        omp_threads,
        gz_resource_path,
        gz_plugin_path,
        gz_sim,
        robot_state_publisher,
        spawn_car,
        bridge,
        sensors_bridge,
        rviz,
    ])
