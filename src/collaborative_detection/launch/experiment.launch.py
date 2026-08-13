#!/usr/bin/env python3
"""
Full stack experiment launch: 2 robots + GNSS + UWB + attacker + CUSUM detector.

Launches the complete TFM pipeline:
  1. Two TurtleBot3 robots in Gazebo (two_robots.launch.py)
  2. GNSS simulator node (adds noise to odom)
  3. UWB ranging simulator node (adds noise to inter-robot distance)
  4. Meaconing injector (spoofs GNSS when activated)
  5. CUSUM detector (publishes alerts on meaconing detection)
  6. Robot mover (autonomous circular motion)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Force FastDDS to use UDP only. The built-in Shared Memory (SHM) transport
# hangs on macOS when many nodes start at once (Gazebo bridges, sim nodes,
# detector...), which made robot2's spawn and bridges stall mid-init.
# Setting the env var at import time guarantees every spawned node inherits it.
os.environ.setdefault(
    'FASTRTPS_DEFAULT_PROFILES_FILE',
    os.path.join(
        get_package_share_directory('collaborative_detection'),
        'config', 'fastdds_udp_only.xml',
    ),
)


def generate_launch_description():
    pkg_dir = get_package_share_directory('collaborative_detection')
    config = os.path.join(pkg_dir, 'config', 'params.yaml')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('x1', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y1', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('x2', default_value='3.0'))
    ld.add_action(DeclareLaunchArgument('y2', default_value='0.0'))

    # --- 1. Two robots in Gazebo ---
    # Forward the spawn positions so e.g. `x2:=5.0` reaches the robot spawner.
    two_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'two_robots.launch.py')
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'x1': LaunchConfiguration('x1'),
            'y1': LaunchConfiguration('y1'),
            'x2': LaunchConfiguration('x2'),
            'y2': LaunchConfiguration('y2'),
        }.items(),
    )
    ld.add_action(two_robots)

    # --- 2. GNSS simulator (delayed: wait for robots to spawn) ---
    gnss_node = Node(
        package='collaborative_detection',
        executable='gnss_sim_node',
        name='gnss_sim_node',
        output='screen',
        parameters=[config],
    )
    ld.add_action(TimerAction(period=5.0, actions=[gnss_node]))

    # --- 3. UWB ranging simulator ---
    uwb_node = Node(
        package='collaborative_detection',
        executable='uwb_sim_node',
        name='uwb_sim_node',
        output='screen',
        parameters=[config],
    )
    ld.add_action(TimerAction(period=5.0, actions=[uwb_node]))

    # --- 4. Meaconing injector ---
    meaconing_node = Node(
        package='collaborative_detection',
        executable='meaconing_injector',
        name='meaconing_injector',
        output='screen',
        parameters=[config],
    )
    ld.add_action(TimerAction(period=5.5, actions=[meaconing_node]))

    # --- 5. CUSUM detector (last: depends on GNSS spoofed + UWB) ---
    cusum_node = Node(
        package='collaborative_detection',
        executable='cusum_detector_node',
        name='cusum_detector_node',
        output='screen',
        parameters=[config],
    )
    ld.add_action(TimerAction(period=6.0, actions=[cusum_node]))

    # --- 6. Robot mover (autonomous circular motion) ---
    mover_node = Node(
        package='collaborative_detection',
        executable='robot_mover_node',
        name='robot_mover_node',
        output='screen',
        parameters=[config],
    )
    ld.add_action(TimerAction(period=7.0, actions=[mover_node]))

    # --- 7. GNSS visualizer (RViz2 markers) ---
    viz_node = Node(
        package='collaborative_detection',
        executable='gnss_viz_node',
        name='gnss_viz_node',
        output='screen',
        parameters=[config],
    )
    ld.add_action(TimerAction(period=7.5, actions=[viz_node]))

    return ld
