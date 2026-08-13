#!/usr/bin/env python3
"""
Launch two TurtleBot3 waffles in Gazebo Sim for the
collaborative GNSS meaconing detection TFM experiment.

Key design: bridges are spawned OUTSIDE PushRosNamespace with explicit
absolute topic names on both ROS and Gazebo sides. This avoids the
namespace + scoped-topic resolution issues observed with the previous
PushRosNamespace-wrapped bridge approach.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

# Force FastDDS to use UDP only. The built-in Shared Memory (SHM) transport
# hangs on macOS when many nodes start at once, stalling robot2's spawn and
# the ros_gz_bridge nodes. Set at import time so every spawned node inherits it.
os.environ.setdefault(
    'FASTRTPS_DEFAULT_PROFILES_FILE',
    os.path.join(
        get_package_share_directory('collaborative_detection'),
        'config', 'fastdds_udp_only.xml',
    ),
)

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
MODEL_FOLDER = f'turtlebot3_{TURTLEBOT3_MODEL}'
SDF_FILE = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'),
    'models', MODEL_FOLDER, 'model.sdf'
)
URDF_FILE = os.path.join(
    get_package_share_directory('turtlebot3_gazebo'),
    'urdf', f'{MODEL_FOLDER}.urdf'
)


def _make_custom_sdf(model_name: str) -> str:
    """
    Copy the TurtleBot3 SDF and patch topic names to be model-specific.

    Gazebo Sim publishes to GLOBAL transport topics by default, so both
    robots would share /odom, /cmd_vel, /joint_states. We rewrite them to
    /model/<name>/odom etc. so each robot has its own transport namespace.

    Returns the path to a temp .sdf file.
    """
    with open(SDF_FILE, 'r') as f:
        sdf = f.read()

    # Patch DiffDrive topics
    sdf = sdf.replace(
        '<topic>cmd_vel</topic>',
        f'<topic>/model/{model_name}/cmd_vel</topic>')
    sdf = sdf.replace(
        '<odom_topic>odom</odom_topic>',
        f'<odom_topic>/model/{model_name}/odom</odom_topic>')
    sdf = sdf.replace(
        '<topic>joint_states</topic>',
        f'<topic>/model/{model_name}/joint_states</topic>')

    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', prefix=f'{model_name}_', delete=False)
    tmp.write(sdf)
    tmp.close()
    return tmp.name


def _generate_bridge_yaml(robot_ns: str, model_name: str) -> str:
    """
    Generate a bridge YAML with model-scoped Gazebo topics.

    We patch the SDF so each robot publishes to /model/<name>/odom,
    /model/<name>/cmd_vel, etc. The bridge subscribes/publishes to those
    same scoped topics on the Gazebo side.

    ROS topics use ABSOLUTE names (/{robot_ns}/...) because the bridge
    runs OUTSIDE PushRosNamespace.
    """
    gz_prefix = f'/model/{model_name}'
    yaml_content = f"""# Auto-generated bridge config for {model_name} (ns={robot_ns})
- ros_topic_name: "/{robot_ns}/clock"
  gz_topic_name: "clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/{robot_ns}/joint_states"
  gz_topic_name: "{gz_prefix}/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/{robot_ns}/odom"
  gz_topic_name: "{gz_prefix}/odom"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/{robot_ns}/tf"
  gz_topic_name: "{gz_prefix}/tf"
  ros_type_name: "tf2_msgs/msg/TFMessage"
  gz_type_name: "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/{robot_ns}/cmd_vel"
  gz_topic_name: "{gz_prefix}/cmd_vel"
  ros_type_name: "geometry_msgs/msg/TwistStamped"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/{robot_ns}/imu"
  gz_topic_name: "imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/{robot_ns}/scan"
  gz_topic_name: "scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
"""
    return yaml_content


def _make_robot_group(robot_ns: str, model_name: str, x: str, y: str, delay: float = 0.0):
    """
    Create a GroupAction for a single robot:
    - robot_state_publisher (namespaced via PushRosNamespace)
    - ros_gz_sim create (spawn in Gazebo)
    - ros_gz_bridge (GLOBAL — NOT inside PushRosNamespace — with explicit absolute topics)
    """
    with open(URDF_FILE, 'r') as f:
        robot_desc = f.read()

    actions = [
        PushRosNamespace(robot_ns),

        # Robot state publisher (namespaced)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_description': robot_desc,
                'frame_prefix': f'{robot_ns}/',
            }],
        ),

        # Spawn in Gazebo Sim with a patched SDF (per-robot topics)
        Node(
            package='ros_gz_sim',
            executable='create',
            name=f'create_{model_name}',
            arguments=[
                '-name', model_name,
                '-file', _make_custom_sdf(model_name),
                '-x', x,
                '-y', y,
                '-z', '0.01',
            ],
            output='screen',
        ),
    ]

    if delay > 0:
        return TimerAction(period=delay, actions=[GroupAction(actions)])
    return GroupAction(actions)


def _make_bridge(robot_ns: str, model_name: str, delay: float = 0.0):
    """Create a GLOBAL bridge node (no PushRosNamespace) with explicit absolute topics."""
    bridge_yaml_content = _generate_bridge_yaml(robot_ns, model_name)
    tmp_yaml = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix=f'bridge_{model_name}_', delete=False
    )
    tmp_yaml.write(bridge_yaml_content)
    tmp_yaml.close()

    node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{model_name}',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={tmp_yaml.name}',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    if delay > 0:
        return TimerAction(period=delay, actions=[node])
    return node


def generate_launch_description():
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    turtlebot3_gazebo_pkg = get_package_share_directory('turtlebot3_gazebo')

    world_file = os.path.join(turtlebot3_gazebo_pkg, 'worlds', 'empty_world.world')

    x1 = LaunchConfiguration('x1', default='0.0')
    y1 = LaunchConfiguration('y1', default='0.0')
    x2 = LaunchConfiguration('x2', default='3.0')
    y2 = LaunchConfiguration('y2', default='0.0')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('x1', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('y1', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('x2', default_value='3.0'))
    ld.add_action(DeclareLaunchArgument('y2', default_value='0.0'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))

    ld.add_action(AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_pkg, 'models'),
    ))

    # Gazebo Sim (server + client)
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
    ))

    # GUI client — skipped in headless mode (gui:=false). The OGRE renderer
    # is broken on macOS and the GUI is not needed to record rosbags, so the
    # experiment runner disables it to save resources.
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-g -v2 ',
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    ))

    # Robot 1 + its bridge (global)
    ld.add_action(_make_robot_group('robot1', 'robot1', x1, y1, delay=2.0))
    ld.add_action(_make_bridge('robot1', 'robot1', delay=2.0))

    # Robot 2 + its bridge (global)
    ld.add_action(_make_robot_group('robot2', 'robot2', x2, y2, delay=3.0))
    ld.add_action(_make_bridge('robot2', 'robot2', delay=3.0))

    return ld
