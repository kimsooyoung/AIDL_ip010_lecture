import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, TimerAction

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare

from osrf_pycommon.terminal_color import ansi

def generate_launch_description():

    open_rviz = LaunchConfiguration('open_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_open_rviz = DeclareLaunchArgument(
        'open_rviz',
        default_value='true',
        description='Launch Rviz?'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock'
    )

    # gazebo
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_path = os.path.join(get_package_share_directory('ip010_gazebo'))
    world_path = os.path.join(pkg_path, 'worlds', 'factory_world.world')

    robomaker_pkg_path = os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'))
    gazebo_model_path = os.path.join(robomaker_pkg_path, 'models')
    
    # TODO : Check gazebo uri path compatibility
    # world_path = os.path.join(robomaker_pkg_path, 'worlds', 'no_roof_small_warehouse', 'no_roof_small_warehouse.world')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ":" + gazebo_model_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_model_path

    print(ansi("yellow"), "If it's your 1st time to download Gazebo model on your computer, it may take few minutes to finish.", ansi("reset"))

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher
    urdf_file = os.path.join(pkg_path, 'urdf', 'ip010.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Spawn Robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ip010',
            '-x', '3.45',
            '-y', '2.15',
            '-z', '0.10',
            '-Y', '3.14',
            # '-x', '6.0',
            # '-y', '2.2',
            # '-z', '0.1',
            # '-R', '0.0',
            # '-P', '0.0',
            # '-Y', '3.14',
        ],
        output='screen'
    )

    # rqt robot steering
    rqt_robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        name='rqt_robot_steering',
        output='screen'
    )

    rviz_config_file = os.path.join(pkg_path, "rviz", "gazebo_default.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration("open_rviz")),
    )

    return LaunchDescription([
        declare_open_rviz,
        declare_use_sim_time,

        TimerAction(
            period=3.0,
            actions=[rviz2]
        ),
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        # rqt_robot_steering,
    ])