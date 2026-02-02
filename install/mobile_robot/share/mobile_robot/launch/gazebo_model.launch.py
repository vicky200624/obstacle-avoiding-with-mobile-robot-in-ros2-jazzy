import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.substitutions import Command
from launch.actions import ExecuteProcess


from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    robotXacroName = 'differential_drive_robot'
    namePackage = 'mobile_robot'
    modelFileRelativePath = 'model/robot.xacro'
    pathModelFile = os.path.join(
        get_package_share_directory(namePackage), modelFileRelativePath
    )

    world_file = PathJoinSubstitution([
    FindPackageShare(namePackage),
    'worlds',
    'house.world'
])


    # Parse XACRO to URDF
    #robotDescription = xacro.process_file(pathModelFile).toxml()
    robotDescription = xacro.process_file(
    pathModelFile,
    mappings={'gazebo_file': os.path.join(
        get_package_share_directory(namePackage),
        'model',
        'robot.gazebo'
    )}
).toxml()


    # Gazebo Sim launch (ros_gz_sim)
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    gazeboLaunch = IncludeLaunchDescription(
    gazebo_rosPackageLaunch,
    launch_arguments={
        'gz_args': f'-r -v 4 {os.path.join(get_package_share_directory(namePackage), "worlds", "house.world")}',
        'on_exit_shutdown': 'true'
    }.items()
)



    spawnModelNodeGazebo = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', robotXacroName,
        '-topic', 'robot_description',
        'z', '0.1'
    ],
    output='screen',
)



    # Publish robot description to TF
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': True
        }],
    )

    # ROS-GZ bridge parameters (optional)
    bridge_params = os.path.join(
        get_package_share_directory(namePackage),
        'parameters',
        'bridge_parameters.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    # Launch description object
    
    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(nodeRobotStatePublisher)
    launchDescriptionObject.add_action(gazeboLaunch)

    launchDescriptionObject.add_action(
        TimerAction(
            period=3.0,
            actions=[spawnModelNodeGazebo]
        )
    )

    launchDescriptionObject.add_action(start_gazebo_ros_bridge_cmd)

    return launchDescriptionObject


   