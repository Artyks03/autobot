import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package='autobot' 

    # Spousti uzly spojene s teleoperovani joystickem (joystick.launch.py) + sim_time true
    # !! obsahuje twist_mux, tj. v pripade nespusteni nebude fungovat transformace /cmd_vel navigace
    joystick_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    # Spousti robot_state_publisher (rsp.launch.py) + sim_time true
    rsp_launcher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    # definice 

    default_world = os.path.join(get_package_share_directory(package), 'worlds', 'empty.world')    
    world_arg = DeclareLaunchArgument('world', default_value=default_world, description='World to load')

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    world = LaunchConfiguration('world')
    gazebo_ignition = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py')]),
                    launch_arguments={'ign_args': ['-r ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # spawn robota v gazebo, soucast balicku ros_gz_sim, nacte tema /robot_description a jmeno robota
    robot_spawner = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description', '-name', 'autobot', '-z', '0.1'],
                        output='screen')


    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])

    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])


    bridge_params = os.path.join(get_package_share_directory(package),'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}',
        ]
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"]
    )


    return LaunchDescription([
        joystick_launcher,
        rsp_launcher,
        world_arg,
        gazebo_ignition,
        robot_spawner,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
        ros_gz_image_bridge,
        
    ])