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
                )]), launch_arguments={'use_sim_time': 'true'}.items()
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


    # spusteni kontroleru se kterymi komunikuje gazebo ignition simulovanym controller managerem
    diff_drive_spawner = Node(package="controller_manager", executable="spawner", arguments=["diff_cont"])

    joint_broad_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_broad"])

    # most k propojeni ROS/topics s GAZEBO/topics
    bridge_params = os.path.join(get_package_share_directory(package),'params','gz_bridge.yaml')
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

    ld = LaunchDescription()
    # spusteni prvku
    ld.add_action(joystick_launcher)
    ld.add_action(rsp_launcher)
    # spusteni gazeba
    ld.add_action(world_arg)
    ld.add_action(gazebo_ignition)
    ld.add_action(robot_spawner)
    # spusteni kontroleru
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)
    # spusteni mostu ros control a ign control
    ld.add_action(ros_gz_bridge)
    ld.add_action(ros_gz_image_bridge)
    
    return ld