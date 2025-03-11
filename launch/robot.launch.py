import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node

def generate_launch_description():
    
    # neni tu twist_mux, tj. navigace nepojede bez spusteneho joystick.launch.py

    package_name='autobot' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    rplidar = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                     get_package_share_directory(package_name),'launch','rplidar.launch.py'
                 )])
    )
    
    camera = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                     get_package_share_directory(package_name),'launch','camera.launch.py'
                 )])
    )

    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(get_package_share_directory(package_name),'params','my_controllers.yaml')

    controller_manager= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_start = TimerAction(period=2.0, actions=[controller_manager, diff_drive_spawner, joint_broad_spawner])
    
    ld = LaunchDescription()

    ld.add_action(rsp)
    ld.add_action(rplidar)
    ld.add_action(camera)
    ld.add_action(delayed_start)

    return ld