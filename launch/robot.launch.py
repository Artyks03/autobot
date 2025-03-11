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


    package_name='autobot' 

    # Spousti robot_state_publisher (rsp.launch.py) + sim_time false
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false'}.items()
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
 
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'params','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # CONTROLLER MANAGER u realneho robota a jeho kontrolery -> musime ho spustit sami
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params= os.path.join(get_package_share_directory(package_name),'params','my_controllers.yaml')

    controller_manager= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params]
    )
    
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    

    ld = LaunchDescription()

    ld.add_action(rsp)
    ld.add_action(rplidar)
    ld.add_action(camera)
    ld.add_action(delayed_controller_manager)
    #ld.add_action(diff_drive_spawner)
    #ld.add_action(joint_broad_spawner)
    ld.add_action(delayed_diff_drive_spawner)
    ld.add_action(delayed_joint_broad_spawner)

    return ld
  