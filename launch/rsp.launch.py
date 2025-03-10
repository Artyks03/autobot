import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

#import xacro


def generate_launch_description():

    # Kontrola parametru v prikazovem radku
    use_sim_time = LaunchConfiguration('use_sim_time')
 
    # Zpracovani modelu URDF + parametru
    pkg_path = os.path.join(get_package_share_directory('autobot'))
    xacro_file = os.path.join(pkg_path,'description','robot_main.xacro')
    robot_description_params = Command(['xacro ', xacro_file, ' sim_mode:=', use_sim_time])
    
    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_params, 'use_sim_time': use_sim_time}]
        )

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'))
    ld.add_action(robot_state_publisher)

    return ld