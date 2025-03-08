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
    use_ros2_control = LaunchConfiguration('use_ros2_control')
 
    # Zpracovani modelu URDF + parametru
    pkg_path = os.path.join(get_package_share_directory('autobot'))
    xacro_file = os.path.join(pkg_path,'description','robot_main.xacro')
    robot_description_params = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    # Vytvoreni uzlu "/robot_state_publisher"
    params = {'robot_description': robot_description_params, 'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'),
        DeclareLaunchArgument('use_ros2_control', default_value='true', description='Use ros2_control if true'),

        robot_state_publisher # <<< UZEL
    ])