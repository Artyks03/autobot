PRED SPUSTENIM!!
v adresari balicku

$colcon build --symlink-install (staci 1x)

$source install/setup.bash (v kazdem novem terminalu)


SIMULACE (Gazebo Ignition)/////////////////

$ros2 launch autobot sim.launch.py

REAL ROBOT/////////////////////////////////

na robotu: $ros2 launch autobot robot.launch.py

na pc: ros2 launch autobot joystick.launch.py

NAVIGACE///////////////////////////////////

SLAM (Mapovani)>>>>>

$ros2 launch slam_toolbox online_async_launch.py slam_params_file:=./src/robotek/config/mapper_params_online_async.yaml use_sim_time:=false

LOKALIZACE>>>>>>>>>>

$ros2 launch robotek localization_launch.py map:={nazev_mapy}.yaml

NAVIGACE>>>>>>>>>>>>

$ros2 launch robotek navigation_launch.py use_sim_time:=false map_subcribe_transient_local:=true

