cd your_ros_ws
colcon build --packages-select 41068_ignition_bringup
source install/setup.zsh or install/setup.sh
ros2 launch 41068_ignition_bringup 41068_both.launch.py world:=new_world rviz:=True nav2:=True use_sim_time:=True
