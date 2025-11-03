# Completely Operational Distributed Ecological Scanner

This is project CODES

Simlink the Codes folder into your ros2 workspace folder /ros2_ws/src

Build stuff: colcon build --symlink-install --packages-select codes
Run: ros2 run codes controllers

To run the search pattern, launch the environment, run the controllers node, in the gui, press activate drone, then press start search pattern

Environment launch function: `ros2 launch 41068_ignition_bringup 41068_both.launch.py world:=generated_world use_sim_time:=True`

navs launch: `ros2 launch my_car_navigation navigation_launch.py`

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 20.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}}"

ros2 topic pub /goal_point geometry_msgs/PoseStamped "header:
  frame_id: 'map'
  stamp:
    sec: 0
    nanosec: 0
pose:
  position:
    x: -10.0
    y: -10.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" -1
