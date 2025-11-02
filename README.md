# Completely Operational Distributed Ecological Scanner

This is project CODES

Environment launch function: `ros2 launch 41068_ignition_bringup 41068_both.launch.py world:=generated_world nav2:=True use_sim_time:=True`

navs launch: `ros2 launch my_car_navigation navigation_launch.py`

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 20.0, y: 20.0, z: 0.0}, orientation: {w: 1.0}}}}"
