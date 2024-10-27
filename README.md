Steps to launch simulation (cf. https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation):
- source install/local_setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo empty_world.launch.py

Steps to launch exercises:
- rosdep install -i --from-path src --rosdistro humble -y
- colcon build --packages-select go_to_point
- source install/local_setup.bash
- ros2 run go_to_point go_to_point_node -p goal_x=10.0 -p goal_y=10.0
- ros2 param list
- ros2 param set /go_to_point goal_x 1.0
- ros2 param set /go_to_point goal_y 1.0


For types list, cf.: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md