# Report

For types list, cf.: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md

## Task1

2. Try out the tele-operation and investigate the topics and nodes with suitable ROS commands 

```bash
ros2 node list
```

```bash
ros2 topic list  -t
```

3. Find out what topic and message are used for sending velocity commands to the TurtleBot

/turtle1/cmd_vel [geometry_msgs/msg/Twist]

4. Use the ros2 topic command to move the robot in a circle and in a square path

Circle

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Square

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}" && ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {y: 2.0}}" && ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: -2.0}}" && ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {y: -2.0}}"
```

## Task 2.2

Steps to launch simulation (cf. https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation):
- source install/local_setup.bash
- export TURTLEBOT3_MODEL=burger
- ros2 launch turtlebot3_gazebo empty_world.launch.py

Steps to launch exercises:
- rosdep install -i --from-path src --rosdistro humble -y
- colcon build --packages-select go_to_point
- source install/local_setup.bash
- ros2 run follow_path follow_path_node
    - {poses: [{pose: {position: {x: 0, y: 2}}}, {pose: {position: {x: 2, y: 2}}}, {pose: {position: {x: 2, y: 0}}}, {pose: {position: {x: 0, y: 0}}}]}

## Task 3

ros2 run rqt_console rqt_console
ros2 run rqt_plot rqt_plot /error/data

ros2 bag record -o error_bag /error
ros2 bag play error_bag