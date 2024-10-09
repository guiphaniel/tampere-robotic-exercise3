# Report

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

