from math import atan2, pi
from math import sqrt
from time import sleep

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
from rosidl_runtime_py import message_to_yaml, set_message_fields

class GoToPoint(Node):    
    def __init__(self):
        # Initialize the node
        super().__init__('GoToPointCmd_publisher')
        
        # Initialize the pose
        self.pose = Pose()
        
        # Ask the user to input the goal position
        path = Path()
        
        path_string = input("Please input you Path (yaml format): ")
        path_yaml = yaml.safe_load(path_string)
        set_message_fields(path, path_yaml)
        
        # Initialize the publisher for the velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize the publisher for the error
        self.error_publisher = self.create_publisher(Float64, '/error', 10)
        
        # Initialize the subscriber to get the current pose
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odom, 10)
        
        # Check if the path is empty
        if len(path.poses) == 0:
            self.get_logger().info("Path is empty!")
            return
        
        # Print first goal position
        self.get_logger().info("First goal position: " + str(path.poses[0].pose.position.x) + ", " + str(path.poses[0].pose.position.y))
        
        # Initialize a timer that excutes call back function every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, lambda path=path: self.moveAlongPath(path))
        

    def update_odom(self, data: Odometry):
        self.pose = data.pose.pose

    def euclidean_distance(self, goal):
        return sqrt(pow((goal.x - self.pose.position.x), 2) +
                    pow((goal.y - self.pose.position.y), 2))

    def linear_vel(self, goal, constant=1.5):
        return min(constant * self.euclidean_distance(goal), 0.5)

    def steering_angle(self, goal):
        return atan2(goal.y - self.pose.position.y, goal.x - self.pose.position.x)
    
    # Set the angle between -pi and pi
    def angle_norm (self,angle):
        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        
        return angle

    def angular_vel(self, goal, constant=1.5):
        # cf. https://answers.ros.org/question/327256/equivalent-turtlesimpose-and-geometry_msgspose/
        
        orientation_list = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        steering_angle = self.steering_angle(goal)
        
        return max(min(constant * self.angle_norm(steering_angle - yaw), 3.0), -3.0)

    def move2goal(self, goal):
        vel_msg = Twist()

        distance = self.euclidean_distance(goal)

        if distance <= 0.1:
            return True

        # Porportional controller.
        # https://en.wikipedia.org/wiki/Proportional_control

        # Linear velocity in the x-axis.
        vel_msg.linear.x = self.linear_vel(goal)

        # Angular velocity in the z-axis.
        vel_msg.angular.z = self.angular_vel(goal)

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)
        
        # Publish error
        distance_msg = Float64()
        distance_msg.data = distance
        self.error_publisher.publish(distance_msg)

        return False
            
    def moveAlongPath(self, path):
        if len(path.poses) == 0:
            return
        
        if self.move2goal(path.poses[0].pose.position):
            self.get_logger().info("Goal reached!")
            # Print current position
            self.get_logger().info("Current position: " + str(self.pose.position.x) + ", " + str(self.pose.position.y))
            # Print goal position
            self.get_logger().info("Goal position: " + str(path.poses[0].pose.position.x) + ", " + str(path.poses[0].pose.position.y))
            # Print error between current and goal position (euclidean distance)
            self.get_logger().info("Error: " + str(self.euclidean_distance(path.poses[0].pose.position)))
            
            path.poses.pop(0)
            
            if len(path.poses) == 0:
                self.stop_turtlebot()
                self.get_logger().info("Path completed!")
            else:
                # Print next goal position
                self.get_logger().info("Next goal position: " + str(path.poses[0].pose.position.x) + ", " + str(path.poses[0].pose.position.y))
            

    def stop_turtlebot(self):
        self.velocity_publisher.publish(Twist())

# Main function takes 2 arguments initial position and goal position
def main(args=None):
    rclpy.init(args=args)
    
    # we are using try-except tools to  catch keyboard interrupt
    try:
    	# create an object for GoToPoint class
        cmd_publisher = GoToPoint()
        # continue untill interrupted
        rclpy.spin(cmd_publisher)
    	
    except KeyboardInterrupt:
    	# execute shutdown function
        cmd_publisher.stop_turtlebot()
        # clear the node
        cmd_publisher.destroy_node()
        rclpy.shutdown()
    	
if __name__ == '__main__':
    main()