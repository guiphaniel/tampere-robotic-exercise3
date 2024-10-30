from math import atan2, pi
from math import sqrt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion

class GoToPoint(Node):    
    def __init__(self):
        # Initialize the node
        super().__init__('GoToPointCmd_publisher')
        
        # Initialize the pose
        self.pose = Pose()
        
        # Ask the user to input the goal position
        goal = Pose()
        goal.position.x = float(input("Set your x goal: "))
        goal.position.y = float(input("Set your y goal: "))
        goal.orientation.z = self.angle_norm(float(input("Set your theta goal: ")))
        
        # Initialize the publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, lambda goal=goal: self.move2goal(goal))
        
        # Initialize the subscriber to get the current pose
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.update_odom, 10)

    def update_odom(self, data: Odometry):
        self.pose = data.pose.pose
        print("Current position: ", self.pose.position.x, self.pose.position.y)

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

    def angular_vel(self, steering_angle, constant=1.5):
        # cf. https://answers.ros.org/question/327256/equivalent-turtlesimpose-and-geometry_msgspose/
        
        orientation_list = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                
        return max(min(constant * self.angle_norm(steering_angle - yaw), 3.0), -3.0)

    def move2goal(self, goal: Pose):
        vel_msg = Twist()

        if self.euclidean_distance(goal.position) >= 0.1:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal.position)

            # Angular velocity in the z-axis.
            steering_angle = self.steering_angle(goal.position)

            vel_msg.angular.z = self.angular_vel(steering_angle)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

        else:
            # Stopping our robot and rotating after goal point has been reached.
            # Angular velocity in the z-axis.
            vel_msg.angular.z = self.angular_vel(goal.orientation.z)
            
            self.velocity_publisher.publish(vel_msg)
            
            # Print success and ask for new goal
            print("Goal reached!")

    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
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