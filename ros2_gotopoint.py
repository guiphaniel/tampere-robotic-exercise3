from math import atan2
from math import sqrt
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from turtlesim.msg import Pose


class GoToPoint(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('GoToPointCmd_publisher')
        
        # Declare goal parameters
        self.declare_parameter('goal', Point)
        
        # Get the params
        goal_pose = Pose()
        goal_pose.x = self.get_parameter('goal').x
        goal_pose.y = self.get_parameter('goal').y
        
        # Initialize the publisher
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, lambda goal_pose=goal_pose: self.move2goal(goal_pose))
        
        # Initialize the subscriber to get the current pose
        self.pose_subscriber = self.create_subscription(Pose, '/pose', self.update_pose, 10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self, goal_pose):
        vel_msg = Twist()

        if self.euclidean_distance(goal_pose) >= 0.01:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)

            # Angular velocity in the z-axis.
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

        else:
            # Stopping our robot after the movement is over.
            self.velocity_publisher.publish(vel_msg)

    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
        self.publisher_.publish(Twist())

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