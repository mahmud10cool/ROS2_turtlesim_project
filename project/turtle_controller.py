#!/usr/bin/env python3

# Importing the ros2 python library
# rcl stands for ROS Client Libraries
import rclpy
# Importing the node library, which does something
from rclpy.node import Node
# Importing the Twist message type for publishing velocity
from geometry_msgs.msg import Twist
# Importing the Pose message type for pose subscription
from turtlesim.msg import Pose
# Importing the math module for pythagorean theorem
import math
# Importing the newly created interface files
from interfaces_robot.msg import*

class TurtleControllerNode(Node):
    def __init__(self):
        # Naming the node
        super().__init__('turtle_controller')
        self.turtle_to_catch_ = None

        # A variable for the pose
        self.pose_ = None
        # Creating a velocity publisher
        self.velocity_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # Creating a subscriber
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.callback_turtle_pose, 10)
        
        # Frequency of the publisher
        timer_period = 0.01

        # Creating the timer
        self.control_loop_timer_ = self.create_timer(timer_period, self.control_loop)

        # Let us write an intializing message
        self.get_logger().info('The turtle controller has now started')

        self.alive_turtles_subscriber_ = self.create_subscription(
            Turtle, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
    
    # Callback function to get the data regarding the pose of the message
    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            self.turtle_to_catch_ = msg.turtles[0]
    
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return 

        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y

        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

        msg = Twist()

        if distance > 0.5:
            # First the position
            msg.linear.x = 2*distance

            # The orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = 6*diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.velocity_publisher_.publish(msg)


# Defining the main function
def main(args=None):
    # Initializing ROS
    rclpy.init(args=args)
    node = TurtleControllerNode()
    # Spinning the node
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
